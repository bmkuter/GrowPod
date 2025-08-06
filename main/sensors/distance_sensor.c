#include "distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>  // For sqrtf() in standard deviation calculation

// NVS for pod calibration persistence
#include "nvs_flash.h"
#include "nvs.h"

// Define which sensor implementation to use
#define USE_LASER_SENSOR 1        // Original laser sensor implementation

// Enable or disable verbose sensor reading logs
#define SENSOR_VERBOSE_LOGS 1  // Set to 1 to enable detailed sensor logs

// Choose which implementation to use - set only one to 1
#define ACTIVE_SENSOR USE_LASER_SENSOR

static const char *TAG = "DIST_SENSOR";

// Mutex to protect access to the distance sensor
static SemaphoreHandle_t sensor_mutex = NULL;

// Last valid measurement and timestamp (for caching)
static volatile int last_valid_distance_mm = -1;
static volatile uint32_t last_measurement_time = 0;

// Last filtered distance measurement
static volatile int last_filtered_distance_mm = WATER_LEVEL_ERROR_MM;

// Dynamic calibration parameters
static int sensor_offset_mm = 0;           // default laser offset in mm (fallback until NVS load)
static int pod_empty_height_mm = TANK_EMPTY_HEIGHT_MM;
static int pod_full_height_mm = 0;          // set after load or calibration

// Calibration scale factor (actual/raw), default=1.0
static float sensor_scale_factor = 1.0f;

//===========================================================
// If using laser sensor (I2C)
#if defined(USE_LASER_SENSOR)

#include "driver/i2c.h"
#include "driver/i2c_master.h"

// Laser sensor I2C address
#define LASER_I2C_ADDRESS   0x74

// Write register helper
static esp_err_t laser_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    // We need to send [reg][data...]
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    // Write to the device in one shot:
    return i2c_master_write_to_device(
        I2C_NUM_0,
        LASER_I2C_ADDRESS, 
        buf, 
        len + 1, 
        pdMS_TO_TICKS(50)
    );
}

// Read register helper
static esp_err_t laser_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    // 1) Write the register address
    // 2) Then read 'len' bytes into 'data'

    // The new API does this in one line:
    return i2c_master_write_read_device(
        I2C_NUM_0,
        LASER_I2C_ADDRESS,
        &reg,  // pointer to register address
        1,     // size of register address
        data, 
        len, 
        pdMS_TO_TICKS(50)
    );
}


static int laser_get_distance_mm(void)
{
    // Command the laser to start measurement
    uint8_t dat = 0xB0;
    esp_err_t ret = laser_write_reg(0x10, &dat, 1);
    if (ret != ESP_OK) {
        return -1; 
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for measurement to complete

    // Read 2 bytes from register 0x02
    uint8_t buf[2];
    ret = laser_read_reg(0x02, buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read distance from laser sensor: %s", esp_err_to_name(ret));
        return -1;
    }

    // Get raw value without any calibration
    int raw = ((int)buf[0] << 8) | buf[1];
    ESP_LOGD(TAG, "Raw laser sensor reading: %d mm", raw);
    
    // Check for obviously invalid readings
    if (raw <= 0) {
        ESP_LOGW(TAG, "Invalid sensor raw reading: %d mm", raw);
        return -1;
    }
    
    return raw;
}


static esp_err_t laser_init_hw(void) {
    // Assume i2c_master_init() is already called in app_main
    return ESP_OK;
}

#endif // USE_LASER_SENSOR

//===========================================================
// Common entry points
//===========================================================
esp_err_t distance_sensor_init(void)
{
    uint32_t err = ESP_OK;
    // Load calibration first
    load_pod_settings();

    // Create mutex for thread-safe access to sensor
    if (sensor_mutex == NULL) {
        sensor_mutex = xSemaphoreCreateMutex();
        if (sensor_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create sensor mutex");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Sensor access mutex created");
    }
    
    // Initialize hardware based on selected sensor
#if ACTIVE_SENSOR == USE_LASER_SENSOR
    ESP_LOGI(TAG, "Distance sensor: LASER mode (legacy), I2C=0x74");
    err = laser_init_hw();
#else
    ESP_LOGE(TAG, "No sensor mode defined!");
    return ESP_FAIL;
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize distance sensor hardware: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Distance sensor initialized");
    // Load persisted calibration
    load_pod_settings();    
    ESP_LOGI(TAG, "Calibration loaded: empty=%d mm, full=%d mm, offset=%d mm, scale=%.3f", 
            pod_empty_height_mm, pod_full_height_mm, sensor_offset_mm, (double)sensor_scale_factor);

    return ESP_OK;
}

/**
 * @brief Take an actual reading from the distance sensor hardware
 * 
 * This is an internal function that accesses the sensor hardware directly.
 * It should only be called when the mutex is held to prevent concurrent access.
 * 
 * @note This function takes about 400ms to complete (due to averaging multiple readings)
 * @return Distance in mm, or negative value if error
 */
static int distance_sensor_read_mm_actual(void)
{
    // Per-reading Kalman filter state variables
    static float reading_kf_x = -1.0f;       // current reading estimate (mm)
    static float reading_kf_P = 25.0f;       // reading estimate variance (mm^2)
    static const float reading_kf_Q = 0.01f; // reading process noise variance
    static const float reading_kf_R = 100.0f; // individual reading noise variance (higher than averaged)
    static bool reading_kf_initialized = false; // Flag to check if filter is initialized

    const int avg_count = 4;
    const int max_acceptable_distance = 400; // Maximum acceptable distance in mm
    const int min_acceptable_distance = 10;  // Minimum acceptable distance in mm
    
    int readings[avg_count];  // Store individual readings for analysis
    int sum = 0;
    int valid_readings = 0;
    int retries = 0;
    const int max_retries = 3; // Max number of retries for anomalous readings

    #if SENSOR_VERBOSE_LOGS
    ESP_LOGD(TAG, "Starting laser distance measurement (averaging %d readings)", avg_count);
    #endif
    
    // Take multiple readings and apply Kalman filter to each one before averaging
    for (int i = 0; i < avg_count; i++) {
        int retry_count = 0;
        int raw_tmp;
        int filtered_tmp;
        
        // Try up to max_retries times to get a reading within acceptable range
        do {
            raw_tmp = laser_get_distance_mm();
            
            // Apply Kalman filter to each individual raw reading
            if (raw_tmp > 0) {
                // Initialize per-reading filter if needed
                if (!reading_kf_initialized) {
                    reading_kf_x = (float)raw_tmp;
                    reading_kf_P = reading_kf_R;
                    reading_kf_initialized = true;
                    filtered_tmp = raw_tmp;
                } else {
                    // Standard Kalman filter update
                    float P_pred = reading_kf_P + reading_kf_Q;
                    float K = P_pred / (P_pred + reading_kf_R);
                    reading_kf_x = reading_kf_x + K * ((float)raw_tmp - reading_kf_x);
                    reading_kf_P = (1.0f - K) * P_pred;
                    filtered_tmp = (int)(reading_kf_x + 0.5f);
                    
                    #if SENSOR_VERBOSE_LOGS
                    ESP_LOGD(TAG, "Reading #%d: RaD=%d mm, K=%.3f, Filtered=%d", 
                             i+1, raw_tmp, (double)K, filtered_tmp);
                    #endif
                }
            } else {
                // Invalid reading, pass through
                filtered_tmp = raw_tmp;
            }
            
            // Check if filtered reading is out of acceptable range but technically valid
            if (filtered_tmp > 0 && (filtered_tmp > max_acceptable_distance || filtered_tmp < min_acceptable_distance)) {
                ESP_LOGW(TAG, "Anomalous reading #%d: %d mm (raw=%d mm) (outside %d-%d mm range), retry %d/%d", 
                        i+1, filtered_tmp, raw_tmp, min_acceptable_distance, max_acceptable_distance, 
                        retry_count+1, max_retries);
                
                // Only count as a retry if it's out of range but > 0
                retry_count++;
                retries++;
                vTaskDelay(pdMS_TO_TICKS(10)); // Slightly longer delay before retry
            } else {
                // Either good reading or error (< 0)
                break;
            }
        } while (retry_count < max_retries);
        
        if (filtered_tmp > 0 && filtered_tmp <= max_acceptable_distance && filtered_tmp >= min_acceptable_distance) {  
            // Valid reading within acceptable range
            readings[valid_readings] = filtered_tmp;
            sum += filtered_tmp;
            valid_readings++;
            #if SENSOR_VERBOSE_LOGS
            ESP_LOGD(TAG, "ReadDng #%d: %d mm (filtered from raw=%d mm) (valid)", 
                    i+1, filtered_tmp, raw_tmp);
            #endif
        } else if (filtered_tmp > 0) {
            // Still out of range after retries
            ESP_LOGW(TAG, "Reading #%d: %d mm (filtered from raw=%d mm) (out of range, discarded)", 
                    i+1, filtered_tmp, raw_tmp);
        } else {
            // Error reading
            ESP_LOGW(TAG, "Invalid laser reading attempt (%d) %d/%d", filtered_tmp, i+1, avg_count);
        }
        
        // Small delay between readings
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (valid_readings >= avg_count/2) {
        int raw_result = sum / valid_readings;
        return raw_result;
    } else {
        ESP_LOGW(TAG, "Too few valid readings (%d/%d), returning error", valid_readings, avg_count);
        return -1;
    }
    ESP_LOGE(TAG, "Too many invalid readings (%d/%d failed)", avg_count - valid_readings, avg_count);
    return -1;
}

int distance_sensor_read_mm(void)
{
    // Return the last processed cached water level
    return last_filtered_distance_mm;
}

/**
 * @brief Load pod calibration settings from NVS (pod_settings namespace)
 */
esp_err_t load_pod_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pod_settings", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        int32_t empty = pod_empty_height_mm;
        if (nvs_get_i32(handle, "empty_height", &empty) == ESP_OK) {
            pod_empty_height_mm = empty;
            ESP_LOGI(TAG, "Loaded pod empty height: %d mm", pod_empty_height_mm);
        } else {
            ESP_LOGW(TAG, "Pod empty height not in NVS, defaulting to %d mm", pod_empty_height_mm);
        }
        int32_t full = pod_full_height_mm;
        if (nvs_get_i32(handle, "full_height", &full) == ESP_OK) {
            pod_full_height_mm = full;
            ESP_LOGI(TAG, "Loaded pod full height: %d mm", pod_full_height_mm);
        } else {
            ESP_LOGW(TAG, "Pod full height not in NVS, defaulting to %d mm", pod_full_height_mm);
        }
        int32_t offset = sensor_offset_mm;
        if (nvs_get_i32(handle, "offset_mm", &offset) == ESP_OK) {
            sensor_offset_mm = offset;
            ESP_LOGI(TAG, "Loaded sensor offset: %d mm", sensor_offset_mm);
        } else {
            ESP_LOGW(TAG, "Sensor offset not in NVS, defaulting to %d mm", sensor_offset_mm);
        }
        // Load scale factor (stored as int32 scale*1000)
        int32_t sf = 1000;
        if (nvs_get_i32(handle, "scale_factor", &sf) == ESP_OK) {
            sensor_scale_factor = ((float)sf) / 1000.0f;
            ESP_LOGI(TAG, "Loaded sensor scale factor: %.3f", (double)sensor_scale_factor);
        } else {
            ESP_LOGW(TAG, "Sensor scale factor not in NVS, defaulting to %.3f", (double)sensor_scale_factor);
        }
        nvs_close(handle);
    } else {
        ESP_LOGW(TAG, "Failed to open pod_settings namespace: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 * @brief Save pod calibration settings to NVS (pod_settings namespace)
 */
esp_err_t save_pod_settings(int empty_mm, int full_mm)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pod_settings", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        // Update calibration parameters in RAM
        // empty_mm is the actual sensor reading when the tank is empty
        // TANK_EMPTY_HEIGHT_MM is the physical measurement we expect
        
        // Store the raw readings
        pod_empty_height_mm = empty_mm;
        pod_full_height_mm = full_mm;
        
        // Calculate the offset between actual sensor reading and physical expectation
        // This offset accounts for sensor non-linearity
        sensor_offset_mm = empty_mm - TANK_EMPTY_HEIGHT_MM;
        ESP_LOGI(TAG, "Calibration: sensor reads %d mm when physically it's %d mm (offset=%d mm)", 
                empty_mm, TANK_EMPTY_HEIGHT_MM, sensor_offset_mm);
        
        // Save to NVS
        nvs_set_i32(handle, "empty_height", empty_mm);
        nvs_set_i32(handle, "full_height", full_mm);
        nvs_set_i32(handle, "offset_mm", sensor_offset_mm);
        // Persist scale factor as int thousandths
        int32_t sf = (int32_t)(sensor_scale_factor * 1000.0f);
        nvs_set_i32(handle, "scale_factor", sf);
        ESP_LOGI(TAG, "Saved sensor scale factor: %.3f", (double)sensor_scale_factor);
        nvs_commit(handle);
        nvs_close(handle);
    }
    return err;
}

/**
 * @brief Background task that periodically updates the distance sensor reading
 * 
 * This task runs continuously in the background and:
 * 1. Updates the cached distance measurement at a regular interval
 * 2. Logs distance information at appropriate levels
 * 
 * @param pvParameters FreeRTOS task parameters (not used)
 */
void distance_sensor_task(void *pvParameters)
{
    // Poll at 10 Hz (every 100ms)
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000);
    
    // // Wait a bit before starting to let other initializations complete
    // vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Distance sensor background task started");
    
    // Counter for retry attempts when reading fails
    int retry_count = 0;
    // Counter for logging (to reduce spam)
    int log_counter = 0;
    // Variable to track significant distance changes
    int last_reported_distance = -1;
    
    while (1) {
        // Try to acquire mutex with a shorter timeout since this is a background task
        if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Take a direct sensor reading (already Kalman filtered internally)
            int dist_mm = distance_sensor_read_mm_actual();
            
            if (dist_mm >= 0) {
                // Valid raw reading - update cached raw value
                last_valid_distance_mm = dist_mm;
                // The water level is now computed directly in distance_sensor_read_mm_actual
                // No need to recompute - dist_mm already contains final calibrated water level
                last_filtered_distance_mm = dist_mm;
                last_measurement_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                retry_count = 0; // Reset retry counter
                
                // Check for significant distance changes (> 5mm or 5% change)
                int distance_diff = abs(dist_mm - last_reported_distance);
                if (last_reported_distance < 0 || 
                    distance_diff > 5 || 
                    (last_reported_distance > 0 && distance_diff > (last_reported_distance / 20))) {
                    
                    ESP_LOGD(TAG, "Distance change: %d mm (prev=%d mm, diff=%d mm)",
                             dist_mm, last_reported_distance, distance_diff);
                    
                    // TODO: Implement event notification for significant distance changes
                    // This will allow other subsystems to subscribe to distance change events
                    // rather than continuously polling the sensor
                    
                    last_reported_distance = dist_mm;
                }
            } else {
                // Invalid reading
                retry_count++;
                ESP_LOGW(TAG, "Invalid distance reading: %d (%d consecutive failures)", dist_mm, retry_count);
                vTaskDelay(pdMS_TO_TICKS(10000)); // Wait before retrying
                
                // If we've had too many consecutive failures, log an error
                if (retry_count >= 5) {
                    ESP_LOGE(TAG, "Distance sensor may be disconnected or faulty");
                }
            }
            
            // Release the mutex
            xSemaphoreGive(sensor_mutex);
            
        } else {
            // Couldn't acquire mutex - sensor is busy
            ESP_LOGW(TAG, "Background task couldn't acquire sensor mutex - sensor busy");
        }
        
        // Wait before next reading
        vTaskDelay(delay_ticks);
    }
}

/**
 * @brief Get minimum calibrated water level (empty) in mm
 * @return Minimum water level (should be 0)
 */
int distance_sensor_get_min_level_mm(void)
{
    return 0;
}

/**
 * @brief Get maximum calibrated water level (full) in mm
 * @return Maximum water level in mm based on calibration
 */
/**
 * @brief Get maximum calibrated water level in mm, accounting for headspace
 * 
 * This function returns the maximum level that should be targeted during
 * fill operations, which is the actual tank height minus the headspace.
 * 
 * @return Maximum safe water level in mm, accounting for headspace
 */
int distance_sensor_get_max_level_mm(void)
{
    // Set TAG to be function name for logging
    static const char *LOCAL_TAG = "DIST_SENSOR_MAX_LEVEL";

    // Our physical calibration reference points
    int raw_empty = pod_empty_height_mm;  // Physical distance when empty
    
    // The actual sensor calibration values, which may differ from physical expectations
    int sensor_empty = raw_empty + sensor_offset_mm;    // Actual sensor reading when empty
    int sensor_full = pod_full_height_mm;               // Actual sensor reading when full
    int sensor_range = sensor_empty - sensor_full;      // Actual sensor reading range
    
    // Adjust range to account for headspace
    int adjusted_range = sensor_range;
    if (sensor_range >= (TANK_EMPTY_HEIGHT_MM - TANK_HEADSPACE_MM)) {
        adjusted_range = sensor_range - TANK_HEADSPACE_MM;
        #if SENSOR_VERBOSE_LOGS
        ESP_LOGI(LOCAL_TAG, "Max fill level accounts for headspace: %d -> %d mm (headspace=%d mm)", 
                sensor_range, adjusted_range, TANK_HEADSPACE_MM);
        #endif
    } else {
        ESP_LOGW(LOCAL_TAG, "Cannot apply full headspace: range=%d mm < headspace=%d mm", 
                sensor_range, TANK_HEADSPACE_MM);
    }
    
    // Convert to depth: sensor_empty maps to depth 0, adjusted_range is max depth
    float depth_mm = (float)adjusted_range;
    
    // Apply scale factor (the ratio between actual physical measurements and sensor readings)
    float max_level = depth_mm * sensor_scale_factor;
    #if SENSOR_VERBOSE_LOGS
    ESP_LOGI(LOCAL_TAG, "Max fill level: %d mm (adjusted from %d mm with scale factor %.3f)", 
            (int)(max_level + 0.5f), adjusted_range, (double)sensor_scale_factor);
    #endif
    return (int)(max_level + 0.5f);
}

/**
 * @brief Set the global sensor scale factor used for level conversion
 * @param factor Scale factor (actual/raw)
 */
void distance_sensor_set_scale_factor(float factor)
{
    sensor_scale_factor = factor;
}

/**
 * @brief Get the absolute maximum water level the tank can hold in mm
 * 
 * This returns the absolute maximum water level (without considering headspace),
 * which should only be used for reporting the physical limits. For fill targets,
 * use distance_sensor_get_max_level_mm() which accounts for the safety headspace.
 * 
 * @return Absolute maximum water level in mm (no headspace)
 */
int distance_sensor_get_absolute_max_level_mm(void)
{
    // Our physical calibration reference points
    int raw_empty = pod_empty_height_mm;  // Physical distance when empty
    
    // The actual sensor calibration values, which may differ from physical expectations
    int sensor_empty = raw_empty + sensor_offset_mm;  // Actual sensor reading when empty
    int sensor_full = pod_full_height_mm;            // Actual sensor reading when full
    int sensor_range = sensor_empty - sensor_full;   // Actual sensor reading range
    
    // Convert to depth: sensor_empty maps to depth 0, sensor_range is max depth
    float depth_mm = (float)sensor_range;
    
    // Apply scale factor (the ratio between actual physical measurements and sensor readings)
    float max_level = depth_mm * sensor_scale_factor;
    return (int)(max_level + 0.5f);
}
