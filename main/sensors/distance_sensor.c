#include "distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>  // For sqrtf() in standard deviation calculation

#if ACTIVE_SENSOR == USE_CONTACT_SENSOR
#include "contact_sensor.h"
#endif

// NVS for pod calibration persistence
#include "nvs_flash.h"
#include "nvs.h"

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
    // Load calibration first for global pod state
    pod_state_load_settings(&s_pod_state);
    ESP_LOGI(TAG, "Pod state initialized: empty_raw=%d mm, full_raw=%d mm, headspace_raw=%d mm",
             s_pod_state.raw_empty_mm, s_pod_state.raw_full_mm, s_pod_state.raw_headspace_mm);

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
#elif ACTIVE_SENSOR == USE_CONTACT_SENSOR
    ESP_LOGI(TAG, "Distance sensor: CONTACT mode via MCP23017");
    err = contact_sensor_init_hw();
#else
    ESP_LOGE(TAG, "No sensor mode defined!");
    return ESP_FAIL;
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize distance sensor hardware: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Distance sensor initialized");
    // Load persisted calibration (already done at start of this function)
    ESP_LOGI(TAG, "Calibration loaded: empty=%d mm, full=%d mm, headspace=%d mm", 
            s_pod_state.raw_empty_mm, s_pod_state.raw_full_mm, s_pod_state.raw_headspace_mm);

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
    static float reading_kf_x = -1.0f;          // current reading estimate (mm)
    static float reading_kf_P = 25.0f;          // reading estimate variance (mm^2)
    static const float reading_kf_Q = 0.07f;     // reading process noise variance
    static const float reading_kf_R = 15.0f;    // individual reading noise variance (higher than averaged)
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

// These functions have been moved to pod_state.c
// Keeping stubs here for backward compatibility
esp_err_t load_pod_settings(void)
{
    ESP_LOGW(TAG, "load_pod_settings is deprecated, use pod_state_load_settings instead");
    return pod_state_load_settings(&s_pod_state);
}

esp_err_t save_pod_settings(int empty_mm, int full_mm)
{
    ESP_LOGW(TAG, "save_pod_settings is deprecated, use pod_state_save_settings instead");
    return pod_state_save_settings(&s_pod_state, empty_mm, full_mm);
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
    // Poll at 2 Hz (every 100ms)
    const TickType_t delay_ticks = pdMS_TO_TICKS(100);
    
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
            
#ifdef USE_CONTACT_SENSOR
            int dist_mm = contact_sensor_read_mm_actual();
#else
            int dist_mm = distance_sensor_read_mm_actual();
#endif

            if (dist_mm >= 0) {
                // Valid raw reading - update cached raw value
                last_valid_distance_mm = dist_mm;
                // The water level is now computed directly in distance_sensor_read_mm_actual
                // No need to recompute - dist_mm already contains final calibrated water level
                last_filtered_distance_mm = dist_mm;
                s_pod_state.current_raw_mm = dist_mm; // Update pod state with latest raw reading
                last_measurement_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                retry_count = 0; // Reset retry counter

                // Print percent value and distance every 5 readings
                if (++log_counter >= 5) {
                    log_counter = 0;
                    int cur_fill_percent = pod_state_calc_fill_percent_int(&s_pod_state);
                    if (cur_fill_percent >= 0) {
                        ESP_LOGD(TAG, "Distance: %d mm, Fill Percent: %d%%", dist_mm, cur_fill_percent);
                    } else {
                        ESP_LOGD(TAG, "Distance: %d mm, Fill Percent: Not calibrated", dist_mm);
                    }
                }


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

    // Use pod_state calibration values
    int raw_empty = s_pod_state.raw_empty_mm;  // Raw distance when empty
    int raw_full = s_pod_state.raw_full_mm;    // Raw distance when full
    int sensor_range = raw_empty - raw_full;   // Actual sensor reading range
    
    // Adjust range to account for headspace
    int adjusted_range = sensor_range;
    if (sensor_range >= s_pod_state.raw_headspace_mm) {
        adjusted_range = sensor_range - s_pod_state.raw_headspace_mm;
        #if SENSOR_VERBOSE_LOGS
        ESP_LOGI(LOCAL_TAG, "Max fill level accounts for headspace: %d -> %d mm (headspace=%d mm)", 
                sensor_range, adjusted_range, s_pod_state.raw_headspace_mm);
        #endif
    } else {
        ESP_LOGW(LOCAL_TAG, "Cannot apply full headspace: range=%d mm < headspace=%d mm", 
                sensor_range, s_pod_state.raw_headspace_mm);
    }
    
    // Convert to depth: raw_empty maps to depth 0, adjusted_range is max depth
    float depth_mm = (float)adjusted_range;
    
    // No scale factor needed now that we're using pod_state calibration
    #if SENSOR_VERBOSE_LOGS
    ESP_LOGI(LOCAL_TAG, "Max fill level: %d mm (adjusted from %d mm)",
            adjusted_range, sensor_range);
    #endif
    return adjusted_range;
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
    // Use pod_state calibration values
    int raw_empty = s_pod_state.raw_empty_mm;  // Raw distance when empty
    int raw_full = s_pod_state.raw_full_mm;    // Raw distance when full
    int sensor_range = raw_empty - raw_full;   // Actual sensor reading range
    
    return sensor_range;
}
