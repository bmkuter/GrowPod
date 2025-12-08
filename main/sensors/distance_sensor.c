#include "distance_sensor.h"
#include "fdc1004_distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

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

//===========================================================
// FDC1004 Capacitive Sensor Initialization
//===========================================================
esp_err_t distance_sensor_init(void)
{
    esp_err_t err = ESP_OK;
    
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
    
    // Initialize FDC1004 capacitive distance sensor
    ESP_LOGI(TAG, "Initializing FDC1004 capacitive distance sensor...");
    err = fdc1004_init();
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FDC1004 sensor: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "FDC1004 distance sensor initialized successfully");
    ESP_LOGI(TAG, "Calibration loaded: empty=%d mm, full=%d mm, headspace=%d mm", 
            s_pod_state.raw_empty_mm, s_pod_state.raw_full_mm, s_pod_state.raw_headspace_mm);

    return ESP_OK;
}

/**
 * @brief Take an actual reading from the distance sensor hardware
 * 
 * This is an internal function that accesses the FDC1004 sensor directly.
 * It should only be called when the mutex is held to prevent concurrent access.
 * 
 * @return Distance in mm, or negative value if error
 */
static int distance_sensor_read_mm_actual(void)
{
    // Read distance from FDC1004 capacitive sensor
    int dist_mm = fdc1004_read_distance_mm();
    
    if (dist_mm < 0) {
        ESP_LOGW(TAG, "Failed to read FDC1004 distance sensor");
        return -1;
    }
    
    ESP_LOGD(TAG, "FDC1004 distance: %d mm", dist_mm);
    return dist_mm;
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
            // Take a direct sensor reading from FDC1004
            int dist_mm = distance_sensor_read_mm_actual();

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
