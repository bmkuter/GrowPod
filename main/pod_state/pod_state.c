#include "pod_state.h"
#include "power_monitor_HAL.h"
#include "distance_sensor.h"  // Include for TANK_HEADSPACE_MM
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

// Variables
static const char *TAG = "POD_STATE";
pod_state_t s_pod_state = {
    .raw_empty_mm = 0,
    .raw_full_mm = 9999,  // Default to a large value until calibrated
    .current_raw_mm = 0,
    .raw_headspace_mm = TANK_HEADSPACE_MM,
    .voltage_mV = 0.0f, 
    .current_mA = 0.0f, 
    .power_mW = 0.0f,
    .calibrated = false   // Default to not calibrated
};

void pod_state_init(pod_state_t *state, int empty_raw_mm, int full_raw_mm, int current_raw_mm) {
    // Initialize calibration and current values
    state->raw_empty_mm   = empty_raw_mm;
    state->raw_full_mm    = full_raw_mm;
    state->current_raw_mm = current_raw_mm;

    // Initialize power readings to zero
    state->voltage_mV = 0.0f;
    state->current_mA = 0.0f;
    state->power_mW   = 0.0f;
    
    // Set calibrated flag to true if valid calibration values are provided
    // Consider calibration valid if empty > full (because sensor distance decreases as water rises)
    state->calibrated = (empty_raw_mm > full_raw_mm);
    
    if (state->calibrated) {
        ESP_LOGI(TAG, "Pod initialized with valid calibration: empty=%d mm, full=%d mm", 
                empty_raw_mm, full_raw_mm);
    } else {
        ESP_LOGW(TAG, "Pod initialized with invalid calibration: empty=%d mm, full=%d mm",
                empty_raw_mm, full_raw_mm);
    }
}

void pod_state_update_measurement(pod_state_t *state, int raw_mm) {
    // Update the latest raw sensor reading
    state->current_raw_mm = raw_mm;
}

void pod_state_update_power(pod_state_t *state) {
    // Read from power monitor HAL and update state
    power_monitor_read_voltage(&state->voltage_mV);
    power_monitor_read_current(&state->current_mA);
    power_monitor_read_power(&state->power_mW);
}

int pod_state_calc_fill_percent_int(const pod_state_t *state) {
    // Check if pod is calibrated
    if (!state->calibrated) {
        ESP_LOGD(TAG, "Cannot calculate fill percentage: pod not calibrated");
        return -1; // Return error if not calibrated
    }

    int empty_raw_mm = state->raw_empty_mm;
    int raw_mm       = state->current_raw_mm;

    // Effective full threshold is largest of full or headspace
    const int full_threshold_mm = (state->raw_headspace_mm < state->raw_full_mm) 
        ? state->raw_full_mm 
        : state->raw_headspace_mm;

    // Calculate range between empty and headspace threshold
    int effective_range = empty_raw_mm - full_threshold_mm;

    // Log calculation details
    ESP_LOGD(TAG, "Calculating fill percent: empty_raw_mm=%d, full_threshold_mm=%d, effective_range=%d, raw_mm=%d",
             empty_raw_mm, full_threshold_mm, effective_range, raw_mm);

    if (effective_range <= 0) {
        // Log error, printing all variables
        ESP_LOGW(TAG, "Invalid effective range for fill percentage calculation. empty_raw_mm=%d, full_threshold_mm=%d, effective_range=%d", 
                 empty_raw_mm, full_threshold_mm, effective_range);
        return -1; // invalid calibration or headspace too large
    }

    // Clamp raw reading to headspace threshold
    if (raw_mm <= full_threshold_mm) {
        return 100; // at or above full level
    }
    if (raw_mm >= empty_raw_mm) {
        return 0;   // at or below empty level
    }

    // Calculate filled millimeters and percentage
    int filled_mm = empty_raw_mm - raw_mm;
    int pct = (filled_mm * 100) / effective_range;
    // log math and related variables
    ESP_LOGD(TAG, "Filled mm: %d, Percentage: %d%% (raw_mm=%d, effective_range=%d)", 
             filled_mm, pct, raw_mm, effective_range);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

/**
 * @brief Load pod calibration settings from NVS (pod_settings namespace)
 */
esp_err_t pod_state_load_settings(pod_state_t *state)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pod_settings", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        int32_t empty = state->raw_empty_mm;
        if (nvs_get_i32(handle, "empty_height", &empty) == ESP_OK) {
            state->raw_empty_mm = empty;
            ESP_LOGI(TAG, "Loaded pod empty height: %d mm", state->raw_empty_mm);
        } else {
            ESP_LOGW(TAG, "Pod empty height not in NVS, defaulting to %d mm", state->raw_empty_mm);
        }
        
        int32_t full = state->raw_full_mm;
        if (nvs_get_i32(handle, "full_height", &full) == ESP_OK) {
            state->raw_full_mm = full;
            ESP_LOGI(TAG, "Loaded pod full height: %d mm", state->raw_full_mm);
        } else {
            ESP_LOGW(TAG, "Pod full height not in NVS, defaulting to %d mm", state->raw_full_mm);
        }
        
        int32_t headspace = state->raw_headspace_mm;
        if (nvs_get_i32(handle, "headspace_mm", &headspace) == ESP_OK) {
            state->raw_headspace_mm = headspace;
            ESP_LOGI(TAG, "Loaded sensor headspace: %d mm", state->raw_headspace_mm);
        } else {
            state->raw_headspace_mm = TANK_HEADSPACE_MM;
            ESP_LOGW(TAG, "Sensor headspace not in NVS, defaulting to %d mm", state->raw_headspace_mm);
        }
        
        uint8_t is_calibrated = 0;
        if (nvs_get_u8(handle, "calibrated", &is_calibrated) == ESP_OK) {
            state->calibrated = (is_calibrated != 0);
            ESP_LOGI(TAG, "Loaded calibration state: %s", state->calibrated ? "calibrated" : "not calibrated");
        } else {
            // If the calibrated flag is not in NVS, determine based on loaded values
            // Consider calibration valid if empty > full (because sensor distance decreases as water rises)
            state->calibrated = (state->raw_empty_mm > state->raw_full_mm);
            ESP_LOGW(TAG, "Calibration state not in NVS, determined as: %s", 
                    state->calibrated ? "calibrated" : "not calibrated");
        }
        
        nvs_close(handle);
    } else {
        ESP_LOGW(TAG, "Failed to open pod_settings namespace: %s", esp_err_to_name(err));
        // Setting Safe Defaults 
        pod_state_init(state, TANK_EMPTY_HEIGHT_MM, TANK_HEADSPACE_MM, distance_sensor_read_mm());
        state->calibrated = false; // Not calibrated if we can't load settings
    }
    return err;
}

/**
 * @brief Save pod calibration settings to NVS (pod_settings namespace)
 */
esp_err_t pod_state_save_settings(pod_state_t *state, int empty_mm, int full_mm)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pod_settings", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        // Update calibration parameters in RAM
        state->raw_empty_mm = empty_mm;
        state->raw_full_mm = full_mm;
        
        // Save to NVS
        nvs_set_i32(handle, "empty_height", empty_mm);
        nvs_set_i32(handle, "full_height", full_mm);
        nvs_set_i32(handle, "headspace_mm", state->raw_headspace_mm);
        nvs_set_u8(handle, "calibrated", state->calibrated ? 1 : 0);
        
        ESP_LOGI(TAG, "Calibration saved: empty=%d mm, full=%d mm, headspace=%d mm, calibrated=%d",
                empty_mm, full_mm, state->raw_headspace_mm, state->calibrated);
        
        nvs_commit(handle);
        nvs_close(handle);
    }
    return err;
}
