#ifndef POD_STATE_H
#define POD_STATE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure to hold food pump calibration data.
 */
typedef struct {
    float flow_rate_mg_per_ms;      // Measured flow rate (mg per millisecond)
    uint32_t test_duration_ms;      // Duration of calibration test (ms)
    uint8_t test_speed_percent;     // Speed % used during test
    float test_output_mg;           // Measured output during test (mg)
    uint64_t calibration_timestamp; // Unix timestamp of calibration
    bool calibrated;                // Flag indicating if pump has been calibrated
    char notes[128];                // Calibration notes (test conditions, etc.)
} food_pump_calibration_t;

/**
 * @brief Structure to hold pod state measurements and calibration.
 */
typedef struct {
    int raw_empty_mm;    // Raw sensor distance when pod is empty
    int raw_full_mm;     // Raw sensor distance when pod is full

    int raw_headspace_mm; // Raw sensor distance for headspace (if applicable)

    int current_raw_mm;  // Latest raw sensor distance reading

    float voltage_mV;    // Latest power monitor voltage reading
    float current_mA;    // Latest power monitor current reading
    float power_mW;      // Latest power monitor power reading
    
    bool calibrated;     // Flag indicating if pod has been calibrated
    
    // Food pump calibration
    food_pump_calibration_t food_pump_cal;
} pod_state_t;

// Add global pod state instance
extern pod_state_t s_pod_state;

/**
 * @brief Initialize pod state with calibration values.
 * Sets state->calibrated to true if valid calibration parameters (empty_raw_mm > full_raw_mm).
 *
 * @param state Pointer to pod_state_t to initialize
 * @param empty_raw_mm Raw sensor distance at empty reference
 * @param full_raw_mm Raw sensor distance at full reference
 * @param current_raw_mm Current raw sensor distance reading
 */
void pod_state_init(pod_state_t *state, int empty_raw_mm, int full_raw_mm, int current_raw_mm);

/**
 * @brief Update the current raw measurement in pod state.
 *
 * @param state Pointer to pod_state_t
 * @param raw_mm New raw sensor distance reading
 */
void pod_state_update_measurement(pod_state_t *state, int raw_mm);

/**
 * @brief Update power measurements in pod state from power monitor API.
 *
 * @param state Pointer to pod_state_t
 */
void pod_state_update_power(pod_state_t *state);

/**
 * @brief Calculate fill percentage as an integer based on pod state calibration and current reading.
 * Requires the pod to be calibrated (state->calibrated must be true).
 *
 * @param state Pointer to pod_state_t containing calibration and current raw reading
 * @return Fill percent (0-100) on success, or -1 if pod is not calibrated or if calibration is invalid
 */
int pod_state_calc_fill_percent_int(const pod_state_t *state);

/**
 * @brief Load pod calibration settings from NVS (pod_settings namespace)
 * Loads empty_height, full_height, headspace_mm, and calibrated flag.
 * Sets state->calibrated to true if valid calibration data was loaded.
 * 
 * @param state Pointer to pod_state_t to update with loaded calibration
 * @return ESP_OK on success, or error code if loading failed
 */
esp_err_t pod_state_load_settings(pod_state_t *state);

/**
 * @brief Save pod calibration settings to NVS (pod_settings namespace)
 * Saves empty_height, full_height, headspace_mm, and calibrated flag.
 * Sets state->calibrated to true if valid calibration parameters are provided.
 *
 * @param state Pointer to pod_state_t containing calibration to save
 * @param empty_mm Empty height in mm
 * @param full_mm Full height in mm
 * @return ESP_OK on success, or error code if saving failed
 */
esp_err_t pod_state_save_settings(pod_state_t *state, int empty_mm, int full_mm);

/**
 * @brief Load food pump calibration from filesystem (LittleFS).
 * Reads calibration data from /littlefs/pump_calibration.json
 * Sets food_pump_cal.calibrated to true if valid calibration data was loaded.
 * 
 * @param state Pointer to pod_state_t to update with loaded calibration
 * @return ESP_OK on success, or error code if loading failed
 */
esp_err_t pod_state_load_pump_calibration(pod_state_t *state);

/**
 * @brief Save food pump calibration to filesystem (LittleFS).
 * Writes calibration data to /littlefs/pump_calibration.json
 * 
 * @param cal Pointer to food_pump_calibration_t containing calibration to save
 * @return ESP_OK on success, or error code if saving failed
 */
esp_err_t pod_state_save_pump_calibration(const food_pump_calibration_t *cal);

/**
 * @brief Calculate expected dose output based on pump calibration.
 * Uses flow_rate_mg_per_ms and applies speed scaling.
 * 
 * @param cal Pointer to food_pump_calibration_t
 * @param duration_ms Dose duration in milliseconds
 * @param speed_percent Dose speed percentage (0-100)
 * @return Expected output in mg, or -1.0 if not calibrated
 */
float pod_state_calc_pump_dose_mg(const food_pump_calibration_t *cal, uint32_t duration_ms, uint8_t speed_percent);

#ifdef __cplusplus
}
#endif

#endif // POD_STATE_H
