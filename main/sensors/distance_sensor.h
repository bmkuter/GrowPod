#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "esp_err.h"
#include "pod_state.h"  // For pod state management

#ifdef __cplusplus
extern "C" {
#endif

// Height from sensor to bottom of tank (empty), mm
#define TANK_EMPTY_HEIGHT_MM 87
#define TANK_DISTANCE_SENSOR_OFFSET_MM 20

// Headspace to maintain at top of tank (mm)
#define TANK_HEADSPACE_MM 25

// Sentinel value for invalid water level readings
#define WATER_LEVEL_ERROR_MM (-10000)

// Use FDC1004 capacitive sensor as the backend
#include "fdc1004_distance_sensor.h"

/**
 * @brief Initialize the distance sensor subsystem (laser sensor).
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t distance_sensor_init(void);

/**
 * @brief Task for distance sensor background measurement loop.
 *        Create this in app_main() or somewhere after initialization.
 *        This task continuously updates the cached distance measurement.
 * 
 * The task runs continuously in the background and:
 * 1. Updates the cached distance measurement every 100ms
 * 2. Logs distance information at appropriate levels
 * 3. Will eventually provide an event notification system for distance changes
 * 
 * @param pvParameters FreeRTOS task parameters (not used)
 */
void distance_sensor_task(void *pvParameters);

/**
 * @brief Get the current cached distance reading in millimeters.
 * 
 * This function returns the latest distance measurement from the background
 * task without accessing the sensor hardware directly. It is non-blocking
 * and safe to call from any task or ISR.
 * 
 * The background task updates this value every 100ms, so readings are always
 * fresh. The first valid reading may take up to 2 seconds after initialization.
 * 
 * @note This function is non-blocking and returns immediately
 * 
 * @return Latest distance in mm, or negative value if no valid reading available
 */
int distance_sensor_read_mm(void);

/**
 * @brief Read the raw sensor distance measurement (mm) without calibration
 * 
 * @return Raw sensor distance in mm, or negative on error
 */
int distance_sensor_read_raw_mm(void);

/**
 * @brief Load pod calibration settings from NVS.
 *
 * @return ESP_OK on success, or error code if loading failed
 */
esp_err_t load_pod_settings(void);

/**
 * @brief Save pod calibration settings to NVS.
 *
 * @param empty_mm Empty height in mm
 * @param full_mm Full height in mm
 * @return ESP_OK on success, or error code if saving failed
 */
esp_err_t save_pod_settings(int empty_mm, int full_mm);

/**
 * @brief Get maximum calibrated water level in mm that accounts for headspace.
 * This is the target level for 100% fill, which includes safety headspace.
 * @return Maximum safe water level in mm
 */
int distance_sensor_get_max_level_mm(void);

/**
 * @brief Get absolute maximum water level in mm (physical limit).
 * This should only be used for reporting the physical limits, not for fill targets.
 * @return Absolute maximum water level in mm (no headspace)
 */
int distance_sensor_get_absolute_max_level_mm(void);

/**
 * @brief Set the global sensor scale factor used for level conversion.
 * @param factor Scale factor (actual/raw)
 */
void distance_sensor_set_scale_factor(float factor);

#ifdef __cplusplus
}
#endif

#endif // DISTANCE_SENSOR_H
