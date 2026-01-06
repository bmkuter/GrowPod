#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include "esp_err.h"
#include "fdc1004_distance_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configuration file path
#define SENSOR_CONFIG_FILE "/lfs/config/sensors.json"

/**
 * @brief Save FDC1004 water level sensor calibration to filesystem
 * 
 * Saves calibration data to /lfs/config/sensors.json for persistence
 * across reboots.
 * 
 * @param calibration Pointer to calibration data to save
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_config_save_water_calibration(const fdc1004_calibration_t *calibration);

/**
 * @brief Load FDC1004 water level sensor calibration from filesystem
 * 
 * Loads calibration data from /lfs/config/sensors.json and applies it
 * to the sensor driver.
 * 
 * @param calibration Pointer to store loaded calibration data
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if file doesn't exist,
 *         other error codes on failure
 */
esp_err_t sensor_config_load_water_calibration(fdc1004_calibration_t *calibration);

/**
 * @brief Initialize sensor configuration subsystem
 * 
 * Creates config directory if needed and loads saved calibrations.
 * Call this during system initialization after filesystem is mounted.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_config_init(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_CONFIG_H
