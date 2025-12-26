/**
 * @file sensor_api.h
 * @brief External API for sensor access throughout the GrowPod system
 * 
 * This header provides the public API that all other components should use
 * to access sensor data. It routes requests through the sensor manager for
 * centralized I2C arbitration and caching.
 * 
 * USAGE:
 * - Other modules should include this header instead of directly accessing sensors
 * - All functions are thread-safe and can be called from any task
 * - Functions use timeouts to prevent blocking indefinitely
 * 
 * MIGRATION NOTES:
 * - Old code using power_monitor_read_x() should switch to sensor_api_read_power_x()
 * - These functions have the same signature but route through sensor manager
 */

#ifndef SENSOR_API_H
#define SENSOR_API_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default timeout for sensor reads (milliseconds)
#define SENSOR_API_DEFAULT_TIMEOUT_MS   5000

/**
 * @brief Read current from power monitor
 * 
 * This is the new API that replaces direct power_monitor_read_current() calls.
 * It routes through the sensor manager for coordinated I2C access.
 * 
 * @param current_ma Pointer to store current in milliamps
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_api_read_power_current(float *current_ma);

/**
 * @brief Read voltage from power monitor
 * 
 * @param voltage_mv Pointer to store voltage in millivolts
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_api_read_power_voltage(float *voltage_mv);

/**
 * @brief Read power from power monitor
 * 
 * @param power_mw Pointer to store power in milliwatts
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_api_read_power_power(float *power_mw);

/**
 * @brief Read temperature from SHT45
 * 
 * @param temperature_c Pointer to store temperature in Celsius
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_api_read_temperature(float *temperature_c);

/**
 * @brief Read humidity from SHT45
 * 
 * @param humidity_rh Pointer to store relative humidity in %
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_api_read_humidity(float *humidity_rh);

/**
 * @brief Read water level
 * 
 * @param level_mm Pointer to store water level in millimeters
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_api_read_water_level(float *level_mm);

/**
 * @brief Read all power metrics at once (more efficient)
 * 
 * @param current_ma Pointer to store current (can be NULL to skip)
 * @param voltage_mv Pointer to store voltage (can be NULL to skip)
 * @param power_mw Pointer to store power (can be NULL to skip)
 * @return ESP_OK if all requested values read successfully
 */
esp_err_t sensor_api_read_power_all(float *current_ma, float *voltage_mv, float *power_mw);

/**
 * @brief Read all environment metrics at once
 * 
 * @param temperature_c Pointer to store temperature (can be NULL to skip)
 * @param humidity_rh Pointer to store humidity (can be NULL to skip)
 * @return ESP_OK if all requested values read successfully
 */
esp_err_t sensor_api_read_environment_all(float *temperature_c, float *humidity_rh);

/**
 * @brief Get cached sensor value without triggering new read
 * 
 * This is useful for displaying data that doesn't need to be real-time.
 * Returns the most recent value from the sensor manager cache.
 * 
 * @param sensor_name Name of sensor ("current", "voltage", "power", "temperature", "humidity", "water_level")
 * @param value Pointer to store value
 * @param age_ms Pointer to store age of data in milliseconds (can be NULL)
 * @return ESP_OK if cached data available, ESP_ERR_NOT_FOUND if no cached data
 */
esp_err_t sensor_api_get_cached(const char *sensor_name, float *value, uint32_t *age_ms);

/**
 * @brief Force immediate sensor reading (bypasses cache)
 * 
 * Use this sparingly as it disrupts the normal polling schedule.
 * Useful for calibration or user-triggered "refresh" operations.
 * 
 * @param sensor_name Name of sensor to read
 * @param value Pointer to store value
 * @return ESP_OK on success
 */
esp_err_t sensor_api_force_read(const char *sensor_name, float *value);

/**
 * @brief Print all current sensor values
 * 
 * Utility function for debugging. Prints to console.
 */
void sensor_api_print_all(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_API_H
