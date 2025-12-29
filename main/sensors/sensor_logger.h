/**
 * @file sensor_logger.h
 * @brief Sensor data logging to persistent storage
 * 
 * This module logs sensor readings to LittleFS at 1-minute intervals,
 * maintaining a rolling 24-hour circular buffer (1,440 entries max).
 * 
 * Features:
 * - Circular buffer: oldest entries automatically overwritten
 * - JSON format for human-readability and easy parsing
 * - Single file: /lfs/data/sensor_history.json
 * - Atomic updates: minimizes corruption risk
 */

#ifndef SENSOR_LOGGER_H
#define SENSOR_LOGGER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sensor history entry structure
 */
typedef struct {
    int64_t timestamp;          ///< Unix timestamp (seconds since epoch)
    float temperature_c;        ///< Temperature in Celsius (-999 if invalid)
    float humidity_rh;          ///< Relative humidity % (-999 if invalid)
    float light_lux;            ///< Illuminance in lux (-999 if invalid)
    uint32_t light_visible;     ///< Visible light counts (0 if invalid)
    uint32_t light_infrared;    ///< Infrared counts (0 if invalid)
    float power_mw;             ///< Power consumption in milliwatts
    float current_ma;           ///< Current in milliamps
    float voltage_mv;           ///< Voltage in millivolts
    float water_level_mm;       ///< Water level in millimeters (-1 if invalid)
} sensor_history_entry_t;

/**
 * @brief Sensor logger statistics
 */
typedef struct {
    uint32_t total_entries;     ///< Total entries written (lifetime)
    uint32_t write_errors;      ///< Number of write failures
    uint32_t current_count;     ///< Current number of entries in buffer
    uint32_t buffer_wraps;      ///< Number of times buffer wrapped around
    int64_t oldest_timestamp;   ///< Timestamp of oldest entry
    int64_t newest_timestamp;   ///< Timestamp of newest entry
} sensor_logger_stats_t;

/**
 * @brief Initialize sensor logger
 * 
 * Creates the data directory and initializes the history file if needed.
 * If the file exists, it will be loaded and validated.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_logger_init(void);

/**
 * @brief Log current sensor snapshot
 * 
 * Captures current sensor readings from sensor manager cache and
 * appends them to the circular buffer. Automatically overwrites
 * oldest entries when buffer is full.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_logger_log_snapshot(void);

/**
 * @brief Get historical data within time range
 * 
 * Retrieves sensor history entries within the specified time range.
 * 
 * @param start_timestamp Unix timestamp for range start (0 = all)
 * @param end_timestamp Unix timestamp for range end (0 = now)
 * @param entries Output buffer for entries
 * @param max_entries Maximum number of entries to return
 * @param count_out Actual number of entries returned
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_logger_get_history(int64_t start_timestamp,
                                     int64_t end_timestamp,
                                     sensor_history_entry_t *entries,
                                     uint32_t max_entries,
                                     uint32_t *count_out);

/**
 * @brief Get logger statistics
 * 
 * @param stats Output buffer for statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_logger_get_stats(sensor_logger_stats_t *stats);

/**
 * @brief Clear all historical data
 * 
 * Removes all entries from the history buffer and resets statistics.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_logger_clear_history(void);

/**
 * @brief Print debug information
 */
void sensor_logger_print_debug_info(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_LOGGER_H
