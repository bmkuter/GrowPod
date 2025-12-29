/**
 * @file sensor_manager.h
 * @brief Centralized sensor management system with priority-based I2C access
 * 
 * This module provides a centralized task-based system for managing all sensors
 * in the GrowPod. It handles:
 * - Priority-based I2C bus arbitration
 * - Cached sensor readings with mutex protection
 * - Periodic polling of sensors at different rates
 * - Timeout-based request/response API for other tasks
 * 
 * Architecture:
 * - Main sensor task runs continuously, polling sensors by priority
 * - Other tasks request sensor data via mailbox (queue) with timeout
 * - Sensor data is cached and protected by mutex
 * - I2C bus access is serialized through the sensor task
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sensor types managed by the sensor manager
 */
typedef enum {
    SENSOR_TYPE_POWER_CURRENT = 0,  // INA219/260 current reading
    SENSOR_TYPE_POWER_VOLTAGE,      // INA219/260 voltage reading
    SENSOR_TYPE_POWER_POWER,        // INA219/260 power reading
    SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY,  // SHT45 temperature and humidity (combined)
    SENSOR_TYPE_LIGHT,              // TSL2591 light sensor (lux + visible + IR)
    SENSOR_TYPE_WATER_LEVEL,        // FDC1004 distance/water level
    SENSOR_TYPE_MAX                 // Total number of sensor types
} sensor_type_t;

/**
 * @brief Sensor reading priority levels
 */
typedef enum {
    SENSOR_PRIORITY_LOW = 0,    // Polled every 50 cycles (~5s)
    SENSOR_PRIORITY_MEDIUM,     // Polled every 25 cycles (~2.5s)
    SENSOR_PRIORITY_HIGH,       // Polled every 10 cycles (~1s)
    SENSOR_PRIORITY_CRITICAL    // Polled every 5 cycles (~500ms)
} sensor_priority_t;

/**
 * @brief Power sensor data (INA219/260)
 */
typedef struct {
    float value;                // Current (mA), Voltage (mV), or Power (mW) depending on sensor type
} power_data_t;

/**
 * @brief Temperature and humidity sensor data (SHT45)
 */
typedef struct {
    float temperature_c;        // Temperature in Celsius
    float humidity_rh;          // Relative humidity in %
} environment_data_t;

/**
 * @brief Water level sensor data
 */
typedef struct {
    float level_mm;             // Water level in millimeters
} water_level_data_t;

/**
 * @brief Light sensor data (TSL2591)
 */
typedef struct {
    float lux;                  // Light intensity in lux
    uint16_t visible;           // Visible light (CH0 - CH1)
    uint16_t infrared;          // Infrared light (CH1)
} light_data_t;

/**
 * @brief Sensor data union - holds data for any sensor type
 */
typedef union {
    power_data_t power;                     // Power monitor data
    environment_data_t environment;         // Temperature and humidity data
    light_data_t light;                     // Light sensor data
    water_level_data_t water_level;         // Water level data
} sensor_data_t;

/**
 * @brief Sensor data cache entry
 */
typedef struct {
    sensor_data_t data;         // Union of all possible sensor data types
    sensor_type_t type;         // Type discriminator for the union
    uint32_t timestamp_ms;      // Timestamp of last update (milliseconds)
    bool valid;                 // True if data is valid
    esp_err_t last_error;       // Last error code
} sensor_cache_entry_t;

/**
 * @brief Sensor request structure for mailbox communication
 */
typedef struct {
    sensor_type_t sensor_type;      // Which sensor to read
    float *value_out;               // Pointer to store result
    esp_err_t *error_out;           // Pointer to store error code (optional)
    SemaphoreHandle_t done_sem;     // Semaphore to signal completion
} sensor_request_t;

/**
 * @brief Sensor manager configuration
 */
typedef struct {
    uint32_t task_stack_size;       // Stack size for sensor task (default: 4096)
    UBaseType_t task_priority;      // FreeRTOS task priority (default: 5)
    uint32_t poll_period_ms;        // Base polling period in ms (default: 100)
    uint32_t cache_timeout_ms;      // Max age of cached data (default: 5000)
    size_t request_queue_size;      // Size of request queue (default: 10)
} sensor_manager_config_t;

/**
 * @brief Get default sensor manager configuration
 * 
 * @return Default configuration structure
 */
sensor_manager_config_t sensor_manager_get_default_config(void);

/**
 * @brief Initialize the sensor manager
 * 
 * This must be called before any other sensor manager functions.
 * It initializes the I2C bus, creates mutexes, and starts the sensor task.
 * 
 * @param config Configuration structure (NULL for defaults)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_init(const sensor_manager_config_t *config);

/**
 * @brief Start the sensor manager task
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_start(void);

/**
 * @brief Stop the sensor manager task
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_stop(void);

/**
 * @brief Request a sensor reading (blocking with timeout)
 * 
 * This is the main API for other tasks to request sensor data.
 * It will return cached data if fresh enough, otherwise trigger a new read.
 * 
 * @param sensor_type Type of sensor to read
 * @param value Pointer to store the sensor value
 * @param timeout_ms Maximum time to wait for reading (milliseconds)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if timeout, other error codes
 */
esp_err_t sensor_manager_read(sensor_type_t sensor_type, float *value, uint32_t timeout_ms);

/**
 * @brief Get cached sensor reading (non-blocking)
 * 
 * Returns the most recent cached value without triggering a new read.
 * Check the timestamp to determine if data is fresh.
 * 
 * @param sensor_type Type of sensor to read
 * @param value Pointer to store the sensor value
 * @param timestamp_ms Pointer to store the timestamp (optional, can be NULL)
 * @return ESP_OK if valid data available, ESP_ERR_NOT_FOUND if no data cached
 */
esp_err_t sensor_manager_get_cached(sensor_type_t sensor_type, float *value, uint32_t *timestamp_ms);

/**
 * @brief Get cached environment sensor reading (temperature and humidity)
 * 
 * Returns both temperature and humidity from a single cache entry.
 * This is more efficient than calling sensor_manager_get_cached twice.
 * 
 * @param temperature_c Pointer to store temperature in Celsius (can be NULL)
 * @param humidity_rh Pointer to store humidity in %RH (can be NULL)
 * @param timestamp_ms Pointer to store the timestamp (optional, can be NULL)
 * @return ESP_OK if valid data available, ESP_ERR_NOT_FOUND if no data cached
 */
esp_err_t sensor_manager_get_environment_cached(float *temperature_c, float *humidity_rh, uint32_t *timestamp_ms);

/**
 * @brief Get full cached sensor data structure (for complex sensors)
 * 
 * Returns the full typed sensor data structure from cache.
 * Use this for sensors that return multiple values.
 * 
 * @param sensor_type Type of sensor to read
 * @param data Pointer to store the full sensor data union
 * @param timestamp_ms Pointer to store the timestamp (optional, can be NULL)
 * @return ESP_OK if valid data available, ESP_ERR_NOT_FOUND if no data cached
 */
esp_err_t sensor_manager_get_data_cached(sensor_type_t sensor_type, sensor_data_t *data, uint32_t *timestamp_ms);

/**
 * @brief Force an immediate sensor reading (bypasses cache)
 * 
 * This function forces a fresh reading from the specified sensor,
 * bypassing the cache. Use sparingly as it can disrupt the polling schedule.
 * 
 * @param sensor_type Type of sensor to read
 * @param value Pointer to store the sensor value
 * @param timeout_ms Maximum time to wait for reading (milliseconds)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_manager_read_forced(sensor_type_t sensor_type, float *value, uint32_t timeout_ms);

/**
 * @brief Get sensor manager statistics
 * 
 * @param total_reads Total number of sensor reads performed
 * @param cache_hits Number of times cached data was used
 * @param cache_misses Number of times fresh read was required
 * @param errors Number of sensor read errors
 */
void sensor_manager_get_stats(uint32_t *total_reads, uint32_t *cache_hits,
                              uint32_t *cache_misses, uint32_t *errors);

/**
 * @brief Print sensor manager debug information
 */
void sensor_manager_print_debug_info(void);

/**
 * @brief Set sensor polling priority
 * 
 * @param sensor_type Sensor to configure
 * @param priority New priority level
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_set_priority(sensor_type_t sensor_type, sensor_priority_t priority);

/**
 * @brief Enable/disable a sensor
 * 
 * @param sensor_type Sensor to configure
 * @param enabled True to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t sensor_manager_set_enabled(sensor_type_t sensor_type, bool enabled);

/**
 * @brief Check if sensor manager is running
 * 
 * @return True if running, false otherwise
 */
bool sensor_manager_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_MANAGER_H
