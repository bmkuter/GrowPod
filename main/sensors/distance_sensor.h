#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the distance sensor subsystem (either ultrasonic or laser).
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t distance_sensor_init(void);

/**
 * @brief Task for distance sensor background measurement loop.
 *        Create this in app_main() or somewhere after initialization.
 *        This task continuously updates the cached distance measurement.
 * 
 * @param pvParameters FreeRTOS task parameters (not used)
 */
void distance_sensor_task(void *pvParameters);

/**
 * @brief Read the current distance in millimeters.
 * 
 * This function is thread-safe and will:
 * 1. Return a cached value if a recent reading (< 500ms old) is available
 * 2. Try to acquire the sensor mutex with a 1s timeout
 * 3. If mutex acquisition succeeds, take a new measurement 
 * 4. If mutex acquisition fails but a previous reading exists, return that
 * 
 * @note This function can block for up to 1 second if the sensor is busy
 * @note Each sensor reading takes approximately 400ms
 * 
 * @return Distance in mm, or negative value if error
 */
int distance_sensor_read_mm(void);

/**
 * @brief Get the last valid reading without attempting to access the sensor.
 * 
 * This function is non-blocking and returns immediately.
 * Use this when you need a quick distance value without caring about freshness.
 * 
 * @return Last valid distance in mm, or -1 if no valid reading available
 */
int distance_sensor_get_last_reading_mm(void);

#ifdef __cplusplus
}
#endif

#endif // DISTANCE_SENSOR_H
