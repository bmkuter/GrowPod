#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the distance sensor subsystem (either ultrasonic or laser).
 */
esp_err_t distance_sensor_init(void);

/**
 * @brief Task for distance sensor measurement loop.
 *        Create this in app_main() or somewhere after initialization.
 */
void distance_sensor_task(void *pvParameters);

int distance_sensor_read_mm();

#ifdef __cplusplus
}
#endif

#endif // DISTANCE_SENSOR_H
