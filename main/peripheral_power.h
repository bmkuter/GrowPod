/**
 * @file peripheral_power.h
 * @brief Control power to peripheral devices (sensors, display, etc.)
 */

#ifndef PERIPHERAL_POWER_H
#define PERIPHERAL_POWER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and enable power to all peripherals
 * 
 * This must be called before initializing I2C sensors or display.
 * Controls GPIO 21 which powers the sensor rails and display.
 * 
 * @return ESP_OK on success
 */
esp_err_t peripheral_power_init(void);

#ifdef __cplusplus
}
#endif

#endif // PERIPHERAL_POWER_H
