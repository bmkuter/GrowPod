#ifndef POWER_MONITOR_HAL_H
#define POWER_MONITOR_HAL_H

#include "esp_err.h"
#include <stdbool.h>
#include "ina260.h"
#include "ina219.h"

// Power monitor chip types
typedef enum {
    POWER_MONITOR_CHIP_INA260,
    POWER_MONITOR_CHIP_INA219
} power_monitor_chip_t;

// Calibration types for INA219
typedef enum {
    POWER_MONITOR_CAL_32V_2A,
    POWER_MONITOR_CAL_32V_1A,
    POWER_MONITOR_CAL_16V_400MA,
    POWER_MONITOR_CAL_12V_3A
} power_monitor_calibration_t;

// Function prototypes
esp_err_t power_monitor_init(power_monitor_chip_t chip_type, uint8_t i2c_address);

// I2C
esp_err_t i2c_master_init(void);

// Reading measurements
esp_err_t power_monitor_read_current(float *current_ma);
esp_err_t power_monitor_read_voltage(float *voltage_mv);
esp_err_t power_monitor_read_power(float *power_mw);

// Set calibration (only applies to INA219)
esp_err_t power_monitor_set_calibration(power_monitor_calibration_t calibration);

// Power saving mode
esp_err_t power_monitor_power_save(bool on);

// Get the current chip type in use
power_monitor_chip_t power_monitor_get_chip_type(void);

// Task for periodic readings
void power_monitor_task(void *pvParameter);

#endif // POWER_MONITOR_HAL_H
