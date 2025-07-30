#ifndef INA260_H
#define INA260_H

#include "esp_err.h"
#include "power_monitor_HAL.h"

#define INA260_ADDRESS    0x40

// I2C configuration
#define I2C_MASTER_SCL_IO          41    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO          42    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS      1000

// INA260 register definitions
#define INA260_REG_CONFIG          0x00 /*!< Configuration Register */
#define INA260_REG_CURRENT         0x01 /*!< Current Register */
#define INA260_REG_BUS_VOLTAGE     0x02 /*!< Bus Voltage Register */
#define INA260_REG_POWER           0x03 /*!< Power Register */

// INA260 scaling factors
#define INA260_SCALING_VOLTAGE     1.25  /*!< Voltage scaling factor (for mV conversion) */
#define INA260_SCALING_CURRENT     1.25  /*!< Current scaling factor (for mA conversion) */
#define INA260_SCALING_POWER       10.0  /*!< Power scaling factor (for mW conversion) */

// Function prototypes
void ina260_init(uint8_t address);

// Reading INA260 registers
esp_err_t ina260_read_register(uint8_t i2c_address, uint8_t reg_addr, uint16_t *data);

// Reading current, voltage, and power from INA260
esp_err_t ina260_read_current(uint8_t address, float *current);
esp_err_t ina260_read_voltage(uint8_t address, float *voltage);
esp_err_t ina260_read_power(uint8_t address, float *power);

#endif // INA260_H
