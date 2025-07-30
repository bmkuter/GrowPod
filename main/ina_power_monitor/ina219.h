#ifndef INA219_H
#define INA219_H

#include "esp_err.h"
#include "power_monitor_HAL.h"

#define INA219_ADDRESS    0x40  // Default I2C address for INA219

// I2C configuration - using the same as INA260 for compatibility
#define I2C_MASTER_SCL_IO          41    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO          42    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS      1000

// INA219 register definitions
#define INA219_REG_CONFIG          0x00 /*!< Configuration Register */
#define INA219_REG_SHUNTVOLTAGE    0x01 /*!< Shunt Voltage Register */
#define INA219_REG_BUSVOLTAGE      0x02 /*!< Bus Voltage Register */
#define INA219_REG_POWER           0x03 /*!< Power Register */
#define INA219_REG_CURRENT         0x04 /*!< Current Register */
#define INA219_REG_CALIBRATION     0x05 /*!< Calibration Register */

// Config register bits
#define INA219_CONFIG_RESET        0x8000 /*!< Reset bit */
#define INA219_CONFIG_BVOLTAGERANGE_32V      0x2000 /*!< Bus Voltage Range Mask (0-32V) */
#define INA219_CONFIG_BVOLTAGERANGE_16V      0x0000 /*!< Bus Voltage Range Mask (0-16V) */
#define INA219_CONFIG_GAIN_8_320MV           0x1800 /*!< Gain 8, 320mV */
#define INA219_CONFIG_GAIN_1_40MV            0x0000 /*!< Gain 1, 40mV */
#define INA219_CONFIG_BADCRES_12BIT          0x0180 /*!< 12-bit bus res = 0..4097 */
#define INA219_CONFIG_SADCRES_12BIT_1S_532US 0x0018 /*!< 1 x 12-bit shunt sample */
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007 /*!< Shunt and Bus, Continuous */

// Function prototypes
esp_err_t ina219_init(uint8_t address);

// Reading INA219 registers
esp_err_t ina219_read_register(uint8_t i2c_address, uint8_t reg_addr, uint16_t *data);
esp_err_t ina219_write_register(uint8_t i2c_address, uint8_t reg_addr, uint16_t data);

// Calibration settings
esp_err_t ina219_set_calibration_32V_2A(uint8_t address);
esp_err_t ina219_set_calibration_32V_1A(uint8_t address);
esp_err_t ina219_set_calibration_16V_400mA(uint8_t address);
esp_err_t ina219_set_calibration_12V_3A(uint8_t address);  // New 12V/3A calibration for GrowPod

// Reading current, voltage, and power from INA219
esp_err_t ina219_read_bus_voltage(uint8_t address, float *voltage);  // returns in mV
esp_err_t ina219_read_shunt_voltage(uint8_t address, float *voltage); // returns in mV
esp_err_t ina219_read_current(uint8_t address, float *current);      // returns in mA
esp_err_t ina219_read_power(uint8_t address, float *power);          // returns in mW

// Power saving mode
esp_err_t ina219_power_save(uint8_t address, bool on);

#endif // INA219_H
