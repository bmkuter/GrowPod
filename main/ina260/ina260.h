#ifndef INA260_H
#define INA260_H

#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO          18    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO          19    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS      1000

#define INA260_ADDRESS             0x40 /*!< Default I2C address */
#define INA260_REG_CONFIG          0x00 /*!< Configuration Register */
#define INA260_REG_CURRENT         0x01 /*!< Current Register */
#define INA260_REG_BUS_VOLTAGE     0x02 /*!< Bus Voltage Register */
#define INA260_REG_POWER           0x03 /*!< Power Register */

void i2c_master_init(void);
void ina260_init(void);
void ina260_task(void *pvParameter);

esp_err_t ina260_read_register(uint8_t reg_addr, uint16_t *data);
esp_err_t ina260_read_current(int16_t *current);
esp_err_t ina260_read_voltage(uint16_t *voltage);
esp_err_t ina260_read_power(uint16_t *power);

#endif // INA260_H
