/**
 * @file i2c_scanner.h
 * @brief Simple I2C bus scanner utility for debugging
 */

#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Scan I2C bus and print all detected devices
 * 
 * @param i2c_num I2C port number
 */
void i2c_scanner_scan(i2c_port_t i2c_num);

#ifdef __cplusplus
}
#endif

#endif // I2C_SCANNER_H
