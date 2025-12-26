/**
 * @file sht45_sensor.h
 * @brief Low-level driver for Sensirion SHT45 Temperature & Humidity Sensor
 * 
 * This driver provides functions to read temperature and humidity from the SHT45
 * sensor via I2C. Based on Sensirion SHT4x datasheet.
 * 
 * Sensor specifications:
 * - Temperature range: -40°C to +125°C
 * - Humidity range: 0% to 100% RH
 * - I2C address: 0x44 (default)
 * - Measurement time: ~9ms (high precision)
 */

#ifndef SHT45_SENSOR_H
#define SHT45_SENSOR_H

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// SHT45 I2C address
#define SHT45_I2C_ADDR                  0x44

// SHT45 Commands
#define SHT45_CMD_MEASURE_HIGH_PREC     0xFD  // High precision measurement (~9ms)
#define SHT45_CMD_MEASURE_MED_PREC      0xF6  // Medium precision measurement (~5ms)
#define SHT45_CMD_MEASURE_LOW_PREC      0xE0  // Low precision measurement (~2ms)
#define SHT45_CMD_READ_SERIAL           0x89  // Read serial number
#define SHT45_CMD_SOFT_RESET            0x94  // Soft reset

// Measurement delays (milliseconds)
// Increased from datasheet typical values for reliability at 100kHz I2C
#define SHT45_DELAY_HIGH_PREC_MS        15  // Datasheet: 8.3ms typ
#define SHT45_DELAY_MED_PREC_MS         8   // Datasheet: 4.5ms typ  
#define SHT45_DELAY_LOW_PREC_MS         4   // Datasheet: 1.7ms typ

/**
 * @brief Measurement precision modes
 */
typedef enum {
    SHT45_PRECISION_HIGH = 0,   // High precision, ~9ms
    SHT45_PRECISION_MEDIUM,     // Medium precision, ~5ms
    SHT45_PRECISION_LOW         // Low precision, ~2ms
} sht45_precision_t;

/**
 * @brief SHT45 measurement data structure
 */
typedef struct {
    float temperature_c;    // Temperature in Celsius
    float humidity_rh;      // Relative humidity in %
    bool valid;            // True if measurement is valid
} sht45_data_t;

/**
 * @brief Initialize SHT45 sensor
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor (default: SHT45_I2C_ADDR)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht45_init(i2c_port_t i2c_num, uint8_t address);

/**
 * @brief Perform soft reset of the sensor
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht45_soft_reset(i2c_port_t i2c_num, uint8_t address);

/**
 * @brief Read temperature and humidity with specified precision
 * 
 * This is the main measurement function. It triggers a measurement and waits
 * for it to complete before reading the data.
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param precision Measurement precision mode
 * @param data Pointer to store measurement results
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht45_read_data(i2c_port_t i2c_num, uint8_t address, 
                          sht45_precision_t precision, sht45_data_t *data);

/**
 * @brief Read temperature only (convenience function)
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param precision Measurement precision mode
 * @param temperature_c Pointer to store temperature in Celsius
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht45_read_temperature(i2c_port_t i2c_num, uint8_t address,
                                 sht45_precision_t precision, float *temperature_c);

/**
 * @brief Read humidity only (convenience function)
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param precision Measurement precision mode
 * @param humidity_rh Pointer to store humidity in %RH
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht45_read_humidity(i2c_port_t i2c_num, uint8_t address,
                              sht45_precision_t precision, float *humidity_rh);

/**
 * @brief Read sensor serial number
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param serial Pointer to store 32-bit serial number
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht45_read_serial(i2c_port_t i2c_num, uint8_t address, uint32_t *serial);

/**
 * @brief Calculate CRC-8 checksum for SHT45 data validation
 * 
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * Initialization: 0xFF
 * 
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return Calculated CRC-8 checksum
 */
uint8_t sht45_calculate_crc(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // SHT45_SENSOR_H
