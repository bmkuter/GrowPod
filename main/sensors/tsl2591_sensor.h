/**
 * @file tsl2591_sensor.h
 * @brief Low-level driver for AMS TSL2591 High Dynamic Range Light Sensor
 * 
 * This driver provides functions to read visible and infrared light intensity
 * from the TSL2591 sensor via I2C. Based on AMS TSL2591 datasheet.
 * 
 * Sensor specifications:
 * - Dynamic range: 188 µLux to 88,000 Lux
 * - Temperature range: -30°C to +70°C
 * - I2C address: 0x29 (default)
 * - 16-bit resolution per channel
 * - Visible + IR channels
 */

#ifndef TSL2591_SENSOR_H
#define TSL2591_SENSOR_H

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// TSL2591 I2C address
#define TSL2591_I2C_ADDR                0x29

// TSL2591 Commands and Registers
#define TSL2591_COMMAND_BIT             0xA0  // Command bit for register access
#define TSL2591_REGISTER_ENABLE         0x00
#define TSL2591_REGISTER_CONTROL        0x01
#define TSL2591_REGISTER_DEVICE_ID      0x12
#define TSL2591_REGISTER_STATUS         0x13
#define TSL2591_REGISTER_C0DATAL        0x14  // CH0 low data byte (visible + IR)
#define TSL2591_REGISTER_C0DATAH        0x15  // CH0 high data byte
#define TSL2591_REGISTER_C1DATAL        0x16  // CH1 low data byte (IR only)
#define TSL2591_REGISTER_C1DATAH        0x17  // CH1 high data byte

// Enable Register (0x00)
#define TSL2591_ENABLE_POWEROFF         0x00
#define TSL2591_ENABLE_POWERON          0x01
#define TSL2591_ENABLE_AEN              0x02  // ALS Enable
#define TSL2591_ENABLE_AIEN             0x10  // ALS Interrupt Enable
#define TSL2591_ENABLE_NPIEN            0xA0  // No Persist Interrupt Enable

// Control Register (0x01) - Gain and Integration Time
#define TSL2591_GAIN_LOW                0x00  // Low gain (1x)
#define TSL2591_GAIN_MED                0x10  // Medium gain (25x)
#define TSL2591_GAIN_HIGH               0x20  // High gain (428x)
#define TSL2591_GAIN_MAX                0x30  // Max gain (9876x)

#define TSL2591_INTEGRATIONTIME_100MS   0x00
#define TSL2591_INTEGRATIONTIME_200MS   0x01
#define TSL2591_INTEGRATIONTIME_300MS   0x02
#define TSL2591_INTEGRATIONTIME_400MS   0x03
#define TSL2591_INTEGRATIONTIME_500MS   0x04
#define TSL2591_INTEGRATIONTIME_600MS   0x05

// Device ID
#define TSL2591_DEVICE_ID               0x50

// Lux calculation coefficients (from Adafruit library)
#define TSL2591_LUX_DF                  408.0f  // Device factor
#define TSL2591_LUX_COEFB               1.64f   // CH0 coefficient
#define TSL2591_LUX_COEFC               0.59f   // CH1 coefficient A
#define TSL2591_LUX_COEFD               0.86f   // CH1 coefficient B

/**
 * @brief Gain settings
 */
typedef enum {
    TSL2591_GAIN_1X = 0,    // Low gain (1x)
    TSL2591_GAIN_25X,       // Medium gain (25x)
    TSL2591_GAIN_428X,      // High gain (428x)
    TSL2591_GAIN_9876X      // Max gain (9876x)
} tsl2591_gain_t;

/**
 * @brief Integration time settings
 */
typedef enum {
    TSL2591_INTEG_100MS = 0,
    TSL2591_INTEG_200MS,
    TSL2591_INTEG_300MS,
    TSL2591_INTEG_400MS,
    TSL2591_INTEG_500MS,
    TSL2591_INTEG_600MS
} tsl2591_integration_t;

/**
 * @brief TSL2591 measurement data structure
 */
typedef struct {
    uint16_t ch0;           // Channel 0 (visible + IR)
    uint16_t ch1;           // Channel 1 (IR only)
    uint16_t visible;       // Calculated visible light
    float lux;              // Calculated lux value
    bool valid;             // True if measurement is valid
} tsl2591_data_t;

/**
 * @brief Initialize TSL2591 sensor
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor (default: TSL2591_I2C_ADDR)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tsl2591_init(i2c_port_t i2c_num, uint8_t address);

/**
 * @brief Enable or disable the sensor
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tsl2591_enable(i2c_port_t i2c_num, uint8_t address, bool enable);

/**
 * @brief Set gain and integration time
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param gain Gain setting
 * @param integration Integration time setting
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tsl2591_set_timing(i2c_port_t i2c_num, uint8_t address, 
                              tsl2591_gain_t gain, tsl2591_integration_t integration);

/**
 * @brief Read light sensor data
 * 
 * This is the main measurement function. It reads both channels and calculates
 * the lux value.
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param data Pointer to data structure to store results
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tsl2591_read_data(i2c_port_t i2c_num, uint8_t address, tsl2591_data_t *data);

/**
 * @brief Calculate lux from raw channel data
 * 
 * @param ch0 Channel 0 value (visible + IR)
 * @param ch1 Channel 1 value (IR only)
 * @param gain Current gain setting
 * @param integration Current integration time setting
 * @return Calculated lux value, or -1.0 on error
 */
float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1, 
                            tsl2591_gain_t gain, tsl2591_integration_t integration);

/**
 * @brief Read device ID
 * 
 * @param i2c_num I2C port number
 * @param address I2C address of the sensor
 * @param device_id Pointer to store device ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tsl2591_read_device_id(i2c_port_t i2c_num, uint8_t address, uint8_t *device_id);

#ifdef __cplusplus
}
#endif

#endif // TSL2591_SENSOR_H
