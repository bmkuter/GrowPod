#ifndef FDC1004_DISTANCE_SENSOR_H
#define FDC1004_DISTANCE_SENSOR_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// FDC1004 I2C Address
#define FDC1004_I2C_ADDR          0x50  // Default address (can be 0x50 or 0x51)

// FDC1004 Register Addresses
#define FDC1004_REG_MEAS1_MSB     0x00
#define FDC1004_REG_MEAS1_LSB     0x01
#define FDC1004_REG_MEAS2_MSB     0x02
#define FDC1004_REG_MEAS2_LSB     0x03
#define FDC1004_REG_MEAS3_MSB     0x04
#define FDC1004_REG_MEAS3_LSB     0x05
#define FDC1004_REG_MEAS4_MSB     0x06
#define FDC1004_REG_MEAS4_LSB     0x07
#define FDC1004_REG_CONF_MEAS1    0x08
#define FDC1004_REG_CONF_MEAS2    0x09
#define FDC1004_REG_CONF_MEAS3    0x0A
#define FDC1004_REG_CONF_MEAS4    0x0B
#define FDC1004_REG_FDC_CONF      0x0C
#define FDC1004_REG_OFFSET_CAL_CIN1 0x0D
#define FDC1004_REG_OFFSET_CAL_CIN2 0x0E
#define FDC1004_REG_OFFSET_CAL_CIN3 0x0F
#define FDC1004_REG_OFFSET_CAL_CIN4 0x10
#define FDC1004_REG_GAIN_CAL_CIN1 0x11
#define FDC1004_REG_GAIN_CAL_CIN2 0x12
#define FDC1004_REG_GAIN_CAL_CIN3 0x13
#define FDC1004_REG_GAIN_CAL_CIN4 0x14
#define FDC1004_REG_MANUFACTURER_ID 0xFE
#define FDC1004_REG_DEVICE_ID     0xFF

// FDC1004 Measurement Configuration
#define FDC1004_CAPDAC_MAX        31    // Maximum CAPDAC value (5-bit)
#define FDC1004_SAMPLE_RATE_100   0x01  // 100 S/s
#define FDC1004_SAMPLE_RATE_200   0x02  // 200 S/s
#define FDC1004_SAMPLE_RATE_400   0x03  // 400 S/s

// Capacitance to distance mapping parameters (to be calibrated)
#define FDC1004_CAP_TO_MM_SCALE   1.0f  // Scale factor (placeholder)
#define FDC1004_CAP_TO_MM_OFFSET  0.0f  // Offset (placeholder)

// Calibration data structure
typedef struct {
    uint8_t capdac;              // Optimal CAPDAC value (0-31)
    float cap_empty_pf;          // Capacitance when empty (pF)
    float cap_full_pf;           // Capacitance when full (pF)
    float height_mm;             // Total calibrated height (mm)
    float cap_per_mm;            // Capacitance change per mm
    bool is_calibrated;          // True if calibration is valid
} fdc1004_calibration_t;

/**
 * @brief Calibrate minimum water level (empty point)
 * 
 * This function measures the capacitance at the minimum water level and
 * calculates the optimal CAPDAC to center the measurement range.
 * Call this first with empty container, then fill and call calibrate_water_max.
 * 
 * @param min_height_mm Height in mm at minimum level (typically 0)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_calibrate_water_min(float min_height_mm);

/**
 * @brief Calibrate maximum water level (full point)
 * 
 * This function measures the capacitance at the maximum water level and
 * completes the two-point calibration. Must call calibrate_water_min first.
 * 
 * @param max_height_mm Height in mm at maximum level (e.g., 75mm)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_calibrate_water_max(float max_height_mm);

/**
 * @brief Calibrate the FDC1004 water level sensor (legacy single-step)
 * 
 * This function performs a two-point calibration of the water level sensor:
 * 1. Measures capacitance with empty container
 * 2. User fills to max line
 * 3. Measures capacitance at full level
 * 4. Calculates optimal CAPDAC and scaling factors
 * 
 * NOTE: Consider using calibrate_water_min/max for better control.
 * 
 * The calibration optimizes CAPDAC to center the measurement range for
 * best resolution and uses the full ±15pF range efficiently.
 * 
 * @param height_mm Height from empty to full mark (e.g., 75mm)
 * @param calibration Pointer to store calibration data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_calibrate_water_level(float height_mm, fdc1004_calibration_t *calibration);

/**
 * @brief Get current calibration data
 * 
 * @param calibration Pointer to store calibration data
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not calibrated
 */
esp_err_t fdc1004_get_calibration(fdc1004_calibration_t *calibration);

/**
 * @brief Set calibration data (e.g., loaded from NVS)
 * 
 * @param calibration Pointer to calibration data to apply
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_set_calibration(const fdc1004_calibration_t *calibration);

/**
 * @brief Initialize the FDC1004 capacitive distance sensor
 * 
 * This function initializes the I2C communication with the FDC1004 and
 * configures it for basic capacitance measurements.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_init(void);

/**
 * @brief Read the raw capacitance value from the FDC1004
 * 
 * This function triggers a measurement and reads the raw capacitance value
 * from the FDC1004. The value is in internal FDC1004 units (24-bit signed).
 * 
 * @param capacitance Pointer to store the raw capacitance value (in pF)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_read_capacitance(float *capacitance);

/**
 * @brief Read the distance in millimeters from the FDC1004
 * 
 * This function reads the raw capacitance and converts it to a distance
 * measurement in millimeters using calibration parameters.
 * 
 * @return Distance in mm, or negative value on error
 */
int fdc1004_read_distance_mm(void);

/**
 * @brief Configure the FDC1004 measurement parameters
 * 
 * @param channel Channel number (1-4)
 * @param capdac CAPDAC value for offset compensation (0-31)
 * @param sample_rate Sample rate setting
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_configure_measurement(uint8_t channel, uint8_t capdac, uint8_t sample_rate);

/**
 * @brief Trigger a single measurement on the FDC1004
 * 
 * @param channel Channel number (1-4)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_trigger_measurement(uint8_t channel);

/**
 * @brief Check if a measurement is complete
 * 
 * @param channel Channel number (1-4)
 * @param is_ready Pointer to store the ready status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_is_measurement_ready(uint8_t channel, bool *is_ready);

/**
 * @brief Read the device and manufacturer IDs
 * 
 * @param manufacturer_id Pointer to store manufacturer ID
 * @param device_id Pointer to store device ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fdc1004_read_device_id(uint16_t *manufacturer_id, uint16_t *device_id);

#ifdef __cplusplus
}
#endif

#endif // FDC1004_DISTANCE_SENSOR_H
