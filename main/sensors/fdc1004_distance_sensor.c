#include "fdc1004_distance_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "power_monitor_HAL.h"  // For shared I2C bus configuration

static const char *TAG = "FDC1004";

// FDC1004 uses the shared I2C bus
static i2c_port_t i2c_port = I2C_NUM_0;

// Calibration parameters (to be determined during calibration)
static float s_cap_to_mm_scale = FDC1004_CAP_TO_MM_SCALE;
static float s_cap_to_mm_offset = FDC1004_CAP_TO_MM_OFFSET;

// Helper function to write to FDC1004 register (16-bit)
static esp_err_t fdc1004_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),   // MSB
        (uint8_t)(value & 0xFF)  // LSB
    };
    
    esp_err_t ret = i2c_master_write_to_device(i2c_port, FDC1004_I2C_ADDR, 
                                                data, sizeof(data), 
                                                1000 / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

// Helper function to read from FDC1004 register (16-bit)
static esp_err_t fdc1004_read_register(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    
    esp_err_t ret = i2c_master_write_read_device(i2c_port, FDC1004_I2C_ADDR,
                                                   &reg, 1,
                                                   data, sizeof(data),
                                                   1000 / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    } else {
        ESP_LOGE(TAG, "Failed to read register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t fdc1004_init(void)
{
    ESP_LOGI(TAG, "Initializing FDC1004 capacitive distance sensor...");
    
    // The I2C bus is already initialized by power_monitor_HAL
    // We just need to verify the device is present and configure it
    
    // Read device and manufacturer IDs to verify presence
    uint16_t manufacturer_id, device_id;
    esp_err_t ret = fdc1004_read_device_id(&manufacturer_id, &device_id);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FDC1004 not found on I2C bus");
        return ret;
    }
    
    ESP_LOGI(TAG, "FDC1004 found - Manufacturer ID: 0x%04x, Device ID: 0x%04x", 
             manufacturer_id, device_id);
    
    // Configure measurement on channel 1
    // Default configuration:
    // - Channel 1 (CIN1) to CAPDAC
    // - CAPDAC = 0 (no offset)
    // - Sample rate = 100 S/s
    ret = fdc1004_configure_measurement(1, 0, FDC1004_SAMPLE_RATE_100);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FDC1004 initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to configure FDC1004");
    }
    
    return ret;
}

esp_err_t fdc1004_configure_measurement(uint8_t channel, uint8_t capdac, uint8_t sample_rate)
{
    if (channel < 1 || channel > 4) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (capdac > FDC1004_CAPDAC_MAX) {
        ESP_LOGE(TAG, "Invalid CAPDAC value: %d", capdac);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate configuration register address
    uint8_t conf_reg = FDC1004_REG_CONF_MEAS1 + (channel - 1);
    
    // Configuration format:
    // [15:13] CHA (positive input channel, e.g., CIN1 = 0b000)
    // [12:10] CHB (negative input channel, CAPDAC = 0b100)
    // [9:5]   CAPDAC value
    // [4:0]   Reserved
    
    uint16_t conf_value = (0b000 << 13) |  // CHA = CIN1 (adjust for other channels)
                          (0b100 << 10) |  // CHB = CAPDAC
                          ((uint16_t)capdac << 5);
    
    esp_err_t ret = fdc1004_write_register(conf_reg, conf_value);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure FDC_CONF register for sample rate and repeat mode
    // [15:10] Reserved
    // [9:8]   Repeat (0b11 = repeat measurements)
    // [7:6]   Sample rate
    // [5:4]   Reserved
    // [3:0]   Measurement enable (bit 0 = MEAS1, bit 1 = MEAS2, etc.)
    
    uint16_t fdc_conf = (0b00 << 10) |  // Reserved
                        (sample_rate << 6) |
                        (1 << (channel - 1));  // Enable measurement on specified channel
    
    return fdc1004_write_register(FDC1004_REG_FDC_CONF, fdc_conf);
}

esp_err_t fdc1004_trigger_measurement(uint8_t channel)
{
    if (channel < 1 || channel > 4) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read current FDC_CONF register
    uint16_t fdc_conf;
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_FDC_CONF, &fdc_conf);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set the measurement bit for the specified channel
    fdc_conf |= (1 << (channel - 1));
    
    return fdc1004_write_register(FDC1004_REG_FDC_CONF, fdc_conf);
}

esp_err_t fdc1004_is_measurement_ready(uint8_t channel, bool *is_ready)
{
    if (channel < 1 || channel > 4 || is_ready == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read FDC_CONF register
    uint16_t fdc_conf;
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_FDC_CONF, &fdc_conf);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Check if measurement is complete (bit is cleared when done)
    *is_ready = ((fdc_conf & (1 << (channel - 1 + 3))) != 0);
    
    return ESP_OK;
}

esp_err_t fdc1004_read_capacitance(float *capacitance)
{
    if (capacitance == NULL) {
        ESP_LOGE(TAG, "Null capacitance pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Trigger measurement on channel 1
    esp_err_t ret = fdc1004_trigger_measurement(1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for measurement to complete (max ~10ms at 100 S/s)
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Read MSB and LSB of measurement 1
    uint16_t msb, lsb;
    ret = fdc1004_read_register(FDC1004_REG_MEAS1_MSB, &msb);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = fdc1004_read_register(FDC1004_REG_MEAS1_LSB, &lsb);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine MSB and LSB to get 24-bit signed value
    // MSB contains bits [23:8], LSB contains bits [7:0]
    int32_t raw_value = ((int32_t)msb << 8) | (lsb >> 8);
    
    // Sign extend from 24-bit to 32-bit
    if (raw_value & 0x800000) {
        raw_value |= 0xFF000000;
    }
    
    // Convert to capacitance in pF
    // FDC1004 LSB = 3.125 fF (femtofarads)
    // Capacitance (pF) = raw_value * 3.125e-6
    *capacitance = (float)raw_value * 3.125e-6f;
    
    ESP_LOGD(TAG, "Raw: 0x%06lx, Capacitance: %.4f pF", raw_value, *capacitance);
    
    return ESP_OK;
}

int fdc1004_read_distance_mm(void)
{
    float capacitance;
    esp_err_t ret = fdc1004_read_capacitance(&capacitance);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read capacitance");
        return -1;
    }
    
    // Convert capacitance to distance using calibration parameters
    // This is a placeholder calculation - actual calibration needed
    float distance_mm = (capacitance - s_cap_to_mm_offset) * s_cap_to_mm_scale;
    
    ESP_LOGD(TAG, "Capacitance: %.4f pF -> Distance: %.2f mm", capacitance, distance_mm);
    
    return (int)distance_mm;
}

esp_err_t fdc1004_read_device_id(uint16_t *manufacturer_id, uint16_t *device_id)
{
    if (manufacturer_id == NULL || device_id == NULL) {
        ESP_LOGE(TAG, "Null pointer for device ID");
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = fdc1004_read_register(FDC1004_REG_MANUFACTURER_ID, manufacturer_id);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return fdc1004_read_register(FDC1004_REG_DEVICE_ID, device_id);
}
