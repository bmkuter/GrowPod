/**
 * @file tsl2591_sensor.c
 * @brief Implementation of TSL2591 High Dynamic Range Light Sensor driver
 */

#include "tsl2591_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "TSL2591";

// Current sensor configuration
static tsl2591_gain_t current_gain = TSL2591_GAIN_25X;
static tsl2591_integration_t current_integration = TSL2591_INTEG_100MS;

/**
 * @brief Write a single byte to a TSL2591 register
 */
static esp_err_t tsl2591_write_register(i2c_port_t i2c_num, uint8_t address, 
                                         uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {TSL2591_COMMAND_BIT | reg, value};
    
    esp_err_t ret = i2c_master_write_to_device(i2c_num, address, write_buf, 2, 
                                                pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Read a single byte from a TSL2591 register
 */
static esp_err_t tsl2591_read_register(i2c_port_t i2c_num, uint8_t address, 
                                        uint8_t reg, uint8_t *value)
{
    uint8_t reg_addr = TSL2591_COMMAND_BIT | reg;
    
    esp_err_t ret = i2c_master_write_read_device(i2c_num, address, &reg_addr, 1, 
                                                  value, 1, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Read multiple bytes from TSL2591 registers
 */
static esp_err_t tsl2591_read_registers(i2c_port_t i2c_num, uint8_t address, 
                                         uint8_t reg, uint8_t *buffer, size_t len)
{
    uint8_t reg_addr = TSL2591_COMMAND_BIT | reg;
    
    esp_err_t ret = i2c_master_write_read_device(i2c_num, address, &reg_addr, 1, 
                                                  buffer, len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %d bytes from register 0x%02X: %s", 
                 len, reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t tsl2591_init(i2c_port_t i2c_num, uint8_t address)
{
    ESP_LOGI(TAG, "Initializing TSL2591 at address 0x%02X", address);
    
    // Try to probe the device first with a simple I2C probe
    ESP_LOGI(TAG, "Probing I2C device at address 0x%02X", address);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t probe_ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (probe_ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C probe failed at address 0x%02X: %s", address, esp_err_to_name(probe_ret));
        ESP_LOGE(TAG, "TSL2591 sensor not detected on I2C bus");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "I2C probe successful at address 0x%02X", address);
    
    // Read and verify device ID
    uint8_t device_id;
    esp_err_t ret = tsl2591_read_device_id(i2c_num, address, &device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }
    
    ESP_LOGI(TAG, "Read device ID: 0x%02X (expected 0x%02X)", device_id, TSL2591_DEVICE_ID);
    
    if (device_id != TSL2591_DEVICE_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X)", 
                 device_id, TSL2591_DEVICE_ID);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Device ID verified: 0x%02X", device_id);
    
    // Enable the sensor
    ret = tsl2591_enable(i2c_num, address, true);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set default timing (medium gain, 100ms integration)
    ret = tsl2591_set_timing(i2c_num, address, TSL2591_GAIN_25X, TSL2591_INTEG_100MS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "TSL2591 initialized successfully (Gain: 25x, Integration: 100ms)");
    return ESP_OK;
}

esp_err_t tsl2591_enable(i2c_port_t i2c_num, uint8_t address, bool enable)
{
    uint8_t value = enable ? (TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN) 
                           : TSL2591_ENABLE_POWEROFF;
    
    ESP_LOGD(TAG, "%s sensor", enable ? "Enabling" : "Disabling");
    return tsl2591_write_register(i2c_num, address, TSL2591_REGISTER_ENABLE, value);
}

esp_err_t tsl2591_set_timing(i2c_port_t i2c_num, uint8_t address, 
                              tsl2591_gain_t gain, tsl2591_integration_t integration)
{
    uint8_t gain_val, integ_val;
    
    // Map enum to register values
    switch (gain) {
        case TSL2591_GAIN_1X:    gain_val = TSL2591_GAIN_LOW; break;
        case TSL2591_GAIN_25X:   gain_val = TSL2591_GAIN_MED; break;
        case TSL2591_GAIN_428X:  gain_val = TSL2591_GAIN_HIGH; break;
        case TSL2591_GAIN_9876X: gain_val = TSL2591_GAIN_MAX; break;
        default:                 gain_val = TSL2591_GAIN_MED; break;
    }
    
    switch (integration) {
        case TSL2591_INTEG_100MS: integ_val = TSL2591_INTEGRATIONTIME_100MS; break;
        case TSL2591_INTEG_200MS: integ_val = TSL2591_INTEGRATIONTIME_200MS; break;
        case TSL2591_INTEG_300MS: integ_val = TSL2591_INTEGRATIONTIME_300MS; break;
        case TSL2591_INTEG_400MS: integ_val = TSL2591_INTEGRATIONTIME_400MS; break;
        case TSL2591_INTEG_500MS: integ_val = TSL2591_INTEGRATIONTIME_500MS; break;
        case TSL2591_INTEG_600MS: integ_val = TSL2591_INTEGRATIONTIME_600MS; break;
        default:                  integ_val = TSL2591_INTEGRATIONTIME_100MS; break;
    }
    
    uint8_t control = gain_val | integ_val;
    
    ESP_LOGD(TAG, "Setting timing: gain=0x%02X, integration=0x%02X", gain_val, integ_val);
    
    esp_err_t ret = tsl2591_write_register(i2c_num, address, TSL2591_REGISTER_CONTROL, control);
    if (ret == ESP_OK) {
        current_gain = gain;
        current_integration = integration;
    }
    
    return ret;
}

esp_err_t tsl2591_read_data(i2c_port_t i2c_num, uint8_t address, tsl2591_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(data, 0, sizeof(tsl2591_data_t));
    
    ESP_LOGD(TAG, "Reading light sensor data");
    
    // Wait for integration time
    uint16_t delay_ms;
    switch (current_integration) {
        case TSL2591_INTEG_100MS: delay_ms = 120; break;
        case TSL2591_INTEG_200MS: delay_ms = 220; break;
        case TSL2591_INTEG_300MS: delay_ms = 320; break;
        case TSL2591_INTEG_400MS: delay_ms = 420; break;
        case TSL2591_INTEG_500MS: delay_ms = 520; break;
        case TSL2591_INTEG_600MS: delay_ms = 620; break;
        default: delay_ms = 120; break;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Read all 4 bytes (2 channels, 16-bit each)
    uint8_t buffer[4];
    esp_err_t ret = tsl2591_read_registers(i2c_num, address, 
                                            TSL2591_REGISTER_C0DATAL, buffer, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }
    
    // Combine bytes into 16-bit values (little-endian)
    data->ch0 = (uint16_t)buffer[1] << 8 | buffer[0];
    data->ch1 = (uint16_t)buffer[3] << 8 | buffer[2];
    
    // Calculate visible light (CH0 - CH1)
    if (data->ch0 >= data->ch1) {
        data->visible = data->ch0 - data->ch1;
    } else {
        data->visible = 0;
    }
    
    // Calculate lux
    data->lux = tsl2591_calculate_lux(data->ch0, data->ch1, 
                                      current_gain, current_integration);
    
    data->valid = (data->lux >= 0.0f);
    
    ESP_LOGD(TAG, "CH0: %u, CH1: %u, Visible: %u, Lux: %.2f", 
             data->ch0, data->ch1, data->visible, data->lux);
    
    return ESP_OK;
}

float tsl2591_calculate_lux(uint16_t ch0, uint16_t ch1, 
                            tsl2591_gain_t gain, tsl2591_integration_t integration)
{
    // Check for overflow
    if (ch0 == 0xFFFF || ch1 == 0xFFFF) {
        ESP_LOGW(TAG, "Sensor overflow detected");
        return -1.0f;
    }
    
    // Check for zero
    if (ch0 == 0) {
        return 0.0f;
    }
    
    // Get integration time in milliseconds
    float atime;
    switch (integration) {
        case TSL2591_INTEG_100MS: atime = 100.0f; break;
        case TSL2591_INTEG_200MS: atime = 200.0f; break;
        case TSL2591_INTEG_300MS: atime = 300.0f; break;
        case TSL2591_INTEG_400MS: atime = 400.0f; break;
        case TSL2591_INTEG_500MS: atime = 500.0f; break;
        case TSL2591_INTEG_600MS: atime = 600.0f; break;
        default: atime = 100.0f; break;
    }
    
    // Get gain multiplier
    float again;
    switch (gain) {
        case TSL2591_GAIN_1X:    again = 1.0f; break;
        case TSL2591_GAIN_25X:   again = 25.0f; break;
        case TSL2591_GAIN_428X:  again = 428.0f; break;
        case TSL2591_GAIN_9876X: again = 9876.0f; break;
        default: again = 25.0f; break;
    }
    
    // Calculate CPL (Counts Per Lux)
    float cpl = (atime * again) / TSL2591_LUX_DF;
    
    // Calculate lux using simplified formula
    // lux = (CH0 - (TSL2591_LUX_COEFB * CH1)) / cpl
    float lux1 = ((float)ch0 - (TSL2591_LUX_COEFB * (float)ch1)) / cpl;
    float lux2 = ((TSL2591_LUX_COEFC * (float)ch0) - (TSL2591_LUX_COEFD * (float)ch1)) / cpl;
    
    // Return the maximum of the two calculations
    float lux = (lux1 > lux2) ? lux1 : lux2;
    
    // Ensure non-negative
    return (lux < 0.0f) ? 0.0f : lux;
}

esp_err_t tsl2591_read_device_id(i2c_port_t i2c_num, uint8_t address, uint8_t *device_id)
{
    if (device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return tsl2591_read_register(i2c_num, address, TSL2591_REGISTER_DEVICE_ID, device_id);
}
