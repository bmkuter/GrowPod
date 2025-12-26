/**
 * @file sht45_sensor.c
 * @brief Low-level driver implementation for Sensirion SHT45 sensor
 */

#include "sht45_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SHT45";

// Internal helper functions
static esp_err_t sht45_send_command(i2c_port_t i2c_num, uint8_t address, uint8_t command);
static esp_err_t sht45_read_bytes(i2c_port_t i2c_num, uint8_t address, 
                                  uint8_t *data, size_t len);

/**
 * @brief Calculate CRC-8 for SHT45 data validation
 */
uint8_t sht45_calculate_crc(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;  // Initialization value
    
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;  // Polynomial: 0x31
            } else {
                crc = (crc << 1);
            }
        }
    }
    
    return crc;
}

/**
 * @brief Send a command to the sensor
 */
static esp_err_t sht45_send_command(i2c_port_t i2c_num, uint8_t address, uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X: %s", command, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read bytes from the sensor using working I2C pattern
 */
// static esp_err_t sht45_read_bytes(i2c_port_t i2c_num, uint8_t address,
//                                    uint8_t *data, size_t len)
// {
//         // Internal function to update sensor measurements if needed
//     static float cached_temperature = 0.0f;
//     static float cached_humidity = 0.0f;
//     static TickType_t last_measurement_ticks = 0;

//     TickType_t now = xTaskGetTickCount();
//     if(now - last_measurement_ticks < pdMS_TO_TICKS(100))
//         return ESP_FAIL; // use cached values

//     uint8_t cmd = 0xFD; // No-heater high precision command
//     esp_err_t err;

//     // I2C write command
//     i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
//     i2c_master_start(i2c_cmd);
//     i2c_master_write_byte(i2c_cmd, (SHT45_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(i2c_cmd, cmd, true);
//     i2c_master_stop(i2c_cmd);
//     err = i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd, pdMS_TO_TICKS(250));
//     i2c_cmd_link_delete(i2c_cmd);
//     if(err != ESP_OK) {
//         ESP_LOGE(TAG, "I2C write failed");
//         return ESP_FAIL;
//     }

//     // Wait for measurement (50 ms for high precision)
//     vTaskDelay(pdMS_TO_TICKS(50));

//     uint8_t readbuffer[6] = {0};
//     i2c_cmd = i2c_cmd_link_create();
//     i2c_master_start(i2c_cmd);
//     i2c_master_write_byte(i2c_cmd, (SHT45_I2C_ADDR << 1) | I2C_MASTER_READ, true);
//     i2c_master_read(i2c_cmd, readbuffer, 6, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(i2c_cmd);
//     err = i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd, pdMS_TO_TICKS(250));
//     i2c_cmd_link_delete(i2c_cmd);
//     if(err != ESP_OK) {
//         ESP_LOGE(TAG, "I2C read failed");
//         return ESP_FAIL;
//     }

//     // Verify CRC for temperature (first two bytes) and humidity (bytes 3-4)
//     if(sht45_calculate_crc(readbuffer, 2) != readbuffer[2] ||
//        sht45_calculate_crc(readbuffer + 3, 2) != readbuffer[5])
//     {
//         ESP_LOGE(TAG, "CRC mismatch");
//         // Print received info vs expected for debugging
//         ESP_LOGE(TAG, "Temp bytes: 0x%02X 0x%02X (CRC: 0x%02X, Calc: 0x%02X)",
//                  readbuffer[0], readbuffer[1], readbuffer[2],
//                  sht45_calculate_crc(readbuffer, 2));
//         return ESP_FAIL;
//     }

//     uint16_t temp_ticks = ((uint16_t)readbuffer[0] << 8) | readbuffer[1];
//     uint16_t hum_ticks  = ((uint16_t)readbuffer[3] << 8) | readbuffer[4];

//     cached_temperature = -45 + 175 * ((float)temp_ticks / 65535);
//     cached_humidity = -6 + 125 * ((float)hum_ticks / 65535);
//     if(cached_humidity < 0.0f) cached_humidity = 0.0f;
//     if(cached_humidity > 100.0f) cached_humidity = 100.0f;

//     // Print debug info
//     ESP_LOGI(TAG, "SHT45 Measurement: Temp=%.2f C, Hum=%.2f %%RH",
//              cached_temperature, cached_humidity);

//     last_measurement_ticks = now;

//     return ESP_OK;
// }

static esp_err_t sht45_read_bytes(i2c_port_t i2c_num, uint8_t address,
                                   uint8_t *data, size_t len) {
    if (len == 0 || data == NULL) {
        ESP_LOGE(TAG, "Invalid arguments to sht45_read_bytes");
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    
    // Read all bytes with LAST_NACK (proper SHT45 protocol)
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(2000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %d bytes from 0x%02X: %s", 
                 len, address, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Successfully read %d bytes from 0x%02X", len, address);
        // Log raw data for debugging
        if (len == 6) {
            ESP_LOGD(TAG, "Raw: %02X %02X %02X %02X %02X %02X",
                     data[0], data[1], data[2], data[3], data[4], data[5]);
        }
    }
    
    return ret;
}

/**
 * @brief Initialize SHT45 sensor
 */
esp_err_t sht45_init(i2c_port_t i2c_num, uint8_t address) {
    // ESP_LOGI(TAG, "Initializing SHT45 sensor at address 0x%02X", address);
    
    // esp_err_t ret;
    
    // // First, try a simple I2C probe to see if device responds
    // ESP_LOGI(TAG, "Probing I2C address 0x%02X...", address);
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    // i2c_cmd_link_delete(cmd);
    
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "No device found at 0x%02X: %s", address, esp_err_to_name(ret));
    //     ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    //     for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    //         cmd = i2c_cmd_link_create();
    //         i2c_master_start(cmd);
    //         i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    //         i2c_master_stop(cmd);
    //         esp_err_t scan_ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(50));
    //         i2c_cmd_link_delete(cmd);
    //         if (scan_ret == ESP_OK) {
    //             ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
    //         }
    //     }
    //     return ret;
    // }
    
    // ESP_LOGI(TAG, "Device responded at 0x%02X", address);

    // // Perform soft reset
    // ESP_LOGI(TAG, "Performing soft reset...");
    // ret = sht45_soft_reset(i2c_num, address);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to reset sensor: %s", esp_err_to_name(ret));
    // }
    
    // // Wait for sensor to be ready after reset
    // vTaskDelay(pdMS_TO_TICKS(10));
    
    // // Try to read serial number to verify communication
    // uint32_t serial;
    // ESP_LOGI(TAG, "Reading serial number...");
    // ret = sht45_read_serial(i2c_num, address, &serial);
    // if (ret == ESP_OK) {
    //     ESP_LOGI(TAG, "SHT45 initialized successfully at 0x%02X. Serial: 0x%08lX", address, serial);
    // } else {
    //     ESP_LOGW(TAG, "Could not read serial number from 0x%02X: %s", address, esp_err_to_name(ret));
    // }
    
    return ESP_OK;
}

/**
 * @brief Perform soft reset
 */
esp_err_t sht45_soft_reset(i2c_port_t i2c_num, uint8_t address) {
    ESP_LOGD(TAG, "Performing soft reset");
    return sht45_send_command(i2c_num, address, SHT45_CMD_SOFT_RESET);
}

/**
 * @brief Read temperature and humidity
 */
esp_err_t sht45_read_data(i2c_port_t i2c_num, uint8_t address,
                          sht45_precision_t precision, sht45_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize data structure
    data->valid = false;
    data->temperature_c = 0.0f;
    data->humidity_rh = 0.0f;
    
    // Select command and delay based on precision
    uint8_t command;
    uint32_t delay_ms;
    
    switch (precision) {
        case SHT45_PRECISION_HIGH:
            command = SHT45_CMD_MEASURE_HIGH_PREC;
            delay_ms = SHT45_DELAY_HIGH_PREC_MS;
            break;
        case SHT45_PRECISION_MEDIUM:
            command = SHT45_CMD_MEASURE_MED_PREC;
            delay_ms = SHT45_DELAY_MED_PREC_MS;
            break;
        case SHT45_PRECISION_LOW:
            command = SHT45_CMD_MEASURE_LOW_PREC;
            delay_ms = SHT45_DELAY_LOW_PREC_MS;
            break;
        default:
            ESP_LOGE(TAG, "Invalid precision mode");
            return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Triggering measurement (precision=%d, delay=%lums)", precision, delay_ms);
    
    // Trigger measurement
    esp_err_t ret = sht45_send_command(i2c_num, address, command);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement");
        return ret;
    }
    
    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Read 6 bytes: temp_msb, temp_lsb, temp_crc, hum_msb, hum_lsb, hum_crc
    uint8_t raw_data[6] = {0};
    ret = sht45_read_bytes(i2c_num, address, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }
    
    ESP_LOGD(TAG, "Raw data: %02X %02X %02X %02X %02X %02X",
             raw_data[0], raw_data[1], raw_data[2], 
             raw_data[3], raw_data[4], raw_data[5]);
    
    // Verify temperature CRC
    uint8_t temp_crc = sht45_calculate_crc(&raw_data[0], 2);
    if (temp_crc != raw_data[2]) {
        ESP_LOGE(TAG, "Temperature CRC mismatch: expected 0x%02X, got 0x%02X",
                 temp_crc, raw_data[2]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Verify humidity CRC
    uint8_t hum_crc = sht45_calculate_crc(&raw_data[3], 2);
    if (hum_crc != raw_data[5]) {
        ESP_LOGE(TAG, "Humidity CRC mismatch: expected 0x%02X, got 0x%02X",
                 hum_crc, raw_data[5]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert raw values to physical units
    // Temperature formula: -45 + 175 * (raw_value / 65535)
    uint16_t temp_raw = (raw_data[0] << 8) | raw_data[1];
    data->temperature_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    
    // Humidity formula: 100 * (raw_value / 65535)
    uint16_t hum_raw = (raw_data[3] << 8) | raw_data[4];
    data->humidity_rh = 100.0f * ((float)hum_raw / 65535.0f);
    
    // Clamp humidity to valid range
    if (data->humidity_rh < 0.0f) data->humidity_rh = 0.0f;
    if (data->humidity_rh > 100.0f) data->humidity_rh = 100.0f;
    
    data->valid = true;
    
    ESP_LOGD(TAG, "Temperature: %.2f°C, Humidity: %.2f%%RH",
             data->temperature_c, data->humidity_rh);
    
    return ESP_OK;
}

/**
 * @brief Read temperature only
 */
esp_err_t sht45_read_temperature(i2c_port_t i2c_num, uint8_t address,
                                 sht45_precision_t precision, float *temperature_c) {
    if (temperature_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sht45_data_t data;
    esp_err_t ret = sht45_read_data(i2c_num, address, precision, &data);
    
    if (ret == ESP_OK && data.valid) {
        *temperature_c = data.temperature_c;
    }
    
    return ret;
}

/**
 * @brief Read humidity only
 */
esp_err_t sht45_read_humidity(i2c_port_t i2c_num, uint8_t address,
                              sht45_precision_t precision, float *humidity_rh) {
    if (humidity_rh == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sht45_data_t data;
    esp_err_t ret = sht45_read_data(i2c_num, address, precision, &data);
    
    if (ret == ESP_OK && data.valid) {
        *humidity_rh = data.humidity_rh;
    }
    
    return ret;
}

/**
 * @brief Read sensor serial number
 */
esp_err_t sht45_read_serial(i2c_port_t i2c_num, uint8_t address, uint32_t *serial) {
    if (serial == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send read serial command
    esp_err_t ret = sht45_send_command(i2c_num, address, SHT45_CMD_READ_SERIAL);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait a bit for the command to process
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Read 6 bytes: serial_msb_high, serial_lsb_high, crc, serial_msb_low, serial_lsb_low, crc
    uint8_t raw_data[6] = {0};
    ret = sht45_read_bytes(i2c_num, address, raw_data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Verify CRCs
    uint8_t crc1 = sht45_calculate_crc(&raw_data[0], 2);
    uint8_t crc2 = sht45_calculate_crc(&raw_data[3], 2);
    
    if (crc1 != raw_data[2] || crc2 != raw_data[5]) {
        ESP_LOGE(TAG, "Serial number CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Combine into 32-bit serial number
    *serial = ((uint32_t)raw_data[0] << 24) |
              ((uint32_t)raw_data[1] << 16) |
              ((uint32_t)raw_data[3] << 8) |
              ((uint32_t)raw_data[4]);
    
    return ESP_OK;
}
