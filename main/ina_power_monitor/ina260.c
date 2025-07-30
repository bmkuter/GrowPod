#include "ina260.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"

static const char *TAG = "INA260";

// Function to read from INA260 register
esp_err_t ina260_read_register(uint8_t address, uint8_t reg_addr, uint16_t *data) {
    // Read 2 bytes from 'reg_addr' at 'address'
    uint8_t buf[2];
    esp_err_t ret = i2c_master_write_read_device(
        I2C_NUM_0,
        address,
        &reg_addr,
        1,
        buf,
        2,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    if (ret == ESP_OK) {
        // Data is MSB:buf[0], LSB:buf[1]
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return ret;
}


// INA260 initialization
void ina260_init(uint8_t address) {
    uint16_t config = 0x40;  // Example configuration
    esp_err_t ret = ina260_read_register(address, INA260_REG_CONFIG, &config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "INA260 at 0x%02X initialized with config: 0x%04x", address, config);
    } else {
        ESP_LOGE(TAG, "Failed to initialize INA260 at 0x%02X", address);
    }
}

// Read current from INA260, returns mA (scaled by 1.25)
esp_err_t ina260_read_current(uint8_t address, float *current) {
    uint16_t raw_data;
    esp_err_t ret = ina260_read_register(address, INA260_REG_CURRENT, &raw_data);
    if (ret == ESP_OK) {
        *current = (int16_t)raw_data * INA260_SCALING_CURRENT;  // Scale by 1.25
    }
    return ret;
}

// Read voltage from INA260, returns mV (scaled by 1.25)
esp_err_t ina260_read_voltage(uint8_t address, float *voltage) {
    uint16_t raw_data;
    esp_err_t ret = ina260_read_register(address, INA260_REG_BUS_VOLTAGE, &raw_data);
    if (ret == ESP_OK) {
        *voltage = raw_data * INA260_SCALING_VOLTAGE;  // Scale by 1.25
    }
    return ret;
}

// Read power from INA260, returns mW (scaled by 10)
esp_err_t ina260_read_power(uint8_t address, float *power) {
    uint16_t raw_data;
    esp_err_t ret = ina260_read_register(address, INA260_REG_POWER, &raw_data);
    if (ret == ESP_OK) {
        *power = raw_data * INA260_SCALING_POWER;  // Scale by 10
    }
    return ret;
}