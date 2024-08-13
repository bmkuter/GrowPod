#include "ina260.h"

void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

esp_err_t ina260_read_register(uint8_t reg_addr, uint16_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;
    
    // Start I2C communication
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA260_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // Restart and read the register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA260_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t *)data + 1, 1, I2C_MASTER_ACK);
    i2c_master_read(cmd, (uint8_t *)data, 1, I2C_MASTER_NACK);
    
    // Stop I2C communication
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

void ina260_init(void) {
    uint16_t config = 0x40; // Example configuration
    i2c_master_init();
    ina260_read_register(INA260_REG_CONFIG, &config);
}

esp_err_t ina260_read_current(int16_t *current) {
    uint16_t raw_data;
    esp_err_t ret = ina260_read_register(INA260_REG_CURRENT, &raw_data);
    if (ret == ESP_OK) {
        *current = (int16_t)raw_data; // Convert raw data to actual current (if needed)
    }
    return ret;
}

esp_err_t ina260_read_voltage(uint16_t *voltage) {
    return ina260_read_register(INA260_REG_BUS_VOLTAGE, voltage);
}

esp_err_t ina260_read_power(uint16_t *power) {
    return ina260_read_register(INA260_REG_POWER, power);
}

void ina260_task(void *pvParameter) {
    int16_t current;
    uint16_t voltage, power;

    ina260_init();

    while (1) {
        if (ina260_read_current(&current) == ESP_OK) {
            printf("Current: %d mA\n", current);
        }

        if (ina260_read_voltage(&voltage) == ESP_OK) {
            printf("Voltage: %d mV\n", voltage);
        }

        if (ina260_read_power(&power) == ESP_OK) {
            printf("Power: %d mW\n", power);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}