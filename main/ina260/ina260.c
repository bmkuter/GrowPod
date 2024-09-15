#include "ina260.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "INA260";

// Initialize I2C master
esp_err_t i2c_master_init(void) {
    static bool initialized = false;
    if (initialized) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        // Driver already installed
        ESP_LOGW(TAG, "I2C driver already installed");
        initialized = true;
        return ESP_OK;
    }
    ESP_ERROR_CHECK(err);
    initialized = true;
    return ESP_OK;
}

// Function to read from INA260 register
esp_err_t ina260_read_register(uint8_t address, uint8_t reg_addr, uint16_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    // Start I2C communication
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Restart and read the register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, ((uint8_t*)data) + 1, I2C_MASTER_ACK);  // Read MSB
    i2c_master_read_byte(cmd, ((uint8_t*)data), I2C_MASTER_NACK);     // Read LSB

    // Stop I2C communication
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

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

// INA260 task to periodically read current, voltage, and power
void ina260_task(void *pvParameter) {
    float current_led, voltage_led, power_led;
    float current_drain, voltage_drain, power_drain;
    float current_source, voltage_source, power_source;
    float current_air, voltage_air, power_air;
    
    i2c_master_init();

    ina260_init(INA260_LED_ADDRESS);
    ina260_init(INA260_DRAIN_ADDRESS);
    ina260_init(INA260_SOURCE_ADDRESS);
    ina260_init(INA260_AIR_ADDRESS);

    while (1) {
        // Read LED Lane
        ina260_read_current(INA260_LED_ADDRESS, &current_led);
        ina260_read_voltage(INA260_LED_ADDRESS, &voltage_led);
        ina260_read_power(INA260_LED_ADDRESS, &power_led);

        // Read Drain Lane
        ina260_read_current(INA260_DRAIN_ADDRESS, &current_drain);
        ina260_read_voltage(INA260_DRAIN_ADDRESS, &voltage_drain);
        ina260_read_power(INA260_DRAIN_ADDRESS, &power_drain);

        // Read Source Lane
        ina260_read_current(INA260_SOURCE_ADDRESS, &current_source);
        ina260_read_voltage(INA260_SOURCE_ADDRESS, &voltage_source);
        ina260_read_power(INA260_SOURCE_ADDRESS, &power_source);

        // Read Air Lane
        ina260_read_current(INA260_AIR_ADDRESS, &current_air);
        ina260_read_voltage(INA260_AIR_ADDRESS, &voltage_air);
        ina260_read_power(INA260_AIR_ADDRESS, &power_air);

        // // Print the results in a row-column view
        // printf("\n---------------------------------------------------------\n");
        // printf("| Lane   | Current (mA) | Voltage (mV) | Power (mW)      |\n");
        // printf("---------------------------------------------------------\n");
        // printf("| LED    | %12.2f | %12.2f | %12.2f   |\n", current_led, voltage_led, power_led);
        // printf("| Drain  | %12.2f | %12.2f | %12.2f   |\n", current_drain, voltage_drain, power_drain);
        // printf("| Source | %12.2f | %12.2f | %12.2f   |\n", current_source, voltage_source, power_source);
        // printf("| Air    | %12.2f | %12.2f | %12.2f   |\n", current_air, voltage_air, power_air);
        // printf("---------------------------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
    }
}
