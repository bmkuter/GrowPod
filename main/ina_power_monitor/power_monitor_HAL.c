#include "power_monitor_HAL.h"
#include "esp_log.h"
#include "ina260.h"
#include "ina219.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"

static const char *TAG = "POWER_MONITOR_HAL";

// Variables to keep track of current configuration
static power_monitor_chip_t current_chip_type = POWER_MONITOR_CHIP_INA219; // Default to INA219
static uint8_t current_i2c_address = 0x40;                                // Default address

// Initialize I2C master - reusing the same function from ina260.c for compatibility
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

// Initialize the power monitor
esp_err_t power_monitor_init(power_monitor_chip_t chip_type, uint8_t i2c_address) {
    esp_err_t ret = ESP_OK;
    
    // Store current configuration
    current_chip_type = chip_type;
    current_i2c_address = i2c_address;
    
    // Initialize the appropriate chip
    if (chip_type == POWER_MONITOR_CHIP_INA260) {
        ESP_LOGI(TAG, "Initializing INA260 at address 0x%02X", i2c_address);
        ina260_init(i2c_address);
    } else if (chip_type == POWER_MONITOR_CHIP_INA219) {
        ESP_LOGI(TAG, "Initializing INA219 at address 0x%02X", i2c_address);
        ret = ina219_init(i2c_address);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize INA219");
            return ret;
        }
        // Use 12V/3A calibration by default for GrowPod's 12V system
        ret = ina219_set_calibration_12V_3A(i2c_address);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set INA219 calibration");
            return ret;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported power monitor chip type");
        return ESP_ERR_INVALID_ARG;
    }
    
    return ret;
}

// Read current in mA
esp_err_t power_monitor_read_current(float *current_ma) {
    if (current_chip_type == POWER_MONITOR_CHIP_INA260) {
        return ina260_read_current(current_i2c_address, current_ma);
    } else if (current_chip_type == POWER_MONITOR_CHIP_INA219) {
        return ina219_read_current(current_i2c_address, current_ma);
    }
    return ESP_ERR_INVALID_STATE;
}

// Read voltage in mV
esp_err_t power_monitor_read_voltage(float *voltage_mv) {
    if (current_chip_type == POWER_MONITOR_CHIP_INA260) {
        return ina260_read_voltage(current_i2c_address, voltage_mv);
    } else if (current_chip_type == POWER_MONITOR_CHIP_INA219) {
        // INA219 has bus voltage and shunt voltage, we'll return the bus voltage for compatibility
        return ina219_read_bus_voltage(current_i2c_address, voltage_mv);
    }
    return ESP_ERR_INVALID_STATE;
}

// Read power in mW
esp_err_t power_monitor_read_power(float *power_mw) {
    if (current_chip_type == POWER_MONITOR_CHIP_INA260) {
        return ina260_read_power(current_i2c_address, power_mw);
    } else if (current_chip_type == POWER_MONITOR_CHIP_INA219) {
        return ina219_read_power(current_i2c_address, power_mw);
    }
    return ESP_ERR_INVALID_STATE;
}

// Set power saving mode
esp_err_t power_monitor_power_save(bool on) {
    if (current_chip_type == POWER_MONITOR_CHIP_INA219) {
        return ina219_power_save(current_i2c_address, on);
    }
    // Note: INA260 doesn't seem to have an explicit power save function in the current implementation
    // If needed, we could add a function to put the INA260 in shutdown mode
    return ESP_ERR_NOT_SUPPORTED;
}

// Set calibration (only applies to INA219)
esp_err_t power_monitor_set_calibration(power_monitor_calibration_t calibration) {
    esp_err_t ret = ESP_OK;
    
    // Only applies to INA219
    if (current_chip_type != POWER_MONITOR_CHIP_INA219) {
        ESP_LOGW(TAG, "Calibration only applies to INA219 chip");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Set the appropriate calibration
    switch (calibration) {
        case POWER_MONITOR_CAL_32V_2A:
            ret = ina219_set_calibration_32V_2A(current_i2c_address);
            ESP_LOGI(TAG, "Setting INA219 calibration to 32V/2A");
            break;
            
        case POWER_MONITOR_CAL_32V_1A:
            ret = ina219_set_calibration_32V_1A(current_i2c_address);
            ESP_LOGI(TAG, "Setting INA219 calibration to 32V/1A");
            break;
            
        case POWER_MONITOR_CAL_16V_400MA:
            ret = ina219_set_calibration_16V_400mA(current_i2c_address);
            ESP_LOGI(TAG, "Setting INA219 calibration to 16V/400mA");
            break;
            
        case POWER_MONITOR_CAL_12V_3A:
            ret = ina219_set_calibration_12V_3A(current_i2c_address);
            ESP_LOGI(TAG, "Setting INA219 calibration to 12V/3A");
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown calibration setting");
            return ESP_ERR_INVALID_ARG;
    }
    
    return ret;
}

// Get current chip type
power_monitor_chip_t power_monitor_get_chip_type(void) {
    return current_chip_type;
}

// Power monitor task
void power_monitor_task(void *pvParameter) {
    float current, voltage, power;
    
    // Initialize with default settings
    power_monitor_init(current_chip_type, current_i2c_address);

    while (1) {
        // Read the sensor values
        power_monitor_read_current(&current);
        power_monitor_read_voltage(&voltage);
        power_monitor_read_power(&power);
        
        // Debug log
        ESP_LOGI(TAG, "Current: %.2f mA, Voltage: %.2f mV, Power: %.2f mW", 
                 current, voltage, power);
                 
        // Delay for 1 second
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
