#include "ina219.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"

static const char *TAG = "INA219";

// Calibration values
static uint16_t ina219_cal_value = 0;
static uint32_t ina219_current_divider_ma = 0;
static float ina219_power_multiplier_mw = 0.0f;

// Function to read from INA219 register
esp_err_t ina219_read_register(uint8_t address, uint8_t reg_addr, uint16_t *data) {
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

// Function to write to INA219 register
esp_err_t ina219_write_register(uint8_t address, uint8_t reg_addr, uint16_t data) {
    uint8_t write_buf[3];
    write_buf[0] = reg_addr;
    write_buf[1] = (data >> 8) & 0xFF;  // MSB
    write_buf[2] = data & 0xFF;         // LSB
    
    esp_err_t ret = i2c_master_write_to_device(
        I2C_NUM_0,
        address,
        write_buf,
        3,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
    
    return ret;
}

// INA219 initialization
esp_err_t ina219_init(uint8_t address) {
    // Note: I2C bus is now initialized by sensor_manager_init() in main.c
    // (GPIO 42 SDA, GPIO 41 SCL, 400kHz)
    
    // Set calibration for 32V, 2A by default
    esp_err_t ret = ina219_set_calibration_32V_2A(address);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "INA219 at 0x%02X initialized", address);
    } else {
        ESP_LOGE(TAG, "Failed to initialize INA219 at 0x%02X", address);
    }
    
    return ret;
}

// Set calibration for 32V, 2A range
esp_err_t ina219_set_calibration_32V_2A(uint8_t address) {
    // Calibration settings for 32V, 2A (0.1 ohm resistor)
    ina219_cal_value = 4096;
    ina219_current_divider_ma = 10;     // Current LSB = 100uA per bit (1000/100 = 10)
    ina219_power_multiplier_mw = 2.0f;  // Power LSB = 2mW per bit
    
    // Set Calibration register
    esp_err_t ret = ina219_write_register(address, INA219_REG_CALIBRATION, ina219_cal_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set Config register
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                     INA219_CONFIG_GAIN_8_320MV |
                     INA219_CONFIG_BADCRES_12BIT |
                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    
    return ina219_write_register(address, INA219_REG_CONFIG, config);
}

// Set calibration for 32V, 1A range
esp_err_t ina219_set_calibration_32V_1A(uint8_t address) {
    // Calibration settings for 32V, 1A (0.1 ohm resistor)
    ina219_cal_value = 10240;
    ina219_current_divider_ma = 25;     // Current LSB = 40uA per bit (1000/40 = 25)
    ina219_power_multiplier_mw = 0.8f;  // Power LSB = 800uW per bit
    
    // Set Calibration register
    esp_err_t ret = ina219_write_register(address, INA219_REG_CALIBRATION, ina219_cal_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set Config register
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                     INA219_CONFIG_GAIN_8_320MV |
                     INA219_CONFIG_BADCRES_12BIT |
                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    
    return ina219_write_register(address, INA219_REG_CONFIG, config);
}

// Set calibration for 16V, 400mA range
esp_err_t ina219_set_calibration_16V_400mA(uint8_t address) {
    // Calibration settings for 16V, 400mA (0.1 ohm resistor)
    ina219_cal_value = 8192;
    ina219_current_divider_ma = 20;     // Current LSB = 50uA per bit (1000/50 = 20)
    ina219_power_multiplier_mw = 1.0f;  // Power LSB = 1mW per bit
    
    // Set Calibration register
    esp_err_t ret = ina219_write_register(address, INA219_REG_CALIBRATION, ina219_cal_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set Config register for 16V range, gain 1 (40mV range)
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                     0x0000 |  // Gain 1, 40mV range
                     INA219_CONFIG_BADCRES_12BIT |
                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    
    return ina219_write_register(address, INA219_REG_CONFIG, config);
}

// Set calibration for 12V, 3A range - Optimized for GrowPod's 12V system
esp_err_t ina219_set_calibration_12V_3A(uint8_t address) {
    // Calibration settings for 12V, 3A (0.1 ohm resistor)
    // VBUS_MAX = 12V
    // VSHUNT_MAX = 0.32V          (Assumes Gain 8, 320mV)
    // RSHUNT = 0.1                (Resistor value in ohms)

    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 3.2A

    // 2. Determine max expected current
    // MaxExpected_I = 3.0A (slightly below max possible for safety margin)

    // 3. Calculate possible range of LSBs
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.0000915            (91.5uA per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0.000732             (732uA per bit)

    // 4. Choose an LSB between the min and max values
    // CurrentLSB = 0.0001 (100uA per bit) - Good balance of precision and range

    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 4096 (0x1000)
    ina219_cal_value = 4096;

    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.002 (2mW per bit)
    ina219_current_divider_ma = 10;     // Current LSB = 100uA per bit (1000/100 = 10)
    ina219_power_multiplier_mw = 2.0f;  // Power LSB = 2mW per bit

    // 7. Compute the maximum current and shunt voltage values before overflow
    // Max_Current = Current_LSB * 32767
    // Max_Current = 3.2767A before overflow (this exceeds our max expected)
    
    // Set Calibration register
    esp_err_t ret = ina219_write_register(address, INA219_REG_CALIBRATION, ina219_cal_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set Config register - using 16V range for better precision on 12V system
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |   // Use 16V range for 12V system
                     INA219_CONFIG_GAIN_8_320MV |         // Gain 8, 320mV range for shunt
                     INA219_CONFIG_BADCRES_12BIT |        // 12-bit bus ADC resolution
                     INA219_CONFIG_SADCRES_12BIT_1S_532US | // 12-bit shunt ADC resolution
                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS; // Continuous mode
    
    ESP_LOGI(TAG, "Setting 12V/3A calibration for INA219 at 0x%02X", address);
    return ina219_write_register(address, INA219_REG_CONFIG, config);
}

// Read bus voltage from INA219 (in mV)
esp_err_t ina219_read_bus_voltage(uint8_t address, float *voltage) {
    uint16_t raw_data;
    esp_err_t ret = ina219_read_register(address, INA219_REG_BUSVOLTAGE, &raw_data);
    if (ret == ESP_OK) {
        // Shift right 3 bits to drop CNVR and OVF flags, multiply by 4 (per datasheet)
        *voltage = ((int16_t)(raw_data >> 3)) * 4;  // Result in mV
    }
    return ret;
}

// Read shunt voltage from INA219 (in mV)
esp_err_t ina219_read_shunt_voltage(uint8_t address, float *voltage) {
    uint16_t raw_data;
    esp_err_t ret = ina219_read_register(address, INA219_REG_SHUNTVOLTAGE, &raw_data);
    if (ret == ESP_OK) {
        // LSB is 10μV, convert to mV
        *voltage = ((int16_t)raw_data) * 0.01f;  // Result in mV
    }
    return ret;
}

// Read current from INA219 (in mA)
esp_err_t ina219_read_current(uint8_t address, float *current) {
    // Ensure calibration is set (in case of reset)
    ina219_write_register(address, INA219_REG_CALIBRATION, ina219_cal_value);
    
    uint16_t raw_data;
    esp_err_t ret = ina219_read_register(address, INA219_REG_CURRENT, &raw_data);
    if (ret == ESP_OK) {
        // Convert raw value to mA using the current divider
        *current = ((int16_t)raw_data) / (float)ina219_current_divider_ma;
    }
    return ret;
}

// Read power from INA219 (in mW)
esp_err_t ina219_read_power(uint8_t address, float *power) {
    // Ensure calibration is set (in case of reset)
    ina219_write_register(address, INA219_REG_CALIBRATION, ina219_cal_value);
    
    uint16_t raw_data;
    esp_err_t ret = ina219_read_register(address, INA219_REG_POWER, &raw_data);
    if (ret == ESP_OK) {
        // Convert raw value to mW using the power multiplier
        *power = raw_data * ina219_power_multiplier_mw;
    }
    return ret;
}

// Power save mode
esp_err_t ina219_power_save(uint8_t address, bool on) {
    uint16_t config;
    esp_err_t ret = ina219_read_register(address, INA219_REG_CONFIG, &config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (on) {
        // Set mode to power-down (bits 0-2 = 0)
        config &= ~0x0007;
    } else {
        // Set mode to shunt and bus, continuous (bits 0-2 = 7)
        config &= ~0x0007;  // Clear mode bits
        config |= 0x0007;   // Set to continuous mode
    }
    
    return ina219_write_register(address, INA219_REG_CONFIG, config);
}