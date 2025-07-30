#include "i2c_motor_driver.h"
#include "esp_log.h"
#include <string.h>
#include "power_monitor_HAL.h"  // Include for shared I2C initialization

static const char *TAG = "I2C_MOTOR";

// I2C port used for motor shield communication
static i2c_port_t i2c_motor_port = I2C_NUM_0;

// PCA9685 registers (Adafruit Motor Shield uses this chip)
#define PCA9685_MODE1            0x00
#define PCA9685_MODE2            0x01
#define PCA9685_PRESCALE         0xFE
#define PCA9685_SUBADR1          0x02
#define PCA9685_SUBADR2          0x03
#define PCA9685_SUBADR3          0x04
#define PCA9685_LED0_ON_L        0x06
#define PCA9685_ALL_LED_ON_L     0xFA

// Motor control offsets (based on Adafruit_MotorShield.cpp)
// Each DC motor uses 3 PWM outputs for control
typedef struct {
    uint8_t pwm_pin;
    uint8_t in1_pin;
    uint8_t in2_pin;
} motor_pins_t;

// Motor pin definitions (from Adafruit_MotorShield.cpp)
static const motor_pins_t motor_pins[4] = {
    {8, 10, 9},    // Motor 1
    {13, 11, 12},  // Motor 2
    {2, 4, 3},     // Motor 3
    {7, 5, 6}      // Motor 4
};

// Helper function to write to a register on the PCA9685
static esp_err_t pca9685_write_register(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MOTOR_SHIELD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_motor_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// Helper function to set PWM value for a specific pin
static esp_err_t pca9685_set_pwm(uint8_t pin, uint16_t value)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    if (value > 4095) {
        // Special case for full-on
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MOTOR_SHIELD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, PCA9685_LED0_ON_L + 4 * pin, true);
        i2c_master_write_byte(cmd, 0x00, true);  // ON_L
        i2c_master_write_byte(cmd, 0x10, true);  // ON_H (bit 4 = full on)
        i2c_master_write_byte(cmd, 0x00, true);  // OFF_L
        i2c_master_write_byte(cmd, 0x00, true);  // OFF_H
        i2c_master_stop(cmd);
    } else {
        // Normal PWM
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MOTOR_SHIELD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, PCA9685_LED0_ON_L + 4 * pin, true);
        i2c_master_write_byte(cmd, 0x00, true);  // ON_L
        i2c_master_write_byte(cmd, 0x00, true);  // ON_H
        i2c_master_write_byte(cmd, value & 0xFF, true);          // OFF_L
        i2c_master_write_byte(cmd, (value >> 8) & 0x0F, true);   // OFF_H
        i2c_master_stop(cmd);
    }
    
    ret = i2c_master_cmd_begin(i2c_motor_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM for pin %d: %s", pin, esp_err_to_name(ret));
    }
    
    return ret;
}

// Helper function to set a pin high or low
static esp_err_t pca9685_set_pin(uint8_t pin, bool value)
{
    if (value == true) {
        return pca9685_set_pwm(pin, 4096); // Full on
    } else {
        return pca9685_set_pwm(pin, 0);    // Off
    }
}

esp_err_t i2c_motor_driver_init(i2c_port_t i2c_port, int sda_pin, int scl_pin, uint32_t clock_speed)
{
    esp_err_t ret;
    
    // Use the shared I2C port from the power monitor
    i2c_motor_port = I2C_MASTER_NUM;
    
    // Use the shared I2C initialization function from power_monitor_HAL
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Note: We're now using the I2C configuration from the power monitor
    // (SDA: GPIO_NUM_42, SCL: GPIO_NUM_41, 100kHz)
    
    // Reset the PCA9685
    ret = pca9685_write_register(PCA9685_MODE1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset PCA9685");
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Set MODE1 register: enable auto-increment, normal mode
    ret = pca9685_write_register(PCA9685_MODE1, 0x20); // Auto increment enabled
    if (ret != ESP_OK) {
        return ret;
    }

    // Set MODE2 register: output logic state not inverted, outputs change on STOP
    ret = pca9685_write_register(PCA9685_MODE2, 0x04);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set PWM frequency to 1600 Hz (default from Adafruit library)
    uint8_t prescale = 25000000 / (4096 * 1600) - 1; // Based on 25MHz internal clock
    
    // To set prescale, we need to set sleep mode first
    ret = pca9685_write_register(PCA9685_MODE1, 0x30); // Set sleep bit
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = pca9685_write_register(PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wake up
    ret = pca9685_write_register(PCA9685_MODE1, 0x20);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Final mode: enable auto increment
    ret = pca9685_write_register(PCA9685_MODE1, 0xA0);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Turn off all outputs
    for (uint8_t i = 0; i < 16; i++) {
        pca9685_set_pwm(i, 0);
    }
    
    ESP_LOGI(TAG, "I2C motor driver initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_motor_set_speed(uint8_t motor_num, uint8_t speed)
{
    if (motor_num < 1 || motor_num > 4) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert to internal index
    uint8_t idx = motor_num - 1;
    
    // Scale from 0-100% to 0-4095 (12-bit PWM)
    uint16_t pwm_value = (uint16_t)((speed * 4095) / 100);
    
    return pca9685_set_pwm(motor_pins[idx].pwm_pin, pwm_value);
}

esp_err_t i2c_motor_run(uint8_t motor_num, uint8_t command)
{
    if (motor_num < 1 || motor_num > 4) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert to internal index
    uint8_t idx = motor_num - 1;
    esp_err_t ret = ESP_OK;
    
    switch (command) {
        case MOTOR_FORWARD:
            ret = pca9685_set_pin(motor_pins[idx].in2_pin, false); // Take low first to avoid 'break'
            if (ret != ESP_OK) return ret;
            ret = pca9685_set_pin(motor_pins[idx].in1_pin, true);
            break;
            
        case MOTOR_BACKWARD:
            ret = pca9685_set_pin(motor_pins[idx].in1_pin, false); // Take low first to avoid 'break'
            if (ret != ESP_OK) return ret;
            ret = pca9685_set_pin(motor_pins[idx].in2_pin, true);
            break;
            
        case MOTOR_RELEASE:
            ret = pca9685_set_pin(motor_pins[idx].in1_pin, false);
            if (ret != ESP_OK) return ret;
            ret = pca9685_set_pin(motor_pins[idx].in2_pin, false);
            break;
            
        default:
            ESP_LOGE(TAG, "Invalid motor command: %d", command);
            return ESP_ERR_INVALID_ARG;
    }
    
    return ret;
}

esp_err_t i2c_motor_set(uint8_t motor_num, uint8_t speed, uint8_t command)
{
    esp_err_t ret;
    
    ret = i2c_motor_set_speed(motor_num, speed);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_motor_run(motor_num, command);
}

esp_err_t i2c_motor_stop_all(void)
{
    esp_err_t ret = ESP_OK;
    
    for (uint8_t motor = 1; motor <= 4; motor++) {
        ret = i2c_motor_set(motor, 0, MOTOR_RELEASE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop motor %d", motor);
            return ret;
        }
    }
    
    return ESP_OK;
}

esp_err_t i2c_led_set_brightness(uint8_t brightness)
{
    if (brightness > 100) {
        brightness = 100;
    }
    
    ESP_LOGI(TAG, "Setting LED brightness to %u%%", brightness);
    
    // We're reusing the same motor channel that was previously used for the air pump
    if (brightness == 0) {
        // Turn off LED
        return i2c_motor_set(LED_CONTROL, 0, MOTOR_RELEASE);
    } else {
        // Set LED brightness using forward direction (same as with motors)
        // This ensures proper PWM control
        return i2c_motor_set(LED_CONTROL, brightness, MOTOR_FORWARD);
    }
}
