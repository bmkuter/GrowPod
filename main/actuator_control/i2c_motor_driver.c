#include "i2c_motor_driver.h"
#include "esp_log.h"
#include <string.h>
#include "power_monitor_HAL.h"  // Include for shared I2C initialization
#include "filesystem/config_manager.h"  // For JSON-based configuration

static const char *TAG = "I2C_MOTOR";

// Motor direction configuration - stored in RAM and NVS
static motor_direction_config_t s_motor_direction_config = {
    .motor1_inverted = false,
    .motor2_inverted = false, 
    .motor3_inverted = false,
    .motor4_inverted = false
};

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

// Helper function to write to a register on a specific PCA9685 address
static esp_err_t pca9685_write_register_addr(uint8_t addr, uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_motor_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register 0x%02x on address 0x%02x: %s", reg, addr, esp_err_to_name(ret));
    }
    return ret;
}

// Helper function to set PWM value for a specific pin on a specific address
static esp_err_t pca9685_set_pwm_addr(uint8_t addr, uint8_t pin, uint16_t value)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    if (value > 4095) {
        // Special case for full-on
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, PCA9685_LED0_ON_L + 4 * pin, true);
        i2c_master_write_byte(cmd, 0x00, true);  // ON_L
        i2c_master_write_byte(cmd, 0x10, true);  // ON_H (bit 4 = full on)
        i2c_master_write_byte(cmd, 0x00, true);  // OFF_L
        i2c_master_write_byte(cmd, 0x00, true);  // OFF_H
        i2c_master_stop(cmd);
    } else {
        // Normal PWM
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
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
        ESP_LOGE(TAG, "Failed to set PWM for pin %d on address 0x%02x: %s", pin, addr, esp_err_to_name(ret));
    }
    
    return ret;
}

// Helper function to set a pin high or low on a specific address
static esp_err_t pca9685_set_pin_addr(uint8_t addr, uint8_t pin, bool value)
{
    if (value == true) {
        return pca9685_set_pwm_addr(addr, pin, 4096); // Full on
    } else {
        return pca9685_set_pwm_addr(addr, pin, 0);    // Off
    }
}

// Helper function to initialize a single PCA9685 shield at a specific address
static esp_err_t pca9685_init_shield(uint8_t addr, const char *name)
{
    esp_err_t ret;
    
    // Reset the PCA9685
    ret = pca9685_write_register_addr(addr, PCA9685_MODE1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset %s at 0x%02x", name, addr);
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Set MODE1 register: enable auto-increment, normal mode
    ret = pca9685_write_register_addr(addr, PCA9685_MODE1, 0x20); // Auto increment enabled
    if (ret != ESP_OK) {
        return ret;
    }

    // Set MODE2 register: output logic state not inverted, outputs change on STOP
    ret = pca9685_write_register_addr(addr, PCA9685_MODE2, 0x04);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set PWM frequency to maximum ~1526 Hz to reduce audible whine
    // Formula: freq = 25MHz / (4096 * (prescale + 1))
    // For max frequency, use minimum prescale value of 3
    // This gives: 25000000 / (4096 * 4) = ~1526 Hz
    uint8_t prescale = 3; // Minimum prescale for maximum frequency (~1526 Hz)
    
    // To set prescale, we need to set sleep mode first
    ret = pca9685_write_register_addr(addr, PCA9685_MODE1, 0x30); // Set sleep bit
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = pca9685_write_register_addr(addr, PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wake up
    ret = pca9685_write_register_addr(addr, PCA9685_MODE1, 0x20);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Final mode: enable auto increment
    ret = pca9685_write_register_addr(addr, PCA9685_MODE1, 0xA0);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Turn off all outputs
    for (uint8_t i = 0; i < 16; i++) {
        pca9685_set_pwm_addr(addr, i, 0);
    }
    
    ESP_LOGI(TAG, "%s (0x%02x) initialized successfully", name, addr);
    return ESP_OK;
}

esp_err_t i2c_motor_driver_init(i2c_port_t i2c_port, int sda_pin, int scl_pin, uint32_t clock_speed)
{
    esp_err_t ret;
    
    // Use the shared I2C port (initialized by sensor_manager)
    i2c_motor_port = I2C_MASTER_NUM;
    
    // Note: I2C bus is now initialized by sensor_manager_init() in main.c
    // (GPIO 42 SDA, GPIO 41 SCL, 400kHz)
    
    // Initialize pump motor shield (0x60) - required
    ret = pca9685_init_shield(MOTOR_SHIELD_PUMP_ADDR, "Pump motor shield");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize pump motor shield - this is required!");
        return ret;
    }
    
    // Initialize LED motor shield (0x61) - optional
    ret = pca9685_init_shield(MOTOR_SHIELD_LED_ADDR, "LED motor shield");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED motor shield not found or failed to initialize (this is optional)");
        // Continue anyway - LED shield is optional
    }
    
    // Load motor direction settings from NVS
    esp_err_t load_err = i2c_motor_load_direction_settings();
    if (load_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load motor direction settings, using defaults");
    }
    
    ESP_LOGI(TAG, "I2C motor driver initialization complete");
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
    
    // Apply direction inversion if configured
    uint8_t actual_command = command;
    bool inverted = false;
    
    switch (motor_num) {
        case 1: inverted = s_motor_direction_config.motor1_inverted; break;
        case 2: inverted = s_motor_direction_config.motor2_inverted; break;
        case 3: inverted = s_motor_direction_config.motor3_inverted; break;
        case 4: inverted = s_motor_direction_config.motor4_inverted; break;
    }
    
    if (inverted && (command == MOTOR_FORWARD || command == MOTOR_BACKWARD)) {
        actual_command = (command == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
        ESP_LOGI(TAG, "Motor %d direction inverted: %d -> %d", motor_num, command, actual_command);
    }
    
    switch (actual_command) {
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

esp_err_t i2c_led_set_channel_brightness(uint8_t channel, uint8_t brightness)
{
    // Validate channel number (1-4)
    if (channel < 1 || channel > 4) {
        ESP_LOGE(TAG, "Invalid LED channel %u. Must be 1-4", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (brightness > 100) {
        brightness = 100;
    }
    
    ESP_LOGI(TAG, "Setting LED channel %u brightness to %u%% on LED shield (0x%02x)", 
             channel, brightness, MOTOR_SHIELD_LED_ADDR);
    
    // Get the PWM pins for the specified channel
    const motor_pins_t *pins = &motor_pins[channel - 1];
    
    // Convert percentage to 12-bit PWM value (0-4095)
    uint16_t pwm_value = (brightness * 4095) / 100;
    
    esp_err_t ret;
    if (brightness == 0) {
        // Turn off LED - release all control pins
        ret = pca9685_set_pin_addr(MOTOR_SHIELD_LED_ADDR, pins->in1_pin, false);
        ret |= pca9685_set_pin_addr(MOTOR_SHIELD_LED_ADDR, pins->in2_pin, false);
        ret |= pca9685_set_pwm_addr(MOTOR_SHIELD_LED_ADDR, pins->pwm_pin, 0);
    } else {
        // Set LED brightness using forward direction
        ret = pca9685_set_pin_addr(MOTOR_SHIELD_LED_ADDR, pins->in2_pin, false);
        ret |= pca9685_set_pin_addr(MOTOR_SHIELD_LED_ADDR, pins->in1_pin, true);
        ret |= pca9685_set_pwm_addr(MOTOR_SHIELD_LED_ADDR, pins->pwm_pin, pwm_value);
    }
    
    return ret;
}

esp_err_t i2c_food_pump_dose_ms(uint32_t duration_ms, uint8_t speed)
{
    if (speed > 100) {
        speed = 100;
    }
    
    if (duration_ms == 0) {
        ESP_LOGW(TAG, "Food pump dose duration is 0ms, ignoring");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Dosing food pump: %lu ms at %u%% speed", duration_ms, speed);
    
    // Start the food pump
    esp_err_t ret = i2c_motor_set(MOTOR_FOOD_PUMP, speed, MOTOR_FORWARD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start food pump: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for the specified duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    // Stop the food pump
    ret = i2c_motor_set(MOTOR_FOOD_PUMP, 0, MOTOR_RELEASE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop food pump: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Food pump dosing completed");
    return ESP_OK;
}

esp_err_t i2c_food_pump_start(uint8_t speed)
{
    if (speed > 100) {
        speed = 100;
    }
    
    ESP_LOGI(TAG, "Starting food pump at %u%% speed", speed);
    
    if (speed == 0) {
        return i2c_motor_set(MOTOR_FOOD_PUMP, 0, MOTOR_RELEASE);
    } else {
        return i2c_motor_set(MOTOR_FOOD_PUMP, speed, MOTOR_FORWARD);
    }
}

esp_err_t i2c_food_pump_stop(void)
{
    ESP_LOGI(TAG, "Stopping food pump");
    return i2c_motor_set(MOTOR_FOOD_PUMP, 0, MOTOR_RELEASE);
}

esp_err_t i2c_planter_pump_sweep_ms(uint32_t duration_ms, uint8_t min_speed, uint8_t max_speed, uint32_t sweep_period_ms)
{
    // Validate parameters
    if (min_speed > 100) min_speed = 100;
    if (max_speed > 100) max_speed = 100;
    if (min_speed > max_speed) {
        ESP_LOGE(TAG, "min_speed (%u) must be <= max_speed (%u)", min_speed, max_speed);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (duration_ms == 0) {
        ESP_LOGW(TAG, "Planter pump sweep duration is 0ms, ignoring");
        return ESP_OK;
    }
    
    if (sweep_period_ms < 100) {
        ESP_LOGW(TAG, "Sweep period too short (%lu ms), setting to 100ms minimum", sweep_period_ms);
        sweep_period_ms = 100;
    }
    
    ESP_LOGI(TAG, "Sweeping planter pump: %lu ms, PWM range %u-%u%%, sweep period %lu ms", 
             duration_ms, min_speed, max_speed, sweep_period_ms);
    
    // Calculate sweep parameters
    uint8_t speed_range = max_speed - min_speed;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;
    
    // Number of steps for smooth sweep (aim for ~50ms updates)
    const uint32_t update_interval_ms = 50;
    uint32_t steps_per_half_cycle = (sweep_period_ms / 2) / update_interval_ms;
    if (steps_per_half_cycle < 2) steps_per_half_cycle = 2; // Minimum 2 steps
    
    esp_err_t ret = ESP_OK;
    
    // Sweep loop
    while (elapsed < duration_ms) {
        // Calculate position in current sweep cycle (0.0 to 1.0)
        uint32_t cycle_position = elapsed % sweep_period_ms;
        float normalized_position;
        
        // Create triangle wave (0->1->0)
        if (cycle_position < sweep_period_ms / 2) {
            // Rising edge: 0 to 1
            normalized_position = (float)cycle_position / (float)(sweep_period_ms / 2);
        } else {
            // Falling edge: 1 to 0
            normalized_position = 1.0f - ((float)(cycle_position - sweep_period_ms / 2) / (float)(sweep_period_ms / 2));
        }
        
        // Calculate current speed
        uint8_t current_speed = min_speed + (uint8_t)(speed_range * normalized_position);
        
        // Set the motor speed
        ret = i2c_motor_set(MOTOR_PLANTER_PUMP, current_speed, MOTOR_FORWARD);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set planter pump speed: %s", esp_err_to_name(ret));
            // Try to stop the motor before returning
            i2c_motor_set(MOTOR_PLANTER_PUMP, 0, MOTOR_RELEASE);
            return ret;
        }
        
        // Wait for next update
        vTaskDelay(pdMS_TO_TICKS(update_interval_ms));
        
        // Update elapsed time
        elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
    }
    
    // Stop the planter pump
    ret = i2c_motor_set(MOTOR_PLANTER_PUMP, 0, MOTOR_RELEASE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop planter pump: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Planter pump sweep completed");
    return ESP_OK;
}

esp_err_t i2c_motor_get_direction_config(motor_direction_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    *config = s_motor_direction_config;
    return ESP_OK;
}

esp_err_t i2c_motor_load_direction_settings(void)
{
    esp_err_t err = config_load_motor_directions(
        &s_motor_direction_config.motor1_inverted,
        &s_motor_direction_config.motor2_inverted,
        &s_motor_direction_config.motor3_inverted,
        &s_motor_direction_config.motor4_inverted
    );
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Motor direction settings loaded from JSON");
        ESP_LOGI(TAG, "  Motor 1 (Planter): %s", s_motor_direction_config.motor1_inverted ? "inverted" : "normal");
        ESP_LOGI(TAG, "  Motor 2 (Food):    %s", s_motor_direction_config.motor2_inverted ? "inverted" : "normal");
        ESP_LOGI(TAG, "  Motor 3 (Source):  %s", s_motor_direction_config.motor3_inverted ? "inverted" : "normal");
        ESP_LOGI(TAG, "  Motor 4 (LED):     %s", s_motor_direction_config.motor4_inverted ? "inverted" : "normal");
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Motor direction settings not found, using defaults");
        err = ESP_OK; // Not an error, just use defaults
    } else {
        ESP_LOGE(TAG, "Failed to load motor direction settings: %s", esp_err_to_name(err));
    }
    
    return err;
}

esp_err_t i2c_motor_set_direction_invert(uint8_t motor_num, bool inverted)
{
    if (motor_num < 1 || motor_num > 4) {
        ESP_LOGE(TAG, "Invalid motor number: %d", motor_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (motor_num) {
        case 1: s_motor_direction_config.motor1_inverted = inverted; break;
        case 2: s_motor_direction_config.motor2_inverted = inverted; break;
        case 3: s_motor_direction_config.motor3_inverted = inverted; break;
        case 4: s_motor_direction_config.motor4_inverted = inverted; break;
    }
    
    ESP_LOGI(TAG, "Motor %d direction inversion set to: %s", motor_num, inverted ? "inverted" : "normal");
    return ESP_OK;
}

esp_err_t i2c_motor_get_direction_invert(uint8_t motor_num, bool *inverted)
{
    if (motor_num < 1 || motor_num > 4 || inverted == NULL) {
        ESP_LOGE(TAG, "Invalid motor number: %d or null pointer", motor_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (motor_num) {
        case 1: *inverted = s_motor_direction_config.motor1_inverted; break;
        case 2: *inverted = s_motor_direction_config.motor2_inverted; break;
        case 3: *inverted = s_motor_direction_config.motor3_inverted; break;
        case 4: *inverted = s_motor_direction_config.motor4_inverted; break;
    }
    
    return ESP_OK;
}

esp_err_t i2c_motor_set_direction_config(const motor_direction_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Null config pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    s_motor_direction_config = *config;
    
    ESP_LOGI(TAG, "Motor direction config updated: M1=%s, M2=%s, M3=%s, M4=%s",
        config->motor1_inverted ? "inverted" : "normal",
        config->motor2_inverted ? "inverted" : "normal", 
        config->motor3_inverted ? "inverted" : "normal",
        config->motor4_inverted ? "inverted" : "normal");
    
    return ESP_OK;
}

esp_err_t i2c_motor_save_direction_settings(void)
{
    esp_err_t err = config_save_motor_directions(
        s_motor_direction_config.motor1_inverted,
        s_motor_direction_config.motor2_inverted,
        s_motor_direction_config.motor3_inverted,
        s_motor_direction_config.motor4_inverted
    );
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Motor direction config saved: M1=%s, M2=%s, M3=%s, M4=%s",
            s_motor_direction_config.motor1_inverted ? "inverted" : "normal",
            s_motor_direction_config.motor2_inverted ? "inverted" : "normal",
            s_motor_direction_config.motor3_inverted ? "inverted" : "normal",
            s_motor_direction_config.motor4_inverted ? "inverted" : "normal");
    } else {
        ESP_LOGE(TAG, "Failed to save motor direction config: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t i2c_motor_pwm_sweep(uint8_t shield_addr)
{
    const char *shield_name;
    if (shield_addr == MOTOR_SHIELD_PUMP_ADDR) {
        shield_name = "Pump motor shield";
    } else if (shield_addr == MOTOR_SHIELD_LED_ADDR) {
        shield_name = "LED motor shield";
    } else {
        ESP_LOGE(TAG, "Invalid shield address: 0x%02x", shield_addr);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting PWM sweep on %s (0x%02x)", shield_name, shield_addr);
    
    // Sweep through all 4 motor channels
    for (uint8_t motor = 0; motor < 4; motor++) {
        const motor_pins_t *pins = &motor_pins[motor];
        
        ESP_LOGI(TAG, "  Channel %d: Sweeping 0%% -> 100%% -> 0%%", motor + 1);
        
        // Set motor to forward direction
        esp_err_t ret = pca9685_set_pin_addr(shield_addr, pins->in2_pin, false);
        ret |= pca9685_set_pin_addr(shield_addr, pins->in1_pin, true);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set direction for channel %d", motor + 1);
            continue;
        }
        
        // Sweep up from 0% to 100% over 1 second
        for (uint8_t pwm = 0; pwm <= 100; pwm += 5) {
            uint16_t pwm_value = (pwm * 4095) / 100;
            ret = pca9685_set_pwm_addr(shield_addr, pins->pwm_pin, pwm_value);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set PWM for channel %d at %d%%", motor + 1, pwm);
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms per step, 20 steps = 1 second
        }
        
        // Sweep down from 100% to 0% over 1 second
        for (int8_t pwm = 100; pwm >= 0; pwm -= 5) {
            uint16_t pwm_value = (pwm * 4095) / 100;
            ret = pca9685_set_pwm_addr(shield_addr, pins->pwm_pin, pwm_value);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set PWM for channel %d at %d%%", motor + 1, pwm);
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms per step, 20 steps = 1 second
        }
        
        // Release motor (turn off)
        ret = pca9685_set_pin_addr(shield_addr, pins->in1_pin, false);
        ret |= pca9685_set_pin_addr(shield_addr, pins->in2_pin, false);
        ret |= pca9685_set_pwm_addr(shield_addr, pins->pwm_pin, 0);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to release channel %d", motor + 1);
        }
        
        ESP_LOGI(TAG, "  Channel %d: Sweep complete", motor + 1);
    }
    
    ESP_LOGI(TAG, "PWM sweep complete on %s", shield_name);
    return ESP_OK;
}
