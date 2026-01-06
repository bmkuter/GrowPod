#ifndef I2C_MOTOR_DRIVER_H
#define I2C_MOTOR_DRIVER_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdbool.h>
#include "ina219.h"  // For shared I2C definitions

// I2C addresses for motor shields
#define MOTOR_SHIELD_PUMP_ADDR         0x60  // Pump control motor shield
#define MOTOR_SHIELD_LED_ADDR          0x61  // LED control motor shield

// For backward compatibility
#define MOTOR_SHIELD_I2C_ADDR          MOTOR_SHIELD_PUMP_ADDR

// Define DC motor channels for each pump (on 0x60 shield)
#define MOTOR_PLANTER_PUMP             1  // DC Motor #1 - Planter pump
#define MOTOR_FOOD_PUMP                2  // DC Motor #2 - Food dosing pump
#define MOTOR_SOURCE_PUMP              3  // DC Motor #3 - Source pump
#define MOTOR_DRAIN_PUMP               4  // DC Motor #4 - Drain pump

// Define LED channels (on 0x61 shield)
#define LED_CHANNEL_1                  1  // LED array on motor channel 1
#define LED_CHANNEL_2                  2  // LED array on motor channel 2
#define LED_CHANNEL_3                  3  // LED array on motor channel 3
#define LED_CHANNEL_4                  4  // LED array on motor channel 4

// Motor commands
#define MOTOR_FORWARD                  1
#define MOTOR_BACKWARD                 2
#define MOTOR_RELEASE                  4  // Stop/brake the motor

// Motor direction inversion settings (for pump shield at 0x60)
typedef struct {
    bool motor1_inverted;  // Planter pump
    bool motor2_inverted;  // Food pump
    bool motor3_inverted;  // Source pump
    bool motor4_inverted;  // Drain pump
} motor_direction_config_t;

/**
 * @brief Initialize the I2C motor driver
 * 
 * This function reuses the I2C driver from power_monitor_HAL.
 * The parameters are kept for compatibility but internally we use
 * the shared I2C bus configuration from the power monitor.
 * 
 * @param i2c_port I2C port number (ignored, uses I2C_MASTER_NUM)
 * @param sda_pin SDA pin number (ignored, uses I2C_MASTER_SDA_IO)
 * @param scl_pin SCL pin number (ignored, uses I2C_MASTER_SCL_IO)
 * @param clock_speed I2C clock speed in Hz (ignored, uses I2C_MASTER_FREQ_HZ)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_driver_init(i2c_port_t i2c_port, int sda_pin, int scl_pin, uint32_t clock_speed);

/**
 * @brief Set the speed of a DC motor (0-100%)
 * 
 * @param motor_num Motor number (1-4)
 * @param speed Speed percentage (0-100)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_set_speed(uint8_t motor_num, uint8_t speed);

/**
 * @brief Run a DC motor in the specified direction
 * 
 * @param motor_num Motor number (1-4)
 * @param command MOTOR_FORWARD, MOTOR_BACKWARD, or MOTOR_RELEASE
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_run(uint8_t motor_num, uint8_t command);

/**
 * @brief Set speed and direction of a DC motor in one call
 * 
 * @param motor_num Motor number (1-4)
 * @param speed Speed percentage (0-100)
 * @param command MOTOR_FORWARD, MOTOR_BACKWARD, or MOTOR_RELEASE
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_set(uint8_t motor_num, uint8_t speed, uint8_t command);

/**
 * @brief Stop all motors
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_stop_all(void);

/**
 * @brief Set the brightness of a specific LED channel using PWM
 * 
 * Controls LED on the specified channel (1-4) of the LED motor shield (address 0x61).
 * 
 * @param channel LED channel number (1-4)
 * @param brightness Brightness level (0-100%)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_led_set_channel_brightness(uint8_t channel, uint8_t brightness);

/**
 * @brief Set motor direction inversion for a specific motor
 * 
 * @param motor_num Motor number (1-4)
 * @param inverted true to invert forward/backward, false for normal operation
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_set_direction_invert(uint8_t motor_num, bool inverted);

/**
 * @brief Get motor direction inversion setting for a specific motor
 * 
 * @param motor_num Motor number (1-4)
 * @param inverted Pointer to store the inversion setting
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_get_direction_invert(uint8_t motor_num, bool *inverted);

/**
 * @brief Set motor direction configuration for all motors
 * 
 * @param config Motor direction configuration structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_set_direction_config(const motor_direction_config_t *config);

/**
 * @brief Get motor direction configuration for all motors
 * 
 * @param config Pointer to store motor direction configuration
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_get_direction_config(motor_direction_config_t *config);

/**
 * @brief Load motor direction settings from NVS
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_load_direction_settings(void);

/**
 * @brief Save motor direction settings to NVS
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_save_direction_settings(void);

/**
 * @brief Dose food pump for a specific duration in milliseconds
 * 
 * @param duration_ms Duration to run the food pump in milliseconds
 * @param speed Speed percentage (0-100%, default recommended: 80-100%)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_food_pump_dose_ms(uint32_t duration_ms, uint8_t speed);

/**
 * @brief Start food pump at specified speed (continuous operation)
 * 
 * @param speed Speed percentage (0-100%)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_food_pump_start(uint8_t speed);

/**
 * @brief Stop food pump
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_food_pump_stop(void);

/**
 * @brief Run planter pump with PWM sweep for a specific duration
 * 
 * Sweeps PWM between min_speed and max_speed to create a sprinkler effect.
 * Continuously sweeps back and forth during the entire duration.
 * 
 * @param duration_ms Duration to run the planter pump in milliseconds
 * @param min_speed Minimum speed percentage (default: 65%)
 * @param max_speed Maximum speed percentage (default: 100%)
 * @param sweep_period_ms Time for one complete sweep cycle (up and down) in milliseconds (default: 2000ms)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_planter_pump_sweep_ms(uint32_t duration_ms, uint8_t min_speed, uint8_t max_speed, uint32_t sweep_period_ms);

/**
 * @brief Perform a PWM sweep on all 4 channels of a motor shield
 * 
 * Sweeps each motor channel from 0% to 100% and back to 0% over 2 seconds.
 * Tests all 4 motor channels sequentially.
 * 
 * @param shield_addr Motor shield I2C address (0x60 for pumps, 0x61 for LEDs)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_motor_pwm_sweep(uint8_t shield_addr);

#endif // I2C_MOTOR_DRIVER_H
