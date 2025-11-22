#ifndef I2C_MOTOR_DRIVER_H
#define I2C_MOTOR_DRIVER_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdbool.h>
#include "ina219.h"  // For shared I2C definitions

// Default I2C address for the motor shield
#define MOTOR_SHIELD_I2C_ADDR          0x60

// Define DC motor channels for each pump
#define MOTOR_PLANTER_PUMP              1  // DC Motor #1
#define MOTOR_FOOD_PUMP                2  // DC Motor #2 (food dosing pump)
#define MOTOR_SOURCE_PUMP              3  // DC Motor #3
#define LED_CONTROL                    4  // LED connected to motor channel 4

// Future expansion - drain pump will use a different channel when added
#define MOTOR_DRAIN_PUMP               5  // DC Motor #5 (future expansion)

// Motor commands
#define MOTOR_FORWARD                  1
#define MOTOR_BACKWARD                 2
#define MOTOR_RELEASE                  4  // Stop/brake the motor

// Motor direction inversion settings
typedef struct {
    bool motor1_inverted;  // Planter pump
    bool motor2_inverted;  // Food pump
    bool motor3_inverted;  // Source pump
    bool motor4_inverted;  // LED control (not typically used for direction)
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
 * @brief Set the LED brightness using PWM
 * 
 * @param brightness Brightness level (0-100%)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t i2c_led_set_brightness(uint8_t brightness);

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

#endif // I2C_MOTOR_DRIVER_H
