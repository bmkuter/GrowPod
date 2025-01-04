#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// GPIO definitions for existing peripherals
#define AIR_PUMP_GPIO       GPIO_NUM_0
#define WATER_PUMP_GPIO     GPIO_NUM_1
#define SERVO_GPIO          GPIO_NUM_2  // Re-use former SOLENOID_VALVE_GPIO
#define LED_ARRAY_GPIO      GPIO_NUM_3

// PWM channel configurations (for pumps)
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE

#define AIR_PUMP_CHANNEL    LEDC_CHANNEL_0
#define WATER_PUMP_CHANNEL  LEDC_CHANNEL_1

// ----- NEW SERVO CONSTANTS -----
#define SERVO_TIMER         LEDC_TIMER_1       // Separate timer for 50 Hz
#define SERVO_CHANNEL       LEDC_CHANNEL_2     // Use a free channel
#define SERVO_SPEED_MODE    LEDC_LOW_SPEED_MODE // Keep low-speed mode

// Typical servo settings
#define SERVO_MIN_PULSEWIDTH_US  500   // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US  2500  // Maximum pulse width in microseconds
#define SERVO_MAX_DEGREE         180   // Maximum angle in degrees

// Command types
typedef enum {
    ACTUATOR_CMD_AIR_PUMP_PWM,
    ACTUATOR_CMD_WATER_PUMP_PWM,
    ACTUATOR_CMD_SERVO_ANGLE, 
    ACTUATOR_CMD_LED_ARRAY_BINARY
} actuator_cmd_t;

// Command structure
typedef struct {
    actuator_cmd_t cmd_type;
    uint32_t value; // duty or angle, depending on the command
} actuator_command_t;

// Function prototypes
void actuator_control_init(void);
void actuator_control_task(void *pvParameters);
QueueHandle_t get_actuator_queue(void);

// Existing functions for pumps
void set_air_pump_pwm(uint32_t duty);
void set_water_pump_pwm(uint32_t duty);

// Replace old solenoid function with servo
void set_servo_angle(uint32_t angle);

// LED array
void set_led_array_binary(bool state);

#endif // ACTUATOR_CONTROL_H
