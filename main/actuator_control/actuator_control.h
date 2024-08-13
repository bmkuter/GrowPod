#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// GPIO definitions
#define AIR_PUMP_GPIO       GPIO_NUM_0
#define WATER_PUMP_GPIO     GPIO_NUM_1
#define SOLENOID_VALVE_GPIO GPIO_NUM_2
#define LED_ARRAY_GPIO      GPIO_NUM_3

// PWM channel configurations (only for pumps)
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE

#define AIR_PUMP_CHANNEL    LEDC_CHANNEL_0
#define WATER_PUMP_CHANNEL  LEDC_CHANNEL_1

// Command types
typedef enum {
    ACTUATOR_CMD_AIR_PUMP_PWM,
    ACTUATOR_CMD_WATER_PUMP_PWM,
    ACTUATOR_CMD_SOLENOID_VALVE,
    ACTUATOR_CMD_LED_ARRAY_BINARY
} actuator_cmd_t;

// Command structure
typedef struct {
    actuator_cmd_t cmd_type;
    uint32_t value; // Can represent duty cycle or state (on/off)
} actuator_command_t;

// Function prototypes
void actuator_control_init(void);
void set_air_pump_pwm(uint32_t duty);
void set_water_pump_pwm(uint32_t duty);
void set_solenoid_valve(bool state);
void set_led_array_binary(bool state);
void actuator_control_task(void *pvParameters);
QueueHandle_t get_actuator_queue(void); // Function to get the queue handle

#endif // ACTUATOR_CONTROL_H
