#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>

// -------------------------------------------------------------
// GPIO definitions
// -------------------------------------------------------------
#define AIR_PUMP_GPIO       GPIO_NUM_0
#define SOURCE_PUMP_GPIO    GPIO_NUM_1
#define DRAIN_PUMP_GPIO     GPIO_NUM_6
#define LED_ARRAY_GPIO      GPIO_NUM_3
#define PLANTER_PUMP_GPIO   GPIO_NUM_7

// -------------------------------------------------------------
// PWM channel definitions (all at 25 kHz, 8-bit resolution)
// -------------------------------------------------------------
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE

#define AIR_PUMP_CHANNEL       LEDC_CHANNEL_0
#define SOURCE_PUMP_CHANNEL    LEDC_CHANNEL_1
#define DRAIN_PUMP_CHANNEL     LEDC_CHANNEL_2
#define PLANTER_PUMP_CHANNEL   LEDC_CHANNEL_3
#define LED_ARRAY_CHANNEL      LEDC_CHANNEL_4

// -------------------------------------------------------------
// Command types for the queue
// -------------------------------------------------------------
typedef enum {
    ACTUATOR_CMD_AIR_PUMP_PWM,     // value = 0..100
    ACTUATOR_CMD_SOURCE_PUMP_PWM,  // value = 0..100
    ACTUATOR_CMD_DRAIN_PUMP_PWM,   // value = 0..100
    ACTUATOR_CMD_PLANTER_PUMP_PWM, // value = 0..100
    ACTUATOR_CMD_LED_ARRAY_PWM     // value = 0..100
} actuator_cmd_t;

// The queue command structure
typedef struct {
    actuator_cmd_t cmd_type;
    uint32_t       value;  // duty% for pumps (0..100), or boolean for LED
} actuator_command_t;

// -------------------------------------------------------------
// Extended "actuator info" tracking
// -------------------------------------------------------------

// Identify each actuator with an index in a global array
typedef enum {
    ACTUATOR_IDX_AIR_PUMP = 0,
    ACTUATOR_IDX_SOURCE_PUMP,
    ACTUATOR_IDX_DRAIN_PUMP,
    ACTUATOR_IDX_PLANTER_PUMP,
    ACTUATOR_IDX_LED_ARRAY,
    ACTUATOR_IDX_MAX
} actuator_index_t;

// Track each actuator’s state, including estimated power usage
typedef struct {
    bool   is_on;                 // true if duty>0 (or LED on)
    float  duty_percentage;       // 0..100 for pumps, 0 or 100 for LED
    float  estimated_power_mW;    // rough estimate of power usage
} actuator_info_t;

// -------------------------------------------------------------
// Public APIs
// -------------------------------------------------------------
void            actuator_control_init(void);
void            actuator_control_task(void *pvParameters);
QueueHandle_t   get_actuator_queue(void);

// Pump control (all 8-bit duty 0..255 equivalent)
void set_air_pump_pwm(uint32_t duty_percentage);
void set_source_pump_pwm(uint32_t duty_percentage);
void set_drain_pump_pwm(uint32_t duty_percentage);
void set_planter_pump_pwm(uint32_t duty_percentage);

// LED array
void set_led_array_binary(bool state);
void set_led_array_pwm(uint32_t duty_percentage);

// Access to the global actuator info array
// (Note: consider adding a mutex if multiple tasks can read/write)
const actuator_info_t* actuator_control_get_info(void);
#endif // ACTUATOR_CONTROL_H
