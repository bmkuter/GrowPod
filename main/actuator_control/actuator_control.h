#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include "i2c_motor_driver.h"
#include "power_monitor_HAL.h"  // For shared I2C definitions

// -------------------------------------------------------------
// Note: We're using the I2C definitions from power_monitor_HAL.h:
// I2C_MASTER_NUM (I2C_NUM_0)
// I2C_MASTER_SDA_IO (GPIO_NUM_42)
// I2C_MASTER_SCL_IO (GPIO_NUM_41)
// I2C_MASTER_FREQ_HZ (100000)
// -------------------------------------------------------------

// -------------------------------------------------------------
// LED Array GPIO definition (for direct GPIO control if needed)
// -------------------------------------------------------------
#define LED_ARRAY_GPIO      GPIO_NUM_10

// -------------------------------------------------------------
// Command types for the queue
// -------------------------------------------------------------
typedef enum {
    ACTUATOR_CMD_AIR_PUMP_PWM,     // value = 0..100 (NOTE: Air pump has been replaced by LED control)
    ACTUATOR_CMD_SOURCE_PUMP_PWM,  // value = 0..100
    ACTUATOR_CMD_DRAIN_PUMP_PWM,   // value = 0..100 (future expansion)
    ACTUATOR_CMD_PLANTER_PUMP_PWM, // value = 0..100
    ACTUATOR_CMD_FOOD_PUMP_PWM,    // value = 0..100
    ACTUATOR_CMD_LED_ARRAY_PWM,    // value = 0..100 (Now using I2C motor driver on channel 4)
    ACTUATOR_CMD_LED_CHANNEL_PWM   // Multi-channel LED control (channel in channel field, value = 0..100)
} actuator_cmd_t;

// The queue command structure
typedef struct {
    actuator_cmd_t cmd_type;
    uint32_t       value;   // duty% for pumps (0..100), or boolean for LED
    uint8_t        channel; // LED channel (1-4) for ACTUATOR_CMD_LED_CHANNEL_PWM
} actuator_command_t;

// -------------------------------------------------------------
// Extended "actuator info" tracking
// -------------------------------------------------------------

// Identify each actuator with an index in a global array
typedef enum {
    ACTUATOR_IDX_AIR_PUMP = 0,    // Note: Air pump has been replaced by LED control
    ACTUATOR_IDX_SOURCE_PUMP,
    ACTUATOR_IDX_DRAIN_PUMP,      // Future expansion - not currently connected
    ACTUATOR_IDX_PLANTER_PUMP,
    ACTUATOR_IDX_FOOD_PUMP,       // New food pump using motor channel 2
    ACTUATOR_IDX_LED_ARRAY,       // Now using I2C motor driver on channel 4
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
void set_drain_pump_pwm(uint32_t duty_percentage);  // Future expansion
void set_planter_pump_pwm(uint32_t duty_percentage);
void set_food_pump_pwm(uint32_t duty_percentage);

// Food pump dosing functions
void dose_food_pump_ms(uint32_t duration_ms, uint8_t speed);

// LED array
void set_led_array_binary(bool state);
void set_led_array_pwm(uint32_t duty_percentage);

// Multi-channel LED control (channels 1-4 on LED shield at 0x61)
void set_led_channel_pwm(uint8_t channel, uint32_t duty_percentage);

// Access to the global actuator info array
// (Note: consider adding a mutex if multiple tasks can read/write)
const actuator_info_t* actuator_control_get_info(void);
#endif // ACTUATOR_CONTROL_H
