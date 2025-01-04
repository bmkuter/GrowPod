// control_logic.h

#ifndef CONTROL_LOGIC_H
#define CONTROL_LOGIC_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "flowmeter_control.h"
#include "ina260.h"
#include "https_server.h"
#include "wifi_manager.h"
#include "uart_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "actuator_control.h" // Include your GPIO control header
#include "esp_log.h"

// System operational states
typedef enum {
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_FEEDING_CYCLE_ACTIVE,
    SYSTEM_STATE_EMPTYING_WATER,
    SYSTEM_STATE_OVERFLOW_DETECTED,
    SYSTEM_STATE_MAINTENANCE_REQUIRED,
    SYSTEM_STATE_ERROR
} system_state_t;

// Extern declaration of current system state
extern system_state_t current_system_state;

// Function prototypes
void set_drain_valve_angle(uint32_t angle_degrees);

// Initialize the control logic module
esp_err_t control_logic_init(void);

// Functions to start and stop feeding cycle
esp_err_t start_feeding_cycle(void);
esp_err_t stop_feeding_cycle(void);

// Functions to start and stop emptying water
esp_err_t start_emptying_water(void);
esp_err_t stop_emptying_water(void);

// Function to handle overflow event
esp_err_t handle_overflow_event(void);

// Function to get current system state
system_state_t get_system_state(void);

// Function to set system state (used internally)
void set_system_state(system_state_t state);

// Function to get system state as a string
const char *get_system_status_string(system_state_t state);

// Control logic task function
void control_logic_task(void *pvParameter);

#endif // CONTROL_LOGIC_H
