// control_logic.c

#include "control_logic.h"
#include "esp_log.h"
#include "flowmeter_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CONTROL_LOGIC";

// Current system state
system_state_t current_system_state = SYSTEM_STATE_IDLE;

// Task handle for control logic task
static TaskHandle_t control_logic_task_handle = NULL;

// Constants and thresholds
#define WATER_PUMP_FEEDING_PWM 80    // PWM value for water pump during feeding
#define OVERFLOW_THRESHOLD 1.0f      // Threshold flow rate for overflow detection (L/min)
#define SOME_MIN_FLOW_RATE 0.1f      // Minimum flow rate to consider as flow stopped

// Function implementations

esp_err_t control_logic_init(void) {
    esp_err_t ret = ESP_OK;

    // Initialize system state
    current_system_state = SYSTEM_STATE_IDLE;

    // Create control logic task
    if (control_logic_task_handle == NULL) {
        xTaskCreate(control_logic_task, "control_logic_task", 4096, NULL, 5, &control_logic_task_handle);
        if (control_logic_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create control logic task");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Control logic module initialized");
    return ret;
}

esp_err_t start_feeding_cycle(void) {
    if (current_system_state != SYSTEM_STATE_IDLE) {
        ESP_LOGW(TAG, "Cannot start feeding cycle: System is not in IDLE state");
        return ESP_ERR_INVALID_STATE;
    }

    // Update system state
    set_system_state(SYSTEM_STATE_FEEDING_CYCLE_ACTIVE);

    // Activate necessary actuators (e.g., water pump)
    // Set water pump PWM to desired value
    set_water_pump_pwm(WATER_PUMP_FEEDING_PWM);

    // Additional logic as needed
    ESP_LOGI(TAG, "Feeding cycle started");
    return ESP_OK;
}

esp_err_t stop_feeding_cycle(void) {
    if (current_system_state != SYSTEM_STATE_FEEDING_CYCLE_ACTIVE) {
        ESP_LOGW(TAG, "Cannot stop feeding cycle: Feeding cycle is not active");
        return ESP_ERR_INVALID_STATE;
    }

    // Deactivate necessary actuators
    set_water_pump_pwm(0); // Stop water pump

    // Update system state
    set_system_state(SYSTEM_STATE_IDLE);

    ESP_LOGI(TAG, "Feeding cycle stopped");
    return ESP_OK;
}

esp_err_t start_emptying_water(void) {
    if (current_system_state != SYSTEM_STATE_IDLE && current_system_state != SYSTEM_STATE_FEEDING_CYCLE_ACTIVE) {
        ESP_LOGW(TAG, "Cannot start emptying water: Invalid system state");
        return ESP_ERR_INVALID_STATE;
    }

    // Update system state
    set_system_state(SYSTEM_STATE_EMPTYING_WATER);

    // Open the drain solenoid valve
    set_drain_valve_state(true); // Open valve

    // Additional logic as needed
    ESP_LOGI(TAG, "Water emptying started");
    return ESP_OK;
}

esp_err_t stop_emptying_water(void) {
    if (current_system_state != SYSTEM_STATE_EMPTYING_WATER) {
        ESP_LOGW(TAG, "Cannot stop emptying water: Water emptying is not active");
        return ESP_ERR_INVALID_STATE;
    }

    // Close the drain solenoid valve
    set_drain_valve_state(false); // Close valve

    // Update system state
    set_system_state(SYSTEM_STATE_IDLE);

    ESP_LOGI(TAG, "Water emptying stopped");
    return ESP_OK;
}

esp_err_t handle_overflow_event(void) {
    if (current_system_state == SYSTEM_STATE_OVERFLOW_DETECTED) {
        // Overflow already handled
        return ESP_OK;
    }

    // Update system state
    set_system_state(SYSTEM_STATE_OVERFLOW_DETECTED);

    // Take necessary actions (e.g., stop pumps, open drain)
    set_water_pump_pwm(0);
    set_air_pump_pwm(0);
    set_drain_valve_state(true); // Open drain valve to prevent overflow

    // Notify user or system
    ESP_LOGE(TAG, "Overflow detected! System has been stopped for safety.");

    // Additional actions as needed
    return ESP_OK;
}

system_state_t get_system_state(void) {
    return current_system_state;
}

void set_system_state(system_state_t state) {
    current_system_state = state;
    ESP_LOGI(TAG, "System state updated to: %s", get_system_status_string(state));
}

const char *get_system_status_string(system_state_t state) {
    switch (state) {
        case SYSTEM_STATE_IDLE:
            return "Idle";
        case SYSTEM_STATE_FEEDING_CYCLE_ACTIVE:
            return "Feeding Cycle Active";
        case SYSTEM_STATE_EMPTYING_WATER:
            return "Emptying Water";
        case SYSTEM_STATE_OVERFLOW_DETECTED:
            return "Overflow Detected";
        case SYSTEM_STATE_MAINTENANCE_REQUIRED:
            return "Maintenance Required";
        case SYSTEM_STATE_ERROR:
            return "Error";
        default:
            return "Unknown";
    }
}

void control_logic_task(void *pvParameter) {
    while (1) {
        // Monitor overflow flowmeter
        float overflow_flow_rate = get_overflow_flow_rate();
        if (overflow_flow_rate > OVERFLOW_THRESHOLD) {
            handle_overflow_event();
        }

        // Monitor emptying water completion
        if (current_system_state == SYSTEM_STATE_EMPTYING_WATER) {
            // Check if drain flow rate has decreased below a threshold
            float drain_flow_rate = get_drain_flow_rate();
            if (drain_flow_rate < SOME_MIN_FLOW_RATE) {
                // Emptying is complete
                stop_emptying_water();
            }
        }

        // Add other monitoring and control logic here
        // For example, monitor feeding cycle completion

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}

// Stub functions for actuator control
// Implement these functions in your pwm_control or gpio_control module

// void set_water_pump_pwm(uint8_t pwm_value) {
//     // Set the PWM value for the water pump
//     ESP_LOGI(TAG, "Setting water pump PWM to %d", pwm_value);
//     // Implement actual PWM control here
//     // e.g., pwm_set_duty(WATER_PUMP_CHANNEL, pwm_value);
// }

// void set_air_pump_pwm(uint8_t pwm_value) {
//     // Set the PWM value for the air pump
//     ESP_LOGI(TAG, "Setting air pump PWM to %d", pwm_value);
//     // Implement actual PWM control here
//     // e.g., pwm_set_duty(AIR_PUMP_CHANNEL, pwm_value);
// }

void set_drain_valve_state(bool open) {
    // Control the drain solenoid valve
    ESP_LOGI(TAG, "Setting drain valve state to %s", open ? "OPEN" : "CLOSED");
    // Implement actual GPIO control here
    // e.g., gpio_set_level(DRAIN_VALVE_GPIO, open ? 1 : 0);
    set_solenoid_valve(open);
}
