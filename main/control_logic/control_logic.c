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

// -----------------------------------------------------------
// Constants and thresholds
// -----------------------------------------------------------
// If you want the drain "open" or "closed" to be just 0/100% PWM:
#define DRAIN_OPEN_PWM     100   // 100% duty to "open"
#define DRAIN_CLOSED_PWM   0     // 0% duty to "close"

// For source pump feeding
#define SOURCE_PUMP_FEEDING_PWM 80

#define OVERFLOW_THRESHOLD 1.0f      // Threshold flow rate for overflow detection (L/min)
#define SOME_MIN_FLOW_RATE 0.1f      // Minimum flow rate to consider as flow stopped

// -----------------------------------------------------------
// Initialization
// -----------------------------------------------------------
esp_err_t control_logic_init(void) {
    // Initialize system state
    current_system_state = SYSTEM_STATE_IDLE;

    // Create control logic task if not already created
    if (control_logic_task_handle == NULL) {
        xTaskCreate(control_logic_task, "control_logic_task", 4096, NULL, 5, &control_logic_task_handle);
        if (control_logic_task_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create control logic task");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Control logic module initialized");
    return ESP_OK;
}

// -----------------------------------------------------------
// Start / Stop Feeding Cycle
// -----------------------------------------------------------
esp_err_t start_feeding_cycle(void) {
    if (current_system_state != SYSTEM_STATE_IDLE) {
        ESP_LOGW(TAG, "Cannot start feeding cycle: System is not in IDLE state");
        return ESP_ERR_INVALID_STATE;
    }

    // Update system state
    set_system_state(SYSTEM_STATE_FEEDING_CYCLE_ACTIVE);

    // Activate the source pump (formerly water pump) at desired PWM
    set_source_pump_pwm(SOURCE_PUMP_FEEDING_PWM);

    ESP_LOGI(TAG, "Feeding cycle started");
    return ESP_OK;
}

esp_err_t stop_feeding_cycle(void) {
    if (current_system_state != SYSTEM_STATE_FEEDING_CYCLE_ACTIVE) {
        ESP_LOGW(TAG, "Cannot stop feeding cycle: It is not active");
        return ESP_ERR_INVALID_STATE;
    }

    // Deactivate the source pump
    set_source_pump_pwm(0);

    // Update system state
    set_system_state(SYSTEM_STATE_IDLE);

    ESP_LOGI(TAG, "Feeding cycle stopped");
    return ESP_OK;
}

// -----------------------------------------------------------
// Start / Stop Emptying Water
// -----------------------------------------------------------
esp_err_t start_emptying_water(void) {
    // Optionally allow emptying water from either IDLE or FEEDING_CYCLE_ACTIVE
    if (current_system_state != SYSTEM_STATE_IDLE &&
        current_system_state != SYSTEM_STATE_FEEDING_CYCLE_ACTIVE)
    {
        ESP_LOGW(TAG, "Cannot start emptying water: Invalid system state");
        return ESP_ERR_INVALID_STATE;
    }

    // Update system state
    set_system_state(SYSTEM_STATE_EMPTYING_WATER);

    // "Open" the drain pump (set to 100% PWM)
    set_drain_pump_pwm(DRAIN_OPEN_PWM);

    ESP_LOGI(TAG, "Water emptying started");
    return ESP_OK;
}

esp_err_t stop_emptying_water(void) {
    if (current_system_state != SYSTEM_STATE_EMPTYING_WATER) {
        ESP_LOGW(TAG, "Cannot stop emptying water: It's not active");
        return ESP_ERR_INVALID_STATE;
    }

    // "Close" the drain pump (set to 0% PWM)
    set_drain_pump_pwm(DRAIN_CLOSED_PWM);

    // Update system state
    set_system_state(SYSTEM_STATE_IDLE);

    ESP_LOGI(TAG, "Water emptying stopped");
    return ESP_OK;
}

// -----------------------------------------------------------
// Overflow Handling
// -----------------------------------------------------------
esp_err_t handle_overflow_event(void) {
    if (current_system_state == SYSTEM_STATE_OVERFLOW_DETECTED) {
        // Overflow already handled
        return ESP_OK;
    }

    // Update system state
    set_system_state(SYSTEM_STATE_OVERFLOW_DETECTED);

    // Stop relevant pumps to prevent further flooding
    set_source_pump_pwm(0);
    set_air_pump_pwm(0);

    // Open the drain pump fully to mitigate overflow
    set_drain_pump_pwm(DRAIN_OPEN_PWM);

    ESP_LOGE(TAG, "Overflow detected! System has been stopped for safety.");
    return ESP_OK;
}

// -----------------------------------------------------------
// System State Accessors
// -----------------------------------------------------------
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

// -----------------------------------------------------------
// Control Logic Task
// -----------------------------------------------------------
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

        // Add other monitoring/logic for the feeding cycle, etc.

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}
