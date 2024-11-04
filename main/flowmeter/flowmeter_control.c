// flowmeter/flowmeter_control.c

#include "flowmeter_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_timer.h" // For esp_timer_get_time()

static const char *TAG = "FLOWMETER";

// Calibration constants
#define PULSES_PER_LITER 450.0f // Based on flowmeter specifications

// Flowmeter IDs
#define FLOWMETER_ID_DRAIN    0
#define FLOWMETER_ID_SOURCE   1
#define FLOWMETER_ID_OVERFLOW 2
#define FLOWMETER_COUNT       3

// Flowmeter event structure
typedef struct {
    int flowmeter_id; // Flowmeter ID
    int64_t timestamp; // Timestamp of the pulse
} flowmeter_event_t;

// Flowmeter data structure
typedef struct {
    int flowmeter_id;
    gpio_num_t gpio_num;
    uint32_t pulse_count;
    int64_t flow_start_time;
    int64_t last_calc_time;
    int64_t last_pulse_time;
    bool flow_active;
} flowmeter_t;

// Array to hold flowmeter data
static flowmeter_t flowmeters[FLOWMETER_COUNT] = {
    [FLOWMETER_ID_DRAIN] = {
        .flowmeter_id = FLOWMETER_ID_DRAIN,
        .gpio_num = FLOWMETER_GPIO_DRAIN,
    },
    [FLOWMETER_ID_SOURCE] = {
        .flowmeter_id = FLOWMETER_ID_SOURCE,
        .gpio_num = FLOWMETER_GPIO_SOURCE,
    },
    [FLOWMETER_ID_OVERFLOW] = {
        .flowmeter_id = FLOWMETER_ID_OVERFLOW,
        .gpio_num = FLOWMETER_GPIO_OVERFLOW,
    },
};

// Event queue for flowmeter events
static QueueHandle_t flowmeter_evt_queue = NULL;

// Forward declaration of the flowmeter task
void flowmeter_task(void *pvParameter);

// GPIO ISR handler
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    int flowmeter_id = (int)(uintptr_t)arg;
    flowmeter_event_t evt = {
        .flowmeter_id = flowmeter_id,
        .timestamp = esp_timer_get_time(),
    };
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(flowmeter_evt_queue, &evt, &high_task_wakeup);

    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t flowmeter_init(void) {
    esp_err_t ret = ESP_OK;

    // Create the event queue
    flowmeter_evt_queue = xQueueCreate(20, sizeof(flowmeter_event_t));
    if (flowmeter_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_FAIL;
    }

    // Install GPIO ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    for (int i = 0; i < FLOWMETER_COUNT; i++) {
        flowmeter_t *fm = &flowmeters[i];

        // Configure the GPIO pin with internal pull-up resistor
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << fm->gpio_num),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE, // Trigger on falling edge
        };
        gpio_config(&io_conf);

        // Add ISR handler
        ret = gpio_isr_handler_add(fm->gpio_num, gpio_isr_handler, (void *)(uintptr_t)fm->flowmeter_id);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler for flowmeter %d", fm->flowmeter_id);
            return ret;
        }
    }

    // Create the flowmeter task
    xTaskCreate(flowmeter_task, "flowmeter_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Flowmeters initialized using GPIO interrupts");
    return ESP_OK;
}

void flowmeter_task(void *pvParameter) {
    const int64_t FLOW_STOPPED_TIMEOUT_US = 2000000; // 2 seconds
    const int64_t FLOW_CALC_INTERVAL_US = 1000000;   // 1 second

    while (1) {
        flowmeter_event_t evt;
        // Wait for an event from the queue with timeout
        if (xQueueReceive(flowmeter_evt_queue, &evt, pdMS_TO_TICKS(1000))) {
            // Event received
            int flowmeter_id = evt.flowmeter_id;
            flowmeter_t *fm = &flowmeters[flowmeter_id];
            int64_t current_time = evt.timestamp; // Use the timestamp from the ISR

            // Pulse detected
            fm->pulse_count++;

            // Update last pulse time
            fm->last_pulse_time = current_time;

            if (!fm->flow_active) {
                // Flow just started
                fm->flow_active = true;
                fm->flow_start_time = current_time;
                fm->last_calc_time = current_time;
                fm->pulse_count = 1; // Reset pulse count to 1 for the first pulse
                ESP_LOGI(TAG, "Flowmeter %d: Flow started at time %lld us", flowmeter_id, fm->flow_start_time);
            }

            // Calculate flow rate every second
            if (current_time - fm->last_calc_time >= FLOW_CALC_INTERVAL_US) {
                // Calculate flow rate
                float flow_rate = (fm->pulse_count / PULSES_PER_LITER) * 60.0f; // L/min

                // Output flow rate with timestamp
                ESP_LOGI(TAG, "Flowmeter %d: Time: %.2f s, Flow Rate: %.2f L/min",
                         flowmeter_id, (current_time - fm->flow_start_time) / 1000000.0f, flow_rate);

                // Reset pulse count and last_calc_time
                fm->pulse_count = 0;
                fm->last_calc_time = current_time;
            }
        } else {
            // Timeout occurred (no event received within 1000 ms)
            int64_t current_time = esp_timer_get_time(); // Current time in microseconds

            for (int i = 0; i < FLOWMETER_COUNT; i++) {
                flowmeter_t *fm = &flowmeters[i];

                if (fm->flow_active) {
                    // Check if flow has stopped
                    if (current_time - fm->last_pulse_time >= FLOW_STOPPED_TIMEOUT_US) {
                        fm->flow_active = false;
                        ESP_LOGI(TAG, "Flowmeter %d: Flow stopped at time %lld us", fm->flowmeter_id, current_time);
                    } else if (current_time - fm->last_calc_time >= FLOW_CALC_INTERVAL_US) {
                        // Flow is still considered active, but no pulses in the last interval
                        // Output zero flow rate
                        ESP_LOGI(TAG, "Flowmeter %d: Time: %.2f s, Flow Rate: 0.00 L/min",
                                 fm->flowmeter_id, (current_time - fm->flow_start_time) / 1000000.0f);

                        // Reset pulse count and last_calc_time
                        fm->pulse_count = 0;
                        fm->last_calc_time = current_time;
                    }
                }
            }
        }
    }
}
