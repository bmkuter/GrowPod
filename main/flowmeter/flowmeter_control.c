// flowmeter/flowmeter_control.c

#include "flowmeter_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>

static const char *TAG = "FLOWMETER";

static QueueHandle_t drain_evt_queue = NULL;
static QueueHandle_t source_evt_queue = NULL;

static float drain_flow_rate = 0.0f;
static float source_flow_rate = 0.0f;

// Calibration constants (replace with your calibration data)
#define PULSES_PER_LITER        450.0f  // Example value for flowmeter

// Forward declaration of helper functions
static float calculate_flow_rate(int num_pulses);

// RMT receive configuration handles
static rmt_channel_handle_t rmt_rx_channel_drain = NULL;
static rmt_channel_handle_t rmt_rx_channel_source = NULL;

// RMT receive callback function
static bool IRAM_ATTR rmt_rx_drain_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    size_t num_items = edata->num_symbols;

    // Send the number of items (pulses) to the queue
    xQueueSendFromISR(drain_evt_queue, &num_items, &high_task_wakeup);

    // Return whether a higher priority task was woken up
    return high_task_wakeup == pdTRUE;
}

static bool IRAM_ATTR rmt_rx_source_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    size_t num_items = edata->num_symbols;

    // Send the number of items (pulses) to the queue
    xQueueSendFromISR(source_evt_queue, &num_items, &high_task_wakeup);

    // Return whether a higher priority task was woken up
    return high_task_wakeup == pdTRUE;
}

esp_err_t flowmeter_init(void) {
    esp_err_t ret = ESP_OK;

    // Create event queues
    drain_evt_queue = xQueueCreate(10, sizeof(size_t));
    if (!drain_evt_queue) {
        ESP_LOGE(TAG, "Failed to create drain event queue");
        return ESP_FAIL;
    }

    source_evt_queue = xQueueCreate(10, sizeof(size_t));
    if (!source_evt_queue) {
        ESP_LOGE(TAG, "Failed to create source event queue");
        return ESP_FAIL;
    }

    // Initialize RMT receiver for drain flowmeter
    rmt_rx_channel_config_t drain_channel_config = {
        .gpio_num = FLOWMETER_GPIO_DRAIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 MHz resolution
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    ret = rmt_new_rx_channel(&drain_channel_config, &rmt_rx_channel_drain);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT RX channel for drain");
        return ret;
    }

    // Register RX event callback for drain
    rmt_rx_event_callbacks_t drain_cbs = {
        .on_recv_done = rmt_rx_drain_callback,
    };
    ret = rmt_rx_register_event_callbacks(rmt_rx_channel_drain, &drain_cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register RX callback for drain");
        return ret;
    }

    // Enable RMT RX channel for drain
    ret = rmt_enable(rmt_rx_channel_drain);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT RX channel for drain");
        return ret;
    }

    // Start receiving on drain channel
    ret = rmt_receive(rmt_rx_channel_drain, NULL, 0, &((rmt_receive_config_t){
        .signal_range_min_ns = 1000,   // Filter out pulses shorter than 1 us
        .signal_range_max_ns = 1000000 // Filter out pulses longer than 1 ms
    }));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT receive for drain");
        return ret;
    }

    // Initialize RMT receiver for source flowmeter (similar steps)
    rmt_rx_channel_config_t source_channel_config = {
        .gpio_num = FLOWMETER_GPIO_SOURCE,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 MHz resolution
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    ret = rmt_new_rx_channel(&source_channel_config, &rmt_rx_channel_source);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT RX channel for source");
        return ret;
    }

    // Register RX event callback for source
    rmt_rx_event_callbacks_t source_cbs = {
        .on_recv_done = rmt_rx_source_callback,
    };
    ret = rmt_rx_register_event_callbacks(rmt_rx_channel_source, &source_cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register RX callback for source");
        return ret;
    }

    // Enable RMT RX channel for source
    ret = rmt_enable(rmt_rx_channel_source);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT RX channel for source");
        return ret;
    }

    // Start receiving on source channel
    ret = rmt_receive(rmt_rx_channel_source, NULL, 0, &((rmt_receive_config_t){
        .signal_range_min_ns = 1000,   // Filter out pulses shorter than 1 us
        .signal_range_max_ns = 1000000 // Filter out pulses longer than 1 ms
    }));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT receive for source");
        return ret;
    }

    ESP_LOGI(TAG, "Flowmeter RMT initialized");
    return ESP_OK;
}

void flowmeter_task(void *pvParameter) {
    size_t drain_num_pulses = 0;
    size_t source_num_pulses = 0;

    while (1) {
        // Wait for data from drain flowmeter
        if (xQueueReceive(drain_evt_queue, &drain_num_pulses, pdMS_TO_TICKS(1000))) {
            drain_flow_rate = calculate_flow_rate(drain_num_pulses);
            ESP_LOGI(TAG, "Drain Flow Rate: %.2f L/min", drain_flow_rate);
        } else {
            // No data received, flow rate is zero
            drain_flow_rate = 0.0f;
        }

        // Wait for data from source flowmeter
        if (xQueueReceive(source_evt_queue, &source_num_pulses, pdMS_TO_TICKS(1000))) {
            source_flow_rate = calculate_flow_rate(source_num_pulses);
            ESP_LOGI(TAG, "Source Flow Rate: %.2f L/min", source_flow_rate);
        } else {
            // No data received, flow rate is zero
            source_flow_rate = 0.0f;
        }

        // Re-start RMT receive for drain
        esp_err_t ret = rmt_receive(rmt_rx_channel_drain, NULL, 0, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart RMT receive for drain");
        }

        // Re-start RMT receive for source
        ret = rmt_receive(rmt_rx_channel_source, NULL, 0, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart RMT receive for source");
        }

        // Delay before next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static float calculate_flow_rate(int num_pulses) {
    // Calculate flow rate in liters per minute
    // Example calculation: flow_rate = (num_pulses / PULSES_PER_LITER) * (60 seconds / measurement_interval)
    // Since measurement_interval is 1 second, the calculation simplifies
    float flow_rate = (num_pulses / PULSES_PER_LITER) * 60.0f;
    return flow_rate;
}

float get_drain_flow_rate(void) {
    return drain_flow_rate;
}

float get_source_flow_rate(void) {
    return source_flow_rate;
}
