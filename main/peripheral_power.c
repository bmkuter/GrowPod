/**
 * @file peripheral_power.c
 * @brief Control power to peripheral devices (sensors, display, etc.)
 */

#include "peripheral_power.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PERIPH_PWR";

// GPIO pin that controls power to peripherals (sensors, display)
#define PIN_NUM_POWER_TOGGLE 21

esp_err_t peripheral_power_init(void)
{
    ESP_LOGI(TAG, "Initializing peripheral power control");
    
    // Configure GPIO 21 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_POWER_TOGGLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", PIN_NUM_POWER_TOGGLE, esp_err_to_name(ret));
        return ret;
    }
    
    // Enable power to peripherals
    gpio_set_level(PIN_NUM_POWER_TOGGLE, 1);
    ESP_LOGI(TAG, "Peripheral power enabled on GPIO %d", PIN_NUM_POWER_TOGGLE);
    
    // Wait for power rails to stabilize and sensors to power up
    // Some sensors (especially optical sensors like TSL2591) need 100-500ms
    ESP_LOGI(TAG, "Waiting 500ms for peripheral power rails to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Peripheral power stabilization complete");
    
    return ESP_OK;
}
