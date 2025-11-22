#include "contact_sensor.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CONTACT_SENSOR";

esp_err_t contact_sensor_init_hw(void)
{
    // Configure all GPA and GPB as inputs
    uint8_t config_reg = 0xFF;
    // IODIRA (0x00), IODIRB (0x01)
    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x00, config_reg}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x01, config_reg}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;

    // Enable pull-ups on GPA and GPB (GPPUA 0x0C, GPPUB 0x0D)
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x0C, config_reg}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x0D, config_reg}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;

    ESP_LOGI(TAG, "MCP23017 initialized: all pins as inputs with pull-ups");
    return ESP_OK;
fail:
    ESP_LOGE(TAG, "Failed to init MCP23017: %s", esp_err_to_name(err));
    return err;
}

int contact_sensor_read_mm_actual(void)
{
    // Read inputs from GPIOA (0x12) and GPIOB (0x13)
    uint8_t levels[2] = {0};
    esp_err_t err = i2c_master_write_read_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x12}, 1, levels, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read GPIO: %s", esp_err_to_name(err));
        return -1;
    }

    // Print raw received value for debugging
    ESP_LOGI(TAG, "Raw contact sensor reading: A=0x%02X, B=0x%02X", levels[0], levels[1]);

    // Combine into 16-bit mask, A bits in LSB
    uint16_t mask = ((uint16_t)levels[1] << 8) | levels[0];

    // Determine current pad index (first grounded bit)
    int current_index = -1;
    for (int i = 0; i < CONTACT_SENSOR_PAD_COUNT; i++) {
        if ((mask & (1 << i)) == 0) {
            current_index = i;
            break;
        }
    }

    // Debounce state (retain across calls)
    static int last_reported_index = -1;
    static int last_seen_index = -1;
    static int last_seen_count = 0;

    // Update debounce counters
    if (current_index == last_seen_index) {
        last_seen_count++;
    } else {
        last_seen_index = current_index;
        last_seen_count = 1;
    }

    // Only update reported index after threshold
    if (last_seen_count >= CONTACT_SENSOR_DEBOUNCE_COUNT) {
        last_reported_index = current_index;
    }

    // If we have a stable pad index, convert to height
    if (last_reported_index >= 0) {
        int step_mm = CONTACT_SENSOR_MAX_HEIGHT_MM / (CONTACT_SENSOR_PAD_COUNT - 1);

        // Log the stable reading
        ESP_LOGI(TAG, "Stable contact sensor reading: index=%d, height=%d mm", last_reported_index, last_reported_index * step_mm);

        return last_reported_index * step_mm;
    }

    // No stable reading yet
    ESP_LOGW(TAG, "No stable contact sensor reading");
    return -1;
}
