#include "contact_sensor.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CONTACT_SENSOR";

// Pin mapping: height in mm -> (port, bit)
// Port A = 0, Port B = 1
typedef struct {
    uint8_t height_mm;
    uint8_t port;      // 0=A, 1=B
    uint8_t bit;       // 0-7
} pin_map_t;

// Working pins only (broken pins excluded)
static const pin_map_t PIN_MAP[] = {
    {5,  0, 7},  // A7 - This will be our current source
    {10, 1, 3},  // B3
    {15, 1, 2},  // B2
    {25, 0, 0},  // A0
    {45, 1, 1},  // B1
    {55, 1, 4},  // B4
    {60, 0, 5},  // A5
    {70, 0, 2},  // A2
    {80, 0, 6},  // A6
};
#define PIN_MAP_COUNT (sizeof(PIN_MAP) / sizeof(pin_map_t))

esp_err_t contact_sensor_init_hw(void)
{
    esp_err_t err;
    
    // Step 1: Set A7 (5mm pin) as OUTPUT, all others as INPUT
    // IODIRA: bit 7 = 0 (output), bits 0,2,5,6 = 1 (input)
    // IODIRB: bits 1,2,3,4 = 1 (input)
    uint8_t dir_a = 0xFF & ~(1 << 7);  // A7 = output, rest = input
    uint8_t dir_b = 0xFF;               // All inputs
    
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x00, dir_a}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;
    
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x01, dir_b}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;

    // Step 2: Enable pull-ups on INPUT pins (not on A7 output)
    uint8_t pullup_a = 0xFF & ~(1 << 7);  // Pull-ups on all except A7
    uint8_t pullup_b = 0xFF;                // Pull-ups on all B pins
    
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x0C, pullup_a}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;
    
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x0D, pullup_b}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;

    // Step 3: Set A7 LOW to act as current source/sink
    // Write to GPIOA register (0x12)
    uint8_t gpio_a = 0x00;  // A7 = LOW, others don't matter (they're inputs)
    err = i2c_master_write_to_device(I2C_NUM_0, CONTACT_SENSOR_I2C_ADDR,
        (uint8_t[]){0x12, gpio_a}, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) goto fail;

    ESP_LOGI(TAG, "MCP23017 initialized: A7 as LOW output (current source), other pins as inputs with pull-ups");
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

    uint8_t gpio_a = levels[0];
    uint8_t gpio_b = levels[1];

    // Log the state of only the pins we're using
    ESP_LOGI(TAG, "Pin states: A7=%d A6=%d A5=%d A2=%d A0=%d | B4=%d B3=%d B2=%d B1=%d",
             (gpio_a >> 7) & 1,  // A7 (5mm - should always be 0 since it's our output)
             (gpio_a >> 6) & 1,  // A6 (80mm)
             (gpio_a >> 5) & 1,  // A5 (60mm)
             (gpio_a >> 2) & 1,  // A2 (70mm)
             (gpio_a >> 0) & 1,  // A0 (25mm)
             (gpio_b >> 4) & 1,  // B4 (55mm)
             (gpio_b >> 3) & 1,  // B3 (10mm)
             (gpio_b >> 2) & 1,  // B2 (15mm)
             (gpio_b >> 1) & 1); // B1 (45mm)

    // Find the highest water level detected
    // We're looking for pins that read LOW (connected to 5mm pin via water)
    int highest_level_mm = -1;
    
    // Start from index 1 (skip 5mm pin since it's the source)
    for (int i = 1; i < PIN_MAP_COUNT; i++) {
        const pin_map_t *pin = &PIN_MAP[i];
        uint8_t pin_value;
        
        if (pin->port == 0) {
            // Port A
            pin_value = (gpio_a >> pin->bit) & 0x01;
        } else {
            // Port B
            pin_value = (gpio_b >> pin->bit) & 0x01;
        }
        
        // If pin reads LOW, there's connectivity via water
        if (pin_value == 0) {
            ESP_LOGI(TAG, "  %dmm pin (%c%d): LOW - water detected", 
                     pin->height_mm, pin->port ? 'B' : 'A', pin->bit);
            if (pin->height_mm > highest_level_mm) {
                highest_level_mm = pin->height_mm;
            }
        } else {
            ESP_LOGD(TAG, "  %dmm pin (%c%d): HIGH - no water", 
                     pin->height_mm, pin->port ? 'B' : 'A', pin->bit);
        }
    }

    // Debounce the reading
    static int last_reported_level = -1;
    static int last_seen_level = -1;
    static int last_seen_count = 0;

    // Update debounce counters
    if (highest_level_mm == last_seen_level) {
        last_seen_count++;
    } else {
        last_seen_level = highest_level_mm;
        last_seen_count = 1;
    }

    // Only update reported level after threshold
    if (last_seen_count >= CONTACT_SENSOR_DEBOUNCE_COUNT) {
        last_reported_level = highest_level_mm;
    }

    // Return the stable reading
    if (last_reported_level >= 0) {
        ESP_LOGI(TAG, "Stable water level: %d mm", last_reported_level);
        return last_reported_level;
    } else {
        ESP_LOGI(TAG, "No water detected (all pins HIGH)");
        return 0;  // Return 0mm instead of -1 when no water is detected
    }
}
