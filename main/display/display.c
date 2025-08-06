#include "display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <esp_heap_caps.h>
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "power_monitor_HAL.h"  // For power monitoring
#include "distance_sensor.h"    // For water level sensor
#include "actuator_control.h"   

static const char *TAG = "display";

// Pin definitions for ST7789
#define LCD_HOST    SPI2_HOST

#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   7
#define PIN_NUM_POWER_TOGGLE 21
#define PIN_NUM_DC   39
#define PIN_NUM_RST  40
#define PIN_NUM_BCKL 45

#define LCD_BK_LIGHT_ON_LEVEL   1

// Panel dimensions
#define LCD_WIDTH  240
#define LCD_HEIGHT 135

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t panel_io_handle = NULL;

// UI Labels for sensor data
static lv_obj_t *title_label;
static lv_obj_t *current_label;
static lv_obj_t *voltage_label;
static lv_obj_t *power_label;
static lv_obj_t *water_level_label;
static lv_obj_t *timestamp_label;
static lv_obj_t *planter_pwm_label;
static lv_obj_t *led_pwm_label;

// Task to update the display with sensor readings
static void lvgl_update_task(void *pvParameter)
{
    esp_err_t ret;
    float current_ma, voltage_mv, power_mw;
    int water_level_mm;
    // add buffers for actuator text
    char current_str[32], voltage_str[32], power_str[32],
         water_level_str[32], timestamp_str[32],
         planter_pwm_str[32], led_pwm_str[32];
    int counter = 0;
    static bool power_monitor_initialized = false;
    
    // Initialize I2C for power monitor
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
    }
    
    // Initialize power monitor
    if (!power_monitor_initialized) {
        ret = power_monitor_init(POWER_MONITOR_CHIP_INA219, INA219_ADDRESS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize power monitor: %s", esp_err_to_name(ret));
        } else {
            power_monitor_initialized = true;
        }
    }

    while (1) {
        // Read power monitor data
        ret = power_monitor_read_current(&current_ma);
        ret |= power_monitor_read_voltage(&voltage_mv);
        ret |= power_monitor_read_power(&power_mw);
        
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error reading power monitor data: %s", esp_err_to_name(ret));
            // Use default values if read fails
            current_ma = 0.0f;
            voltage_mv = 0.0f;
            power_mw = 0.0f;
        }
        
        // Read water level sensor (use non-blocking function)
        water_level_mm = distance_sensor_read_mm();
        
        // Format strings for display
        snprintf(current_str, sizeof(current_str), "Current: %.2f mA", current_ma);
        snprintf(voltage_str, sizeof(voltage_str), "Voltage: %.2f mV", voltage_mv);
        snprintf(power_str, sizeof(power_str), "Power:   %.2f mW", power_mw);
        snprintf(water_level_str, sizeof(water_level_str), "Water level: %d mm", water_level_mm);
        snprintf(timestamp_str, sizeof(timestamp_str), "Updated: %d", (counter)); counter++;

        // Read actuator states
        const actuator_info_t *act = actuator_control_get_info();
        float planter_duty = act[ACTUATOR_IDX_PLANTER_PUMP].duty_percentage;
        float led_duty     = act[ACTUATOR_IDX_LED_ARRAY].duty_percentage;
        snprintf(planter_pwm_str, sizeof(planter_pwm_str), "Planter: %.0f%%", planter_duty);
        snprintf(led_pwm_str,     sizeof(led_pwm_str),     "LED: %.0f%%",     led_duty);

        // Update UI labels
        lv_label_set_text(current_label, current_str);
        lv_label_set_text(voltage_label, voltage_str);
        lv_label_set_text(power_label, power_str);
        lv_label_set_text(water_level_label, water_level_str);
        lv_label_set_text(timestamp_label, timestamp_str);
        lv_label_set_text(planter_pwm_label, planter_pwm_str);
        lv_label_set_text(led_pwm_label,     led_pwm_str);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

#define SIZE_16_FONT  &lv_font_unscii_16
#define SIZE_8_FONT   &lv_font_unscii_8

void display_lvgl_init(void)
{
    /* Set LVGL background to black */
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

    // Create title label (16 px high)
    title_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(title_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(title_label, SIZE_16_FONT, LV_PART_MAIN);
    lv_label_set_text(title_label, "GrowPod Sensors");
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 5);

    // Row y=25: current measurement
    current_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(current_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(current_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(current_label, "Current: 0.00 mA");
    lv_obj_align(current_label, LV_ALIGN_TOP_LEFT, 5, 25);

    // Row y=37: voltage
    voltage_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(voltage_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(voltage_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(voltage_label, "Voltage: 0.00 mV");
    lv_obj_align(voltage_label, LV_ALIGN_TOP_LEFT, 5, 37);

    // Row y=49: power
    power_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(power_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(power_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(power_label, "Power:   0.00 mW");
    lv_obj_align(power_label, LV_ALIGN_TOP_LEFT, 5, 49);

    // Row y=61: water level
    water_level_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(water_level_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(water_level_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(water_level_label, "Water level: 0 mm");
    lv_obj_align(water_level_label, LV_ALIGN_TOP_LEFT, 5, 61);

    // Row y=73: planter PWM
    planter_pwm_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(planter_pwm_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(planter_pwm_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(planter_pwm_label, "Planter: 0%");
    lv_obj_align(planter_pwm_label, LV_ALIGN_TOP_LEFT, 5, 73);

    // Row y=85: LED PWM
    led_pwm_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(led_pwm_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(led_pwm_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(led_pwm_label, "LED: 0%");
    lv_obj_align(led_pwm_label, LV_ALIGN_TOP_LEFT, 5, 85);

    // Timestamp bottom right
    timestamp_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(timestamp_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(timestamp_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(timestamp_label, "Updated: 0 s");
    lv_obj_align(timestamp_label, LV_ALIGN_BOTTOM_RIGHT, -5, -5);

    // Launch update task
    xTaskCreate(lvgl_update_task, "lvgl_update_task", 4096, NULL, 5, NULL);
}

// Helper: fill entire screen with a solid color
static esp_err_t panel_clear_color(esp_lcd_panel_handle_t panel, uint16_t color)
{
    size_t px_count = LCD_WIDTH * LCD_HEIGHT;
    uint16_t *buf = heap_caps_malloc(px_count * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buf) return ESP_ERR_NO_MEM;
    for (size_t i = 0; i < px_count; i++) {
        buf[i] = color;
    }
    esp_err_t ret = esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_WIDTH, LCD_HEIGHT, buf);
    heap_caps_free(buf);
    return ret;
}

esp_lcd_panel_handle_t display_init(void)
{
    ESP_LOGI(TAG, "Starting display_init()");

    // Power on display
    gpio_set_direction(PIN_NUM_POWER_TOGGLE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_POWER_TOGGLE, 1);
    ESP_LOGI(TAG, "Power toggled on pin %d", PIN_NUM_POWER_TOGGLE);

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t)
    };
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "SPI bus initialized");

    // Configure panel IO
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .flags = {
            .dc_high_on_cmd = 0,
            .dc_low_on_data = 0,
            .dc_low_on_param = 0,
            .octal_mode = 0,
            .quad_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));
    // Store IO handle for LVGL port
    panel_io_handle = io_handle;

    // Configure panel driver
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_dev_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_dev_config, &panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_st7789 failed: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "Panel driver created");

    // Turn on backlight
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BCKL, LCD_BK_LIGHT_ON_LEVEL);
    ESP_LOGI(TAG, "Backlight on pin %d level %d", PIN_NUM_BCKL, LCD_BK_LIGHT_ON_LEVEL);

    panel_handle = panel;

    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_reset failed: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "Panel reset");

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_init failed: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "Panel initialized");

    // Rotate display 180°
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, true);

    // Invert display if needed for this particular module
    ret = esp_lcd_panel_invert_color(panel_handle, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_invert_color failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    // Set gap offsets for 1.14" 240x135 display (ST7789 240x320 cropped)
    ret = esp_lcd_panel_set_gap(panel_handle, 40, 52); // adjusted vertical gap by -2 to remove top bar
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_set_gap failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    // Turn on display 
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_disp_on_off failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    ret = panel_clear_color(panel_handle, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel_clear_color failed: %s", esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "Screen cleared");

    return panel_handle;
}

/**
 * @brief Get the panel IO handle initialized in display_init
 *
 * @return esp_lcd_panel_io_handle_t Stored IO handle
 */
esp_lcd_panel_io_handle_t display_get_io_handle(void)
{
    return panel_io_handle;
}
