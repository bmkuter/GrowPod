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
#include "pod_state.h"          // For pod state and fill percent
#include "filesystem/config_manager.h"  // For plant info
#include "sensors/sensor_api.h" // For centralized sensor readings
#include "sensors/sensor_manager.h" // For sensor_data_t and SENSOR_TYPE_LIGHT
#include <string.h>

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
static lv_obj_t *plant_info_label;       // Plant name + days
static lv_obj_t *power_label;            // Consolidated: Current, Voltage, Power
static lv_obj_t *temp_humidity_label;    // Temperature + Humidity
static lv_obj_t *light_label;            // Light sensor (lux)
static lv_obj_t *water_level_label;
static lv_obj_t *planter_pwm_label;
static lv_obj_t *led_pwm_label;

// LVGL timer for updating sensor readings
static lv_timer_t *sensor_update_timer = NULL;

// Text scrolling state for plant info
typedef enum {
    SCROLL_STATE_IDLE,       // Paused at start position (5 seconds)
    SCROLL_STATE_SCROLLING   // Continuously scrolling right with wrapping
} scroll_state_t;

static scroll_state_t scroll_state = SCROLL_STATE_IDLE;
static uint32_t scroll_timer_ms = 0;
static char plant_info_full_text[128] = {0};  // Store full text
static char plant_info_display_text[300] = {0};  // Store text with wrapping for marquee (needs 2x + spacing)
static int16_t scroll_offset = 0;
static bool text_needs_scroll = false;

// Scroll timing configuration (in milliseconds)
#define SCROLL_PAUSE_MS         5000   // Pause 5s when text completes one full cycle
#define SCROLL_SPEED_MS         100    // Scroll every 100ms (10 chars/sec)
#define SCROLL_SPACING          30     // Space (in pixels) between end and start of text

// Font definitions
#define SIZE_16_FONT  &lv_font_unscii_16
#define SIZE_8_FONT   &lv_font_unscii_8

// Forward declarations
static bool does_text_need_scroll(const char *text, const lv_font_t *font, int16_t max_width);
static void update_plant_info_scroll(void);

// Timer callback to update the display with sensor readings
static void lvgl_sensor_update_cb(lv_timer_t *timer)
{
    float current_ma = 0.0f, voltage_mv = 0.0f, power_mw = 0.0f;
    float temperature_c = 0.0f, humidity_rh = 0.0f;
    int water_level_mm;
    char power_str[48], temp_humidity_str[40], light_str[32],
         water_level_str[32], plant_info_str[128],
         planter_pwm_str[24], led_pwm_str[24];
    
    // CRITICAL: Use ONLY cached reads (non-blocking) in LVGL task
    // The LVGL task must never block or it will trigger the watchdog
    // All sensor reads use sensor_manager_get_data_cached() which returns immediately
    
    // Read power monitor data from cache (non-blocking)
    sensor_data_t power_data;
    esp_err_t ret = sensor_manager_get_data_cached(SENSOR_TYPE_POWER_CURRENT, &power_data, NULL);
    if (ret == ESP_OK) {
        current_ma = power_data.power.value;
    }
    
    ret = sensor_manager_get_data_cached(SENSOR_TYPE_POWER_VOLTAGE, &power_data, NULL);
    if (ret == ESP_OK) {
        voltage_mv = power_data.power.value;
    }
    
    ret = sensor_manager_get_data_cached(SENSOR_TYPE_POWER_POWER, &power_data, NULL);
    if (ret == ESP_OK) {
        power_mw = power_data.power.value;
    }
    
    // Read temperature and humidity from cache (non-blocking)
    sensor_data_t env_data;
    ret = sensor_manager_get_data_cached(SENSOR_TYPE_TEMPERATURE_AND_HUMIDITY, &env_data, NULL);
    if (ret == ESP_OK) {
        temperature_c = env_data.environment.temperature_c;
        humidity_rh = env_data.environment.humidity_rh;
    }
    
    // Read light sensor from cache (non-blocking)
    sensor_data_t light_data;
    ret = sensor_manager_get_data_cached(SENSOR_TYPE_LIGHT, &light_data, NULL);
    if (ret != ESP_OK) {
        light_data.light.lux = 0.0f;
    }
    
    // Read water level sensor from cache (non-blocking)
    sensor_data_t water_level_data;
    float water_fill_percent = 0.0f;
    ret = sensor_manager_get_data_cached(SENSOR_TYPE_WATER_LEVEL, &water_level_data, NULL);
    if (ret == ESP_OK) {
        water_fill_percent = water_level_data.water_level.fill_percent;
        water_level_mm = (int)water_level_data.water_level.level_mm;  // Legacy, for reference only
    } else {
        // Fallback to direct reading if sensor manager cache isn't available
        water_fill_percent = fdc1004_read_fill_percent();
        water_level_mm = distance_sensor_read_mm();
    }
    
    // Format power in SI units (A, V, W)
    float current_a = current_ma / 1000.0f;
    float voltage_v = voltage_mv / 1000.0f;
    float power_w = power_mw / 1000.0f;
    snprintf(power_str, sizeof(power_str), "%.2fA %.2fV %.2fW", current_a, voltage_v, power_w);
    
    // Temperature and humidity (compact format)
    float temp_f = (temperature_c * 9.0f / 5.0f) + 32.0f;
    snprintf(temp_humidity_str, sizeof(temp_humidity_str), "%.1fC(%.0fF) %.0f%%RH", 
             temperature_c, temp_f, humidity_rh);
    
    // Light sensor (lux)
    snprintf(light_str, sizeof(light_str), "Light:%.0flux", light_data.light.lux);
    
    // Water level as fill percentage (primary display)
    snprintf(water_level_str, sizeof(water_level_str), "Water:%.0f%%", water_fill_percent);

    // Get plant info and calculate days growing
    plant_info_t plant_info = {0};
    esp_err_t plant_err = config_load_plant_info(&plant_info);
    if (plant_err == ESP_OK && strlen(plant_info.plant_name) > 0) {
        int32_t days = config_get_days_growing(&plant_info);
        if (days >= 0) {
            snprintf(plant_info_str, sizeof(plant_info_str), "%s - Day %ld", 
                     plant_info.plant_name, (long)days);
        } else {
            snprintf(plant_info_str, sizeof(plant_info_str), "%s - Day --", 
                     plant_info.plant_name);
        }
    } else {
        snprintf(plant_info_str, sizeof(plant_info_str), "No plant set");
    }

    // Update plant info and check if scrolling is needed
    // Only reset scroll state if text changed
    if (strcmp(plant_info_full_text, plant_info_str) != 0) {
        strncpy(plant_info_full_text, plant_info_str, sizeof(plant_info_full_text) - 1);
        plant_info_full_text[sizeof(plant_info_full_text) - 1] = '\0';
        
        // Check if text needs scrolling (leave ~10px margin on right)
        text_needs_scroll = does_text_need_scroll(plant_info_full_text, SIZE_16_FONT, LCD_WIDTH - 10);
        
        // Create marquee text by duplicating with spacing for wrapping effect
        if (text_needs_scroll) {
            // Safely create wrapped text: "text     text"
            size_t len = strlen(plant_info_full_text);
            size_t max_len = sizeof(plant_info_display_text) - 1;
            
            // First part: original text
            strncpy(plant_info_display_text, plant_info_full_text, max_len);
            size_t pos = (len < max_len) ? len : max_len;
            
            // Middle part: spacing (5 spaces)
            if (pos + 5 < max_len) {
                memcpy(plant_info_display_text + pos, "     ", 5);
                pos += 5;
            }
            
            // Last part: repeated text
            if (pos < max_len) {
                strncpy(plant_info_display_text + pos, plant_info_full_text, max_len - pos);
            }
            
            plant_info_display_text[max_len] = '\0';
        } else {
            strncpy(plant_info_display_text, plant_info_full_text, sizeof(plant_info_display_text) - 1);
            plant_info_display_text[sizeof(plant_info_display_text) - 1] = '\0';
        }
        
        // Reset scroll state when text changes
        scroll_state = SCROLL_STATE_IDLE;
        scroll_timer_ms = 0;
        scroll_offset = 0;
        lv_obj_set_x(plant_info_label, 3);
    }
    
    lv_label_set_text(plant_info_label, plant_info_display_text);
    
    // For scrolling text, enable long mode to allow text to extend beyond width
    if (text_needs_scroll) {
        lv_label_set_long_mode(plant_info_label, LV_LABEL_LONG_CLIP);
    } else {
        lv_label_set_long_mode(plant_info_label, LV_LABEL_LONG_CLIP);
    }
    
    // Update scrolling animation
    update_plant_info_scroll();

    // Read actuator states (compact format)
    const actuator_info_t *act = actuator_control_get_info();
    float planter_duty = act[ACTUATOR_IDX_PLANTER_PUMP].duty_percentage;
    float led_duty     = act[ACTUATOR_IDX_LED_ARRAY].duty_percentage;
    snprintf(planter_pwm_str, sizeof(planter_pwm_str), "Pump:%.0f%%", planter_duty);
    snprintf(led_pwm_str,     sizeof(led_pwm_str),     "LED:%.0f%%",  led_duty);

    // Update UI labels (plant_info_label already updated above with scrolling)
    lv_label_set_text(power_label, power_str);
    lv_label_set_text(temp_humidity_label, temp_humidity_str);
    lv_label_set_text(light_label, light_str);
    lv_label_set_text(water_level_label, water_level_str);
    lv_label_set_text(planter_pwm_label, planter_pwm_str);
    lv_label_set_text(led_pwm_label, led_pwm_str);
}

/**
 * @brief Check if text is too wide for the display
 * @return true if text needs scrolling
 */
static bool does_text_need_scroll(const char *text, const lv_font_t *font, int16_t max_width)
{
    lv_point_t size;
    lv_txt_get_size(&size, text, font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    return (size.x > max_width);
}

/**
 * @brief Update scrolling animation for plant info label
 * Called periodically from sensor update callback
 * Implements circular/marquee scrolling with wrapping text
 */
static void update_plant_info_scroll(void)
{
    static uint32_t last_update_ms = 0;
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed_ms = current_ms - last_update_ms;
    
    if (!text_needs_scroll) {
        // Text fits, no scrolling needed
        scroll_state = SCROLL_STATE_IDLE;
        scroll_offset = 0;
        lv_obj_set_x(plant_info_label, 3);
        return;
    }
    
    // Get original text width (not the duplicated display text)
    lv_point_t size;
    lv_txt_get_size(&size, plant_info_full_text, SIZE_16_FONT, 0, 0, 
                   LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    int16_t text_width = size.x;
    
    // Calculate width of spacing (5 spaces in 16px font)
    lv_point_t spacing_size;
    lv_txt_get_size(&spacing_size, "     ", SIZE_16_FONT, 0, 0, 
                   LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    int16_t spacing_width = spacing_size.x;
    
    // Full cycle = original text + spacing between duplicates
    int16_t full_cycle = text_width + spacing_width;
    
    switch (scroll_state) {
        case SCROLL_STATE_IDLE:
            // Pause at start position for 5 seconds
            lv_obj_set_x(plant_info_label, 3);
            scroll_offset = 0;
            scroll_timer_ms += elapsed_ms;
            if (scroll_timer_ms >= SCROLL_PAUSE_MS) {
                scroll_state = SCROLL_STATE_SCROLLING;
                scroll_timer_ms = 0;
                scroll_offset = 0;
            }
            break;
            
        case SCROLL_STATE_SCROLLING:
            // Continuously scroll LEFT (negative offset reveals text to the right)
            scroll_timer_ms += elapsed_ms;
            if (scroll_timer_ms >= SCROLL_SPEED_MS) {
                scroll_offset -= 10;  // Move 10 pixels LEFT (negative = scroll left)
                scroll_timer_ms = 0;
                
                // Check if we've completed one full cycle (text wrapped back to start)
                // When offset reaches -(text_width + spacing), we've seen the full text wrap
                if (scroll_offset <= -full_cycle) {
                    // Reset to start and pause
                    scroll_offset = 0;
                    lv_obj_set_x(plant_info_label, 3);
                    scroll_state = SCROLL_STATE_IDLE;
                    scroll_timer_ms = 0;
                } else {
                    // Continue scrolling with wrapping effect
                    lv_obj_set_x(plant_info_label, 3 + scroll_offset);
                }
            }
            break;
    }
    
    last_update_ms = current_ms;
}

void display_lvgl_init(void)
{
    /* Lock LVGL mutex - CRITICAL when LVGL task runs on different core!
     * All LVGL API calls from outside the LVGL task must be protected.
     */
    if (!lvgl_port_lock(1000)) {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock within 1000ms");
        return;
    }

    /* Set LVGL background to black */
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    
    /* Disable scrollbars on the screen */
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(lv_scr_act(), LV_SCROLLBAR_MODE_OFF);

    // Top row: Plant name and days (y=3) - 16px font for prominence
    plant_info_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(plant_info_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(plant_info_label, SIZE_16_FONT, LV_PART_MAIN);
    lv_label_set_text(plant_info_label, "No plant set");
    lv_obj_align(plant_info_label, LV_ALIGN_TOP_LEFT, 3, 3);

    // Row y=23: Power (consolidated - Current, Voltage, Power in SI units)
    power_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(power_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(power_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(power_label, "0.00A 0.00V 0.00W");
    lv_obj_align(power_label, LV_ALIGN_TOP_LEFT, 3, 23);

    // Row y=35: Temperature + Humidity
    temp_humidity_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(temp_humidity_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(temp_humidity_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(temp_humidity_label, "0.0C(0F) 0%RH");
    lv_obj_align(temp_humidity_label, LV_ALIGN_TOP_LEFT, 3, 35);

    // Row y=47: Light sensor
    light_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(light_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(light_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(light_label, "Light:0lux");
    lv_obj_align(light_label, LV_ALIGN_TOP_LEFT, 3, 47);

    // Row y=59: Water level
    water_level_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(water_level_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(water_level_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(water_level_label, "Water:0%");
    lv_obj_align(water_level_label, LV_ALIGN_TOP_LEFT, 3, 59);

    // Actuators - Bottom rows
    // Row y=71: Planter PWM
    planter_pwm_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(planter_pwm_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(planter_pwm_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(planter_pwm_label, "Pump:0%");
    lv_obj_align(planter_pwm_label, LV_ALIGN_TOP_LEFT, 3, 71);

    // Row y=83: LED PWM
    led_pwm_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(led_pwm_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(led_pwm_label, SIZE_8_FONT, LV_PART_MAIN);
    lv_label_set_text(led_pwm_label, "LED:0%");
    lv_obj_align(led_pwm_label, LV_ALIGN_TOP_LEFT, 3, 83);

    // Create LVGL timer for sensor updates (250ms = 4Hz)
    sensor_update_timer = lv_timer_create(lvgl_sensor_update_cb, 250, NULL);
    if (sensor_update_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL sensor update timer");
    }

    /* Unlock LVGL mutex */
    lvgl_port_unlock();
}

// Helper: fill entire screen with a solid color
// Clear in chunks using single reusable buffer with delays to manage SPI queue
static esp_err_t panel_clear_color(esp_lcd_panel_handle_t panel, uint16_t color)
{
    const int CHUNK_LINES = 10;  // Process 10 lines at a time
    const size_t chunk_pixels = LCD_WIDTH * CHUNK_LINES;
    const size_t chunk_bytes = chunk_pixels * sizeof(uint16_t);
    
    // Allocate single reusable buffer
    uint16_t *buf = heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for screen clear", chunk_bytes);
        return ESP_ERR_NO_MEM;
    }
    
    // Fill buffer with color once
    for (size_t i = 0; i < chunk_pixels; i++) {
        buf[i] = color;
    }
    
    ESP_LOGI(TAG, "Clearing display to color 0x%04X using %zu byte buffer", color, chunk_bytes);
    
    // Draw in chunks with delay to respect queue depth
    esp_err_t ret = ESP_OK;
    for (int y = 0; y < LCD_HEIGHT; y += CHUNK_LINES) {
        int lines_to_draw = (y + CHUNK_LINES <= LCD_HEIGHT) ? CHUNK_LINES : (LCD_HEIGHT - y);
        
        ret = esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_WIDTH, y + lines_to_draw, buf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw at y=%d: %s", y, esp_err_to_name(ret));
            break;
        }

        // Wait 25ms between chunks to allow SPI queue to drain
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    
    heap_caps_free(buf);
    return ret;
}

esp_lcd_panel_handle_t display_init(void)
{
    ESP_LOGI(TAG, "Starting display_init()");

    // Note: Power control (GPIO 21) is now handled by peripheral_power_init()
    // which is called in main.c before sensor manager initialization.
    // This ensures sensors get power before we try to initialize them.

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
        .trans_queue_depth = 20,  // Increased from 10 to handle 14 strips + headroom for LVGL
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
    // Adjusted to eliminate white bar at bottom: shifted vertical gap down
    ret = esp_lcd_panel_set_gap(panel_handle, 40, 52);
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
