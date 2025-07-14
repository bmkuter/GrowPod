#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"  // for esp_lcd_panel_handle_t
#include "esp_lcd_panel_io.h"    // for esp_lcd_panel_io_handle_t

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

/**
 * @brief Initialize display hardware and start the DisplayTask.
 * 
 * @return esp_lcd_panel_handle_t Handle to the initialized LCD panel.
 */
esp_lcd_panel_handle_t display_init(void);

/**
 * @brief Create the LVGL user interface.
 */
void display_lvgl_init(void);

/**
 * @brief Get the panel IO handle initialized in display_init.
 *
 * @return esp_lcd_panel_io_handle_t Handle to the LCD panel IO.
 */
esp_lcd_panel_io_handle_t display_get_io_handle(void);
