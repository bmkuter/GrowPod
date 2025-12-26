/**
 * @file i2c_scanner.c
 * @brief Simple I2C bus scanner utility for debugging
 */

#include "i2c_scanner.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_SCANNER";

void i2c_scanner_scan(i2c_port_t i2c_num)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    uint8_t devices_found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at address 0x%02X", addr);
            devices_found++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between probes
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found on bus");
    } else {
        ESP_LOGI(TAG, "Scan complete: %d device(s) found", devices_found);
    }
}
