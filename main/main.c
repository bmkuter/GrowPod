// main/main.c

#include "Networking/wifi_manager.h"
#include "actuator_control.h"
#include "uart_comm.h"
#include "ina260.h"
#include "flowmeter_control.h"
#include "distance_sensor.h"
#include "https_server/https_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mdns.h"
#include "control_logic.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "APP_MAIN";

void start_mdns_service(void) {
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("hydroponics_device"));
    ESP_ERROR_CHECK(mdns_instance_name_set("Hydroponics Device"));

    // Set service type and port
    mdns_service_add("Hydroponics Service", "_hydroponics", "_tcp", 443, NULL, 0);

    ESP_LOGI("MDNS", "mDNS service started");
}

/**
 * @brief Initialize SNTP.
 */
void initialize_sntp(void) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}

/**
 * @brief Set the timezone to US Eastern (Boston).
 * 
 * The TZ string "EST5EDT,M3.2.0,M11.1.0" means:
 * - Standard time is EST (UTC-5)
 * - Daylight time is EDT (UTC-4)
 * - DST starts on the second Sunday in March and ends on the first Sunday in November.
 */
void set_timezone_to_boston(void) {
    setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
    tzset();
}

void app_main(void) {
    vTaskDelay(1000/portTICK_PERIOD_MS);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      (nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGW(TAG, "NVS Status: %d", ret);

    i2c_master_init();
    // Initialize Wi-Fi
    if (wifi_init() == ESP_OK) {
        // Start mDNS
        start_mdns_service();
        // Start HTTPS server
        start_https_server();
    } else {
        ESP_LOGE("MAIN", "Failed to initialize Wi-Fi");
    }

    vTaskDelay(1000/portTICK_PERIOD_MS);

    // Initialize SNTP and wait for time synchronization
    initialize_sntp();
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (time(&now) < 1000000000 && retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry+1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        retry++;
    }
    // Set timezone to US Eastern (Boston) and update local time info
    set_timezone_to_boston();
    localtime_r(&now, &timeinfo);
    ESP_LOGW(TAG, "Time is set: %s", asctime(&timeinfo));

    vTaskDelay(1000/portTICK_PERIOD_MS);

    // Start important peripherals

    uart_comm_init();
    actuator_control_init();
    init_schedule_manager();
    // Initialize UART console
    xTaskCreate(uart_console_task, "uart_console_task", 8192, NULL, 5, NULL);

    ret = distance_sensor_init();
    // if (ret == ESP_OK) {
    //     xTaskCreate(distance_sensor_task, "distance_sensor_task", 2048, NULL, 5, NULL);
    // } else {
    //     ESP_LOGE("MAIN", "distance_sensor_init failed: %s", esp_err_to_name(ret));
    // }

    // Start INA260 task
    xTaskCreate(ina260_task, "ina260_task", 4096, NULL, 5, NULL);

    vTaskDelay(1000/portTICK_PERIOD_MS);

    // Initialize actuators
    xTaskCreate(actuator_control_task, "actuator_control_task", 4096, NULL, 5, NULL);

    // Initialize and start flowmeter task
    if (flowmeter_init() == ESP_OK) {
        xTaskCreate(flowmeter_task, "flowmeter_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE("MAIN", "Failed to initialize flowmeter");
    }

    // Initialize control logic
    ret = control_logic_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize control logic module");
    }
}
