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
#include "display.h"
#include "esp_lvgl_port.h"

static const char *TAG = "APP_MAIN";

void start_mdns_service(void) {
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    char customSuffix[64] = "UNMODIFIED";
    size_t suffix_len = sizeof(customSuffix);
    nvs_handle_t handle;
    esp_err_t err = nvs_open("user_settings", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        nvs_get_str(handle, "mdns_suffix", customSuffix, &suffix_len);
        nvs_close(handle);
    }
    char fullHostname[128];
    snprintf(fullHostname, sizeof(fullHostname), "hydroponics_device_%s", customSuffix);
    printf("mdns: %s\n", fullHostname);
    ESP_ERROR_CHECK(mdns_hostname_set(fullHostname));
    ESP_ERROR_CHECK(mdns_instance_name_set("Hydroponics Device"));

    // Set service type and port
    mdns_service_add("Hydroponics Service", "_hydroponics", "_tcp", 443, NULL, 0);

    ESP_LOGI("MDNS", "mDNS service started");
}

static void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time sync event: %s", asctime(localtime(&tv->tv_sec)));
}

/**
 * @brief Initialize SNTP.
 */
void initialize_sntp(void) {
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time1.google.com");
    esp_sntp_setservername(1, "time2.google.com");
    esp_sntp_setservername(2, "time3.google.com");
    esp_sntp_setservername(3, "time4.google.com");
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
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

void init_screen_wrapper()
{
        /* Initialize LVGL port */
    lvgl_port_init(&((const lvgl_port_cfg_t) { .task_priority = 4, .task_stack = 4096, .timer_period_ms = 5 }));

    /* Initialize display driver and get handles */
    esp_lcd_panel_handle_t panel = display_init();
    esp_lcd_panel_io_handle_t io = display_get_io_handle();

    /* Add display to LVGL port */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io, // Use initialized IO handle
        .panel_handle = panel, // Use initialized panel handle
        .buffer_size = LCD_WIDTH * LCD_HEIGHT,
        .double_buffer = true,
        .hres = LCD_WIDTH,
        .vres = LCD_HEIGHT,
        .monochrome = false,
        .rotation = {
            .swap_xy = true,   // rotate 90° counter-clockwise
            .mirror_x = false,
            .mirror_y = true,
        }
    };
    lvgl_port_add_disp(&disp_cfg);


    display_lvgl_init();
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

    // Init Screen
    init_screen_wrapper();

    i2c_master_init();
    // Initialize Wi-Fi
    printf("Starting WiFi...\n");
    if (wifi_init() == ESP_OK) {
        // Start mDNS
        printf("Starting mDNS...\n");
        start_mdns_service();
        // Start HTTPS server
        printf("Starting https...\n");
        start_https_server();
        printf("Starting sntp...\n");
        // Initialize SNTP and wait for time synchronization
        initialize_sntp();
    } else {
        ESP_LOGE("MAIN", "Failed to initialize Wi-Fi");
    }

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 40;
    while (time(&now) < 1000000000 && retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry+1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(250));
        retry++;
    }
    // Set timezone to US Eastern (Boston) and update local time info
    set_timezone_to_boston();
    localtime_r(&now, &timeinfo);
    ESP_LOGW(TAG, "Time is set: %s", asctime(&timeinfo));

    vTaskDelay(100/portTICK_PERIOD_MS);

    // Initialize UART console
    uart_comm_init();

    xTaskCreate(uart_console_task, "uart_console_task", 8192, NULL, 5, NULL);

    actuator_control_init();

    vTaskSuspend(NULL); // DEBUG HALT

    init_schedule_manager();


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

    // // Initialize and start flowmeter task
    // if (flowmeter_init() == ESP_OK) {
    //     xTaskCreate(flowmeter_task, "flowmeter_task", 4096, NULL, 5, NULL);
    // } else {
    //     ESP_LOGE("MAIN", "Failed to initialize flowmeter");
    // }

    // Initialize control logic
    ret = control_logic_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize control logic module");
    }
}
