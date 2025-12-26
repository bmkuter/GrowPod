// main/main.c

#include "Networking/wifi_manager.h"
#include "actuator_control.h"
#include "uart_comm.h"
#include "power_monitor_HAL.h"
#include "distance_sensor.h"
#include "https_server/https_server.h"
#include "sensors/sensor_manager.h"  // NEW: Sensor manager
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mdns.h"
#include "control_logic.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "display.h"
#include "esp_lvgl_port.h"
#include "pod_state.h"
#include "filesystem/filesystem_manager.h"
#include "filesystem/config_manager.h"

#define ACTUATOR_TASK_SIZE (1024 * 4) // 4 KB stack size for actuator task
#define UI_TASK_SIZE (1024 * 4) // 4 KB stack size for UI task

static const char *TAG = "APP_MAIN";

void start_mdns_service(void) {
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    
    char customSuffix[64] = "UNMODIFIED";
    esp_err_t err = config_load_mdns_suffix(customSuffix, sizeof(customSuffix));
    if (err != ESP_OK) {
        ESP_LOGW("MDNS", "Failed to load mDNS suffix, using default: %s", customSuffix);
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
        /* Initialize LVGL port with more reasonable timer period */
    lvgl_port_init(&((const lvgl_port_cfg_t) { .task_priority = 4, .task_stack = 4096, .timer_period_ms = 20 }));

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

void log_memory_stats(void)
{
    // Default heap (byte-addressable DRAM + PSRAM)
    size_t free_default = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    // Internal DRAM only
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    // External PSRAM only
    size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    // Largest contiguous free block in all DRAM heaps
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    // Minimum ever free heap since boot
    size_t min_ever = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "Free heap (default)      : %u bytes", free_default);
    ESP_LOGI(TAG, "Free internal DRAM      : %u bytes", free_internal);
    ESP_LOGI(TAG, "Free PSRAM              : %u bytes", free_spiram);
    ESP_LOGI(TAG, "Largest free block      : %u bytes", largest_block);
    ESP_LOGI(TAG, "Minimum ever free heap  : %u bytes", min_ever);

    // Print a detailed breakdown of each heap region
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
}

void wait_for_time_sync() {
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 100;
    while (time(&now) < 1000000000 && retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry+1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(250));
        retry++;
    }

    // Set timezone to US Eastern (Boston) and update local time info
    set_timezone_to_boston();
    localtime_r(&now, &timeinfo);
    ESP_LOGW(TAG, "Time is set: %s", asctime(&timeinfo));
}

void app_main(void) {
    vTaskDelay(500/portTICK_PERIOD_MS);

//Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      (nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGW(TAG, "NVS Status: %d", ret);

// Initialize LittleFS filesystem
    ESP_LOGI(TAG, "Initializing filesystem...");
    ret = filesystem_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Filesystem initialized successfully");
        
        // Run filesystem test
        ESP_LOGI(TAG, "Running filesystem test...");
        ret = filesystem_test();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Filesystem test completed successfully");
        } else {
            ESP_LOGE(TAG, "Filesystem test failed");
        }
        
        // Initialize configuration manager
        ESP_LOGI(TAG, "Initializing configuration manager...");
        ret = config_manager_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Configuration manager initialized successfully");
        } else {
            ESP_LOGE(TAG, "Failed to initialize configuration manager");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize filesystem");
    }

// Initialize sensor manager (centralized sensor reading system)
    // Note: I2C initialization is now handled by sensor_manager_init()
    ESP_LOGI(TAG, "Initializing sensor manager...");
    sensor_manager_config_t sensor_config = sensor_manager_get_default_config();
    ret = sensor_manager_init(&sensor_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor manager initialized successfully");
        
        // Start sensor manager task
        ret = sensor_manager_start();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Sensor manager task started");
        } else {
            ESP_LOGE(TAG, "Failed to start sensor manager task");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize sensor manager");
    }
    
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

// Setting NTP time
     wait_for_time_sync();

    vTaskDelay(100/portTICK_PERIOD_MS);

// Initialize UART console
    uart_comm_init();

    xTaskCreatePinnedToCore(uart_console_task, "uart_console_task", UI_TASK_SIZE, NULL, 5, NULL, 0);

// Init Screen
    init_screen_wrapper();

    actuator_control_init();

    init_schedule_manager();

// Start schedule manager task
    ESP_LOGI(TAG, "Starting schedule manager task");
    xTaskCreatePinnedToCore(schedule_manager_task, "schedule_manager", ACTUATOR_TASK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL, 0);

// Start LED schedule task
    ESP_LOGI(TAG, "Starting LED schedule task");
    xTaskCreatePinnedToCore(schedule_led_task, "schedule_led", ACTUATOR_TASK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL, 0);

// Start planter schedule task
    ESP_LOGI(TAG, "Starting planter schedule task");
    xTaskCreatePinnedToCore(schedule_planter_task, "schedule_planter", ACTUATOR_TASK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL, 0);

    // ret = distance_sensor_init();
    // if (ret == ESP_OK) {
    //     xTaskCreate(distance_sensor_task, "distance_sensor_task", (1024 * 8), NULL, 5, NULL);
    // } else {
    //     ESP_LOGE("MAIN", "distance_sensor_init failed: %s", esp_err_to_name(ret));
    // }
    
    // Note: Power monitor is now initialized by sensor_manager_init()
    // which was called earlier in app_main()

    vTaskDelay(1000/portTICK_PERIOD_MS);


    // Initialize actuators
    xTaskCreatePinnedToCore(actuator_control_task, "actuator_control_task", 4096, NULL, 5, NULL, 0);

    log_memory_stats();
}
