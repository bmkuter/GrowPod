// main/main.c

#include "Networking/wifi_manager.h"
#include "actuator_control.h"
#include "uart_comm.h"
#include "ina260.h"
#include "flowmeter_control.h"
#include "https_server/https_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mdns.h"

void start_mdns_service(void) {
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("hydroponics_device"));
    ESP_ERROR_CHECK(mdns_instance_name_set("Hydroponics Device"));

    // Set service type and port
    mdns_service_add("Hydroponics Service", "_hydroponics", "_tcp", 443, NULL, 0);

    ESP_LOGI("MDNS", "mDNS service started");
}

void app_main(void) {
    // Initialize Wi-Fi
    if (wifi_init() == ESP_OK) {
        // Start mDNS
        start_mdns_service();
        // Start HTTPS server
        start_https_server();
    } else {
        ESP_LOGE("MAIN", "Failed to initialize Wi-Fi");
    }

    // Initialize actuators
    actuator_control_init();
    xTaskCreate(actuator_control_task, "actuator_control_task", 4096, NULL, 5, NULL);

    // Initialize UART console
    uart_comm_init();
    xTaskCreate(uart_console_task, "uart_console_task", 8192, NULL, 5, NULL);

    // Start INA260 task
    xTaskCreate(ina260_task, "ina260_task", 4096, NULL, 5, NULL);

    // Initialize and start flowmeter task
    if (flowmeter_init() == ESP_OK) {
        xTaskCreate(flowmeter_task, "flowmeter_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE("MAIN", "Failed to initialize flowmeter");
    }

    // Rest of your application...
}
