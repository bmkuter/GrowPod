# CMakeLists.txt

idf_component_register(
    SRCS
        "main.c"
        "ina260/ina260.c"
        "actuator_control/actuator_control.c"
        "uart_comm/uart_comm.c"
        "sensors/flowmeter_control.c"
        "sensors/distance_sensor.c"
        "https_server/https_server.c"
        "Networking/wifi_manager.c"
        "control_logic/control_logic.c"
    INCLUDE_DIRS
        "."
        "ina260"
        "actuator_control"
        "uart_comm"
        "sensors"
        "https_server"
        "Networking"
        "control_logic"
    # REQUIRES
    #     esp_https_server   # Add this line
    #     esp_wifi           # Include other components as needed
    #     driver             # For peripherals like RMT
    #     esp_netif
    #     esp_timer
    #     # cjson              # If using cJSON library
    #     # esp_adc_cal        # If using ADC calibration
    #     nvs_flash          # For NVS flash
    #     esp_event          # For event handling
    #     freertos           # For FreeRTOS functions
    #     # Add other required components
)

# Embed server certificate and key
target_add_binary_data(${COMPONENT_TARGET} "certs/server.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/server.key" TEXT)

# Optionally, if using mutual TLS, embed the CA certificate (root_ca.crt)
target_add_binary_data(${COMPONENT_TARGET} "certs/root_ca.crt" TEXT)
