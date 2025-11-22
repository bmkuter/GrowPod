# CMakeLists.txt

idf_component_register(
    SRCS
        "main.c"
        "ina_power_monitor/ina260.c"
        "ina_power_monitor/ina219.c"
        "ina_power_monitor/power_monitor_HAL.c"
        "actuator_control/actuator_control.c"
        "actuator_control/i2c_motor_driver.c"
        "uart_comm/uart_comm.c"
        "sensors/flowmeter_control.c"
        "sensors/distance_sensor.c"
        "sensors/contact_sensor.c"
        "https_server/https_server.c"
        "Networking/wifi_manager.c"
        "control_logic/control_logic.c"
        "display/display.c"
        "pod_state/pod_state.c"
    INCLUDE_DIRS
        "."
        "ina_power_monitor"
        "actuator_control"
        "uart_comm"
        "sensors"
        "https_server"
        "Networking"
        "control_logic"
        "display"
        "pod_state"
    # REQUIRES
    #     esp_vfs_dev      # VFS driver for UART/USB Serial JTAG
    #     driver           # GPIO, LEDC, UART, etc.
    #     argtable3        # Argument parsing
    #     linenoise        # Line editing support
    #     nvs_flash        # NVS storage
    #     esp_event        # Event loop
    #     freertos         # FreeRTOS
    #     esp_netif        # Network interface
    #     esp_wifi         # Wi-Fi
    #     mdns             # mDNS
    #     esp_sntp         # SNTP
    #     esp_https_server # HTTPS server
)

# Embed server certificate and key
target_add_binary_data(${COMPONENT_TARGET} "certs/server.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/server.key" TEXT)

# Optionally, if using mutual TLS, embed the CA certificate (root_ca.crt)
target_add_binary_data(${COMPONENT_TARGET} "certs/root_ca.crt" TEXT)
