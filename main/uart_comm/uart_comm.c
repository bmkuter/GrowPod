#include "uart_comm.h"
#include "actuator_control.h"
#include "ina260.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_dev.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "UART_COMM";

// Forward declarations for command handler functions
static int cmd_airpump(int argc, char **argv);
static int cmd_waterpump(int argc, char **argv);
static int cmd_solenoid(int argc, char **argv);
static int cmd_led(int argc, char **argv);
static int cmd_read_sensors(int argc, char **argv);

// Function to register console commands
static void register_console_commands(void);

/**
 * @brief Initialize UART communication.
 */
void uart_comm_init(void) {
    ESP_LOGI(TAG, "Initializing UART communication");

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk= UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART communication initialized successfully");
}

/**
 * @brief Console task that handles input and executes commands.
 */
void uart_console_task(void *pvParameter) {
    // Configure the console
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    // Configure linenoise line completion library
    linenoiseSetMultiLine(1);
    linenoiseHistorySetMaxLen(100);
    linenoiseAllowEmpty(false);

    // Install UART driver for interrupt-driven reads and writes
    esp_vfs_dev_uart_use_driver(UART_PORT_NUM);

    // Set up the UART console
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_CRLF);

    // Initialize the readline environment
    esp_console_register_help_command();
    register_console_commands();

    ESP_LOGI(TAG, "Console started. Type 'help' to get the list of commands.");

    while (1) {
        char* line = linenoise("hydroponics> ");
        if (line == NULL) { // EOF or error
            continue;
        }

        // Add the command to history
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
        }

        // Try to run the command
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "Unrecognized command");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // Command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            ESP_LOGW(TAG, "Command returned non-zero error code: 0x%x (%s)", ret, esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Internal error: %s", esp_err_to_name(err));
        }

        // Free the line buffer
        linenoiseFree(line);
    }
}

/**
 * @brief Register console commands.
 */
static void register_console_commands(void) {
    // Air pump command
    {
        const esp_console_cmd_t cmd = {
            .command = "airpump",
            .help = "Set air pump PWM value (0-100)",
            .hint = NULL,
            .func = &cmd_airpump,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Water pump command
    {
        const esp_console_cmd_t cmd = {
            .command = "waterpump",
            .help = "Set water pump PWM value (0-100)",
            .hint = NULL,
            .func = &cmd_waterpump,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Solenoid valve command
    {
        const esp_console_cmd_t cmd = {
            .command = "solenoid",
            .help = "Set solenoid valve state (0 or 1)",
            .hint = NULL,
            .func = &cmd_solenoid,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // LED array command
    {
        const esp_console_cmd_t cmd = {
            .command = "led",
            .help = "Set LED array state (0 or 1)",
            .hint = NULL,
            .func = &cmd_led,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Read sensors command
    {
        const esp_console_cmd_t cmd = {
            .command = "read_sensors",
            .help = "Read and display INA260 sensor data",
            .hint = NULL,
            .func = &cmd_read_sensors,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}

/**
 * @brief Command handler for the 'read_sensors' command.
 */
static int cmd_read_sensors(int argc, char **argv) {
    esp_err_t ret;
    float current_led, voltage_led, power_led;
    float current_drain, voltage_drain, power_drain;
    float current_source, voltage_source, power_source;
    float current_air, voltage_air, power_air;

    // Initialize I2C master
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master: %s", esp_err_to_name(ret));
        return 1;
    }

    // Initialize INA260 sensors
    ina260_init(INA260_LED_ADDRESS);
    ina260_init(INA260_DRAIN_ADDRESS);
    ina260_init(INA260_SOURCE_ADDRESS);
    ina260_init(INA260_AIR_ADDRESS);

    // Read data from sensors
    ret = ina260_read_current(INA260_LED_ADDRESS, &current_led);
    ret |= ina260_read_voltage(INA260_LED_ADDRESS, &voltage_led);
    ret |= ina260_read_power(INA260_LED_ADDRESS, &power_led);

    ret |= ina260_read_current(INA260_DRAIN_ADDRESS, &current_drain);
    ret |= ina260_read_voltage(INA260_DRAIN_ADDRESS, &voltage_drain);
    ret |= ina260_read_power(INA260_DRAIN_ADDRESS, &power_drain);

    ret |= ina260_read_current(INA260_SOURCE_ADDRESS, &current_source);
    ret |= ina260_read_voltage(INA260_SOURCE_ADDRESS, &voltage_source);
    ret |= ina260_read_power(INA260_SOURCE_ADDRESS, &power_source);

    ret |= ina260_read_current(INA260_AIR_ADDRESS, &current_air);
    ret |= ina260_read_voltage(INA260_AIR_ADDRESS, &voltage_air);
    ret |= ina260_read_power(INA260_AIR_ADDRESS, &power_air);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return 1;
    }

    // Print the results in a row-column view
    printf("\n---------------------------------------------------------\n");
    printf("| Lane   | Current (mA) | Voltage (mV) | Power (mW)      |\n");
    printf("---------------------------------------------------------\n");
    printf("| LED    | %12.2f | %12.2f | %12.2f   |\n", current_led, voltage_led, power_led);
    printf("| Drain  | %12.2f | %12.2f | %12.2f   |\n", current_drain, voltage_drain, power_drain);
    printf("| Source | %12.2f | %12.2f | %12.2f   |\n", current_source, voltage_source, power_source);
    printf("| Air    | %12.2f | %12.2f | %12.2f   |\n", current_air, voltage_air, power_air);
    printf("---------------------------------------------------------\n");

    return 0;
}

/**
 * @brief Command handler for the 'airpump' command.
 */
static int cmd_airpump(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: airpump <value>");
        return 1;
    }

    int value = atoi(argv[1]);
    if (value < 0 || value > 100) {
        ESP_LOGW(TAG, "Invalid value. Please provide a PWM value between 0 and 100.");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_AIR_PUMP_PWM,
        .value = value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "Air pump PWM set to %d", value);
    return 0;
}

/**
 * @brief Command handler for the 'waterpump' command.
 */
static int cmd_waterpump(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: waterpump <value>");
        return 1;
    }

    int value = atoi(argv[1]);
    if (value < 0 || value > 100) {
        ESP_LOGW(TAG, "Invalid value. Please provide a PWM value between 0 and 100.");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_WATER_PUMP_PWM,
        .value = value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "Water pump PWM set to %d", value);
    return 0;
}

/**
 * @brief Command handler for the 'solenoid' command.
 */
static int cmd_solenoid(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: solenoid <0|1>");
        return 1;
    }

    int value = atoi(argv[1]);
    if (value != 0 && value != 1) {
        ESP_LOGW(TAG, "Invalid value. Use 0 (off) or 1 (on).");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_SOLENOID_VALVE,
        .value = value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "Solenoid valve set to %d", value);
    return 0;
}

/**
 * @brief Command handler for the 'led' command.
 */
static int cmd_led(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: led <0|1>");
        return 1;
    }

    int value = atoi(argv[1]);
    if (value != 0 && value != 1) {
        ESP_LOGW(TAG, "Invalid value. Use 0 (off) or 1 (on).");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_LED_ARRAY_BINARY,
        .value = value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "LED array set to %d", value);
    return 0;
}
