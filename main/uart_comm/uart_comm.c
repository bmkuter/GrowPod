#include "uart_comm.h"
#include "actuator_control.h"
#include "ina260.h"
#include "distance_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_dev.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "UART_COMM";

// Forward declarations for command handler functions
static int cmd_airpump(int argc, char **argv);
static int cmd_sourcepump(int argc, char **argv);
static int cmd_planterpump(int argc, char **argv);
static int cmd_drainpump(int argc, char **argv);
static int cmd_led(int argc, char **argv);
static int cmd_read_sensors(int argc, char **argv);
static int cmd_fill_pod(int argc, char **argv);
static int cmd_empty_pod(int argc, char **argv);
static int cmd_schedule(int argc, char **argv);
static int cmd_showschedules(int argc, char **argv); // New forward declaration

// Function to register console commands
static void register_console_commands(void);

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

    // Send a test message to verify UART setup
    const char *test_msg = "UART communication initialized successfully.\r\n";
    uart_write_bytes(UART_PORT_NUM, test_msg, strlen(test_msg));

    ESP_LOGI(TAG, "UART communication initialized successfully");
}

void uart_console_task(void *pvParameter) 
{
    vTaskDelay(2500 / portTICK_PERIOD_MS);

    // Configure the console
    esp_console_config_t console_config = {
        .max_cmdline_args   = 32,   // allow up to 32 args (schedule needs 26)
        .max_cmdline_length = 512,  // increase buffer for long lines
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    linenoiseSetMultiLine(1);
    linenoiseHistorySetMaxLen(100);
    linenoiseAllowEmpty(false);

    // Install UART driver for interrupt-driven reads/writes
    esp_vfs_dev_uart_use_driver(UART_PORT_NUM);

    // Set up the UART console line endings
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_CRLF);

    // Initialize the readline environment
    esp_console_register_help_command();
    register_console_commands();

    ESP_LOGI(TAG, "Console started. Type 'help' to get the list of commands.");

    while (1) {
        char* line = linenoise("hydroponics> ");
        if (line == NULL) { // EOF or error
            ESP_LOGW(TAG, "No input received, restart console task loop");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Add the command to history
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
        }

        // Execute the command
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
 * @brief Command handler for the 'read_sensors' command.
 */
// Example: cmd_read_sensors() but with only one INA260
static int cmd_read_sensors(int argc, char **argv) {
    esp_err_t ret;
    float current_total, voltage_total, power_total;

    // 1) Ensure I2C is initialized
    ret = i2c_master_init(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(ret));
        return 1;
    }

    // 2) Initialize the single INA260
    ina260_init(INA260_ADDRESS);

    // 3) Read data from the single sensor
    ret  = ina260_read_current(INA260_ADDRESS, &current_total);
    ret |= ina260_read_voltage(INA260_ADDRESS, &voltage_total);
    ret |= ina260_read_power(INA260_ADDRESS,   &power_total);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return 1;
    }

    // 4) Print aggregated results 
    printf("\n---------------------------------------\n");
    printf(" Single INA260 @ 0x%02X Reading:\n", INA260_ADDRESS);
    printf("   Current: %.2f mA\n", current_total);
    printf("   Voltage: %.2f mV\n", voltage_total);
    printf("   Power:   %.2f mW\n", power_total);
    printf("---------------------------------------\n\n");

    // 5) Water level
    printf("Water level: %d cm\n", distance_sensor_read_mm() / 10);

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
 * @brief Command handler for the 'sourcepump' command (formerly 'waterpump').
 */
static int cmd_sourcepump(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: sourcepump <value>");
        return 1;
    }

    int value = atoi(argv[1]);
    if (value < 0 || value > 100) {
        ESP_LOGW(TAG, "Invalid value. Please provide a PWM value between 0 and 100.");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_SOURCE_PUMP_PWM,
        .value = value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "Source pump PWM set to %d", value);
    return 0;
}

/**
 * @brief Command handler for the 'planterpump' command.
 */
static int cmd_planterpump(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: planterpump <value>");
        return 1;
    }

    int value = atoi(argv[1]);
    if (value < 0 || value > 100) {
        ESP_LOGW(TAG, "Invalid value. Please provide a PWM value between 0 and 100.");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_PLANTER_PUMP_PWM,
        .value = value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "Planter pump PWM set to %d", value);
    return 0;
}

/**
 * @brief Command handler for the 'drainpump' command (replacing servo).
 */
static int cmd_drainpump(int argc, char **argv)
{
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: drainpump <value>");
        ESP_LOGW(TAG, "Example: drainpump 100 (full duty)");
        return 1;
    }

    int value = atoi(argv[1]);
    // Validate range
    if (value < 0 || value > 100) {
        ESP_LOGW(TAG, "Invalid value. Must be between 0 and 100.");
        return 1;
    }

    // Create an actuator command for the drain pump
    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_DRAIN_PUMP_PWM,  // newly replaced command enum
        .value = (uint32_t)value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);

    ESP_LOGI(TAG, "Drain pump PWM set to %d", value);
    return 0;
}

/**
 * @brief Command handler for the 'led' command.
 */
static int cmd_led(int argc, char **argv) {
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: led <value>");
        ESP_LOGW(TAG, "Example: led 100 (full duty)");
        return 1;
    }

    int value = atoi(argv[1]);
    // Validate range
    if (value < 0 || value > 100) {
        ESP_LOGW(TAG, "Invalid value. Must be between 0 and 100.");
        return 1;
    }

    actuator_command_t command = {
        .cmd_type = ACTUATOR_CMD_LED_ARRAY_PWM,
        .value = (uint32_t)value
    };
    xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
    ESP_LOGI(TAG, "LED array set to %d", value);
    return 0;
}

static int cmd_fill_pod(int argc, char **argv)
{
    // no args
    start_fill_pod_routine();
    return 0;
}

static int cmd_empty_pod(int argc, char **argv)
{
    // no args
    start_empty_pod_routine();
    return 0;
}

static int cmd_schedule(int argc, char **argv) {
    if (argc != 26) {
        ESP_LOGW(TAG, "Usage: schedule <light|planter|air> v0 v1 ... v23");
        return 1;
    }
    uint8_t sched[24];
    for (int i = 0; i < 24; i++) {
        int v = atoi(argv[i+2]);
        if (v < 0) v = 0; else if (v > 100) v = 100;
        sched[i] = v;
    }
    if (strcmp(argv[1], "light") == 0) {
        start_light_schedule(sched);
    } else if (strcmp(argv[1], "planter") == 0) {
        start_planter_schedule(sched);
    } else if (strcmp(argv[1], "air") == 0) {
        start_air_schedule(sched);
    } else {
        ESP_LOGW(TAG, "Unknown schedule type '%s'", argv[1]);
        return 1;
    }
    ESP_LOGI(TAG, "Scheduled %s_schedule", argv[1]);
    return 0;
}

// Command handler: showschedules
static int cmd_showschedules(int argc, char **argv) {
    print_schedules();
    return 0;
}

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

    // Source pump command (replaces 'waterpump')
    {
        const esp_console_cmd_t cmd = {
            .command = "sourcepump",
            .help = "Set source pump PWM value (0-100)",
            .hint = NULL,
            .func = &cmd_sourcepump,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Planter pump command
    {
        const esp_console_cmd_t cmd = {
            .command = "planterpump",
            .help = "Set planter pump PWM value (0-100)",
            .hint = NULL,
            .func = &cmd_planterpump,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Drain pump command (replaces the old servo command)
    {
        const esp_console_cmd_t cmd = {
            .command = "drainpump",
            .help = "Set drain pump PWM value (0-100)",
            .hint = NULL,
            .func = &cmd_drainpump,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // LED array command
    {
        const esp_console_cmd_t cmd = {
            .command = "led",
            .help = "Set LED array PWM value (0-100)",
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

    // fill_pod routine
    {
        const esp_console_cmd_t cmd = {
            .command = "fill_pod",
            .help    = "Start fill_pod routine and log level each second",
            .hint    = NULL,
            .func    = &cmd_fill_pod,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // empty_pod routine
    {
        const esp_console_cmd_t cmd = {
            .command = "empty_pod",
            .help    = "Start empty_pod routine and log level each second",
            .hint    = NULL,
            .func    = &cmd_empty_pod,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // generic schedule routine
    {
        const esp_console_cmd_t cmd = {
            .command = "schedule",
            .help    = "Start a 24h schedule: schedule <light|planter|air> v0..v23",
            .hint    = NULL,
            .func    = &cmd_schedule,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
    // Show schedules command
    {
        const esp_console_cmd_t cmd = {
            .command = "showschedules",
            .help    = "Print current LED, planter and air schedules",
            .hint    = NULL,
            .func    = &cmd_showschedules,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}
