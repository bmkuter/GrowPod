#include "uart_comm.h"
#include "actuator_control.h"
#include "power_monitor_HAL.h"
#include "distance_sensor.h"
#include "https_server.h"   // For start_calibrate_pod_routine
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_dev.h"
#include <stdio.h>
#include <string.h>
#include "pod_state.h"
#include "i2c_motor_driver.h"  // For motor direction configuration
#include "filesystem/filesystem_manager.h"  // For filesystem operations

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
static int cmd_calibrate_pod(int argc, char **argv); 
static int cmd_confirm_level(int argc, char **argv);  // Command to confirm calibration level
static int cmd_schedule(int argc, char **argv);
static int cmd_showschedules(int argc, char **argv);
static int cmd_motor_direction(int argc, char **argv);  // Motor direction configuration command
static int cmd_motor_test(int argc, char **argv);       // Motor test command for calibration
static int cmd_foodpump(int argc, char **argv);         // Food pump control command
static int cmd_fs_list(int argc, char **argv);          // List filesystem contents
static int cmd_fs_cat(int argc, char **argv);           // Display file contents
static int cmd_fs_info(int argc, char **argv);          // Show filesystem info
static int cmd_fs_test(int argc, char **argv);          // Test filesystem operations

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
            vTaskDelay(pdMS_TO_TICKS(25));
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
// Example: cmd_read_sensors() but with power monitor (INA219 or INA260)
static int cmd_read_sensors(int argc, char **argv) {
    esp_err_t ret;
    float current_total, voltage_total, power_total;
    static bool initialized = false;

    // 1) Ensure I2C is initialized
    ret = i2c_master_init(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(ret));
        return 1;
    }

    // 2) Initialize the single INA260
    if (!initialized) {
        ESP_LOGI(TAG, "Initializing power monitor (INA219 or INA260)");
        power_monitor_init(POWER_MONITOR_CHIP_INA219, INA219_ADDRESS);
        initialized = true;
    }

    // 3) Read data from the power monitor
    ret  = power_monitor_read_current(&current_total);
    ret |= power_monitor_read_voltage(&voltage_total);
    ret |= power_monitor_read_power(&power_total);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read power sensor data: %s", esp_err_to_name(ret));
        // return 1;
    }
    else {
    // 4) Print aggregated results 
        printf("\n---------------------------------------\n");
        printf(" Power Monitor Reading:\n");
        printf("   Current: %.2f mA\n", current_total);
        printf("   Voltage: %.2f mV\n", voltage_total);
        printf("   Power:   %.2f mW\n", power_total);
        printf("---------------------------------------\n\n");
    }

    // 5) Water level (using non-blocking function) and percent full
    int dist_mm = distance_sensor_read_mm();
    printf("Water level: %d mm\n", dist_mm);
    
    int fill_percent = pod_state_calc_fill_percent_int(&s_pod_state);
    if (fill_percent >= 0) {
        printf("Percent full: %d%%\n", fill_percent);
    } else {
        printf("Percent full: Not calibrated\n");
    }

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
 * @brief Command handler for the 'foodpump' command.
 */
static int cmd_foodpump(int argc, char **argv) {
    if (argc < 2 || argc > 4) {
        printf("Usage:\n");
        printf("  foodpump <pwm_value>           - Set continuous PWM (0-100%%)\n");
        printf("  foodpump dose <duration_ms>    - Dose for specified milliseconds at 100%% speed\n");
        printf("  foodpump dose <duration_ms> <speed>  - Dose for specified milliseconds at custom speed\n");
        printf("Examples:\n");
        printf("  foodpump 50         - Run at 50%% speed continuously\n");
        printf("  foodpump 0          - Stop food pump\n");
        printf("  foodpump dose 1000  - Dose for 1 second at 100%% speed\n");
        printf("  foodpump dose 500 80 - Dose for 500ms at 80%% speed\n");
        return 1;
    }

    if (strcmp(argv[1], "dose") == 0) {
        // Dosing mode
        if (argc < 3) {
            printf("Error: Missing duration for dose command\n");
            return 1;
        }
        
        uint32_t duration_ms = (uint32_t)atoi(argv[2]);
        uint8_t speed = 100; // Default speed
        
        if (argc == 4) {
            int speed_arg = atoi(argv[3]);
            if (speed_arg < 0 || speed_arg > 100) {
                printf("Error: Speed must be between 0 and 100\n");
                return 1;
            }
            speed = (uint8_t)speed_arg;
        }
        
        if (duration_ms == 0) {
            printf("Error: Duration must be greater than 0\n");
            return 1;
        }
        
        printf("Dosing food pump: %lu ms at %u%% speed\n", (unsigned long)duration_ms, speed);
        dose_food_pump_ms(duration_ms, speed);
        printf("Food pump dosing completed\n");
        
    } else {
        // Continuous PWM mode
        int value = atoi(argv[1]);
        if (value < 0 || value > 100) {
            printf("Error: PWM value must be between 0 and 100\n");
            return 1;
        }

        actuator_command_t command = {
            .cmd_type = ACTUATOR_CMD_FOOD_PUMP_PWM,
            .value = (uint32_t)value
        };
        xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
        printf("Food pump PWM set to %d%%\n", value);
    }
    
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

/**
 * @brief Command handler for 'fill_pod' command.
 * Usage: fill_pod [target_mm]
 * @param argc argument count
 * @param argv argument values
 * @return 0 on success, non-zero on error
 */
static int cmd_fill_pod(int argc, char **argv)
{
    int target_mm = -1;
    if (argc == 2) {
        target_mm = atoi(argv[1]);
        if (target_mm < 0) {
            ESP_LOGW(TAG, "Invalid target_mm '%s'. Must be non-negative.", argv[1]);
            return 1;
        }
    } else if (argc > 2) {
        ESP_LOGW(TAG, "Usage: fill_pod [target_mm]");
        return 1;
    }
    start_fill_pod_routine(target_mm);
    return 0;
}

static int cmd_empty_pod(int argc, char **argv)
{
    int target_pct = -1; // default to empty completely

    if (argc > 1) {
        target_pct = atoi(argv[1]);
        printf("Emptying pod to %d%% remaining\n", target_pct);
    } else {
        printf("Emptying pod completely\n");
    }
    
    start_empty_pod_routine(target_pct);
    return 0;
}

// Command handler for 'calibrate_pod' command
static int cmd_calibrate_pod(int argc, char **argv)
{
    // no args
    start_calibrate_pod_routine();
    return 0;
}

/**
 * @brief Command handler for 'confirm_level' command.
 * Usage: confirm_level <mm>
 */
static int cmd_confirm_level(int argc, char **argv)
{
    if (argc != 2) {
        ESP_LOGW(TAG, "Usage: confirm_level <mm>");
        return 1;
    }
    int mm = atoi(argv[1]);
    if (mm < 0) {
        ESP_LOGW(TAG, "Invalid level '%s'. Must be non-negative.", argv[1]);
        return 1;
    }
    confirm_level(mm);
    ESP_LOGI(TAG, "Calibration level confirmed: %d mm", mm);
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

// Command handler: motor_direction
static int cmd_motor_direction(int argc, char **argv) {
    if (argc == 1) {
        // Show current motor direction settings
        motor_direction_config_t config;
        esp_err_t err = i2c_motor_get_direction_config(&config);
        if (err == ESP_OK) {
            printf("Current motor direction settings:\n");
            printf("  Motor 1 (Planter): %s\n", config.motor1_inverted ? "inverted" : "normal");
            printf("  Motor 2 (Food):    %s\n", config.motor2_inverted ? "inverted" : "normal");
            printf("  Motor 3 (Source):  %s\n", config.motor3_inverted ? "inverted" : "normal");
            printf("  Motor 4 (LED):     %s\n", config.motor4_inverted ? "inverted" : "normal");
        } else {
            printf("Error reading motor direction config: %s\n", esp_err_to_name(err));
        }
        return 0;
    }
    
    if (argc == 3) {
        // Set motor direction: motor_direction <motor_num> <normal|inverted>
        int motor_num = atoi(argv[1]);
        if (motor_num < 1 || motor_num > 4) {
            printf("Invalid motor number. Use 1-4 (1=Planter, 2=Food, 3=Source, 4=LED)\n");
            return 1;
        }
        
        bool inverted;
        if (strcmp(argv[2], "normal") == 0) {
            inverted = false;
        } else if (strcmp(argv[2], "inverted") == 0) {
            inverted = true;
        } else {
            printf("Invalid direction. Use 'normal' or 'inverted'\n");
            return 1;
        }
        
        esp_err_t err = i2c_motor_set_direction_invert(motor_num, inverted);
        if (err == ESP_OK) {
            printf("Motor %d direction set to: %s\n", motor_num, inverted ? "inverted" : "normal");
            
            // Save to NVS
            err = i2c_motor_save_direction_settings();
            if (err == ESP_OK) {
                printf("Motor direction settings saved to flash\n");
            } else {
                printf("Warning: Failed to save settings to flash: %s\n", esp_err_to_name(err));
            }
        } else {
            printf("Error setting motor direction: %s\n", esp_err_to_name(err));
        }
        return 0;
    }
    
    printf("Usage:\n");
    printf("  motor_direction                      - Show current settings\n");
    printf("  motor_direction <1-4> <normal|inverted> - Set motor direction\n");
    printf("Motor mapping: 1=Planter, 2=Food, 3=Source, 4=LED\n");
    return 1;
}

// Command handler: motor_test
static int cmd_motor_test(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: motor_test <motor_num> <forward|backward|stop>\n");
        printf("Motor mapping: 1=Planter, 2=Food, 3=Source, 4=LED\n");
        printf("Use this to test motor directions for calibration\n");
        return 1;
    }
    
    int motor_num = atoi(argv[1]);
    if (motor_num < 1 || motor_num > 4) {
        printf("Invalid motor number. Use 1-4 (1=Planter, 2=Food, 3=Source, 4=LED)\n");
        return 1;
    }
    
    uint8_t command;
    uint8_t speed = 50; // Default test speed
    
    if (strcmp(argv[2], "forward") == 0) {
        command = MOTOR_FORWARD;
        printf("Running motor %d forward at 50%% speed...\n", motor_num);
    } else if (strcmp(argv[2], "backward") == 0) {
        command = MOTOR_BACKWARD;
        printf("Running motor %d backward at 50%% speed...\n", motor_num);
    } else if (strcmp(argv[2], "stop") == 0) {
        command = MOTOR_RELEASE;
        speed = 0;
        printf("Stopping motor %d...\n", motor_num);
    } else {
        printf("Invalid command. Use 'forward', 'backward', or 'stop'\n");
        return 1;
    }
    
    esp_err_t err = i2c_motor_set(motor_num, speed, command);
    if (err == ESP_OK) {
        printf("Motor %d command executed successfully\n", motor_num);
        if (command != MOTOR_RELEASE) {
            printf("Note: Motor will keep running. Use 'motor_test %d stop' to stop it.\n", motor_num);
        }
    } else {
        printf("Error controlling motor: %s\n", esp_err_to_name(err));
    }
    
    return 0;
}

// Command handler: fs_list
static int cmd_fs_list(int argc, char **argv) {
    const char *path = LFS_MOUNT_POINT;
    
    if (argc == 2) {
        path = argv[1];
    }
    
    if (!filesystem_is_mounted()) {
        printf("Error: Filesystem not mounted\n");
        return 1;
    }
    
    esp_err_t err = filesystem_list_dir_recursive(path);
    if (err != ESP_OK) {
        printf("Error listing directory: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    return 0;
}

// Command handler: fs_cat
static int cmd_fs_cat(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: fs_cat <file_path>\n");
        printf("Example: fs_cat /lfs/config/motors.json\n");
        return 1;
    }
    
    if (!filesystem_is_mounted()) {
        printf("Error: Filesystem not mounted\n");
        return 1;
    }
    
    char *buffer = NULL;
    size_t size = 0;
    
    esp_err_t err = filesystem_read_file(argv[1], &buffer, &size);
    if (err != ESP_OK) {
        printf("Error reading file: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    printf("--- File: %s (%zu bytes) ---\n", argv[1], size);
    printf("%s", buffer);
    if (size > 0 && buffer[size - 1] != '\n') {
        printf("\n");  // Add newline if file doesn't end with one
    }
    printf("--- End of file ---\n");
    
    free(buffer);
    return 0;
}

// Command handler: fs_info
static int cmd_fs_info(int argc, char **argv) {
    if (!filesystem_is_mounted()) {
        printf("Filesystem Status: NOT MOUNTED\n");
        return 1;
    }
    
    printf("Filesystem Status: MOUNTED\n");
    printf("Mount Point: %s\n", LFS_MOUNT_POINT);
    
    size_t total = 0, used = 0;
    esp_err_t err = filesystem_get_usage(&total, &used);
    if (err == ESP_OK) {
        printf("Total Size: %d bytes (%.2f KB)\n", total, total / 1024.0);
        printf("Used Space: %d bytes (%.2f KB)\n", used, used / 1024.0);
        printf("Free Space: %d bytes (%.2f KB)\n", total - used, (total - used) / 1024.0);
        printf("Usage: %.1f%%\n", 100.0 * used / total);
    } else {
        printf("Error getting filesystem info: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    return 0;
}

// Command handler: fs_test
static int cmd_fs_test(int argc, char **argv) {
    if (!filesystem_is_mounted()) {
        printf("Error: Filesystem not mounted\n");
        return 1;
    }
    
    printf("Running filesystem test...\n");
    esp_err_t err = filesystem_test();
    if (err == ESP_OK) {
        printf("Filesystem test completed successfully!\n");
    } else {
        printf("Filesystem test failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    
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
            .help = "Set drain pump PWM value (0-100) - Future expansion",
            .hint = NULL,
            .func = &cmd_drainpump,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Food pump command
    {
        const esp_console_cmd_t cmd = {
            .command = "foodpump",
            .help = "Control food pump: PWM or timed dosing",
            .hint = "<pwm_value> | dose <duration_ms> [speed]",
            .func = &cmd_foodpump,
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
            .help = "Read and display power monitor data (INA219 or INA260)",
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
            .help    = "Start fill_pod routine (optional target_mm)",
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
            .help    = "Start empty_pod routine with optional target percentage (0-100). Usage: empty_pod [percent]",
            .hint    = NULL,
            .func    = &cmd_empty_pod,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // calibrate_pod routine
    {
        const esp_console_cmd_t cmd = {
            .command = "calibrate_pod",
            .help    = "Calibrate the pod (fill and empty)",
            .hint    = NULL,
            .func    = &cmd_calibrate_pod,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // confirm_level command for calibration feedback
    {
        const esp_console_cmd_t cmd = {
            .command = "confirm_level",
            .help    = "Confirm measured water level during calibration (mm)",
            .hint    = NULL,
            .func    = &cmd_confirm_level,
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

    // Motor direction configuration command
    {
        const esp_console_cmd_t cmd = {
            .command = "motor_direction",
            .help    = "Configure motor direction inversion settings",
            .hint    = "[motor_num] [normal|inverted]",
            .func    = &cmd_motor_direction,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Motor test command for calibration
    {
        const esp_console_cmd_t cmd = {
            .command = "motor_test",
            .help    = "Test motor direction for calibration",
            .hint    = "<motor_num> <forward|backward|stop>",
            .func    = &cmd_motor_test,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Filesystem list command
    {
        const esp_console_cmd_t cmd = {
            .command = "fs_list",
            .help    = "List files recursively with full paths",
            .hint    = "[path]",
            .func    = &cmd_fs_list,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Filesystem cat command
    {
        const esp_console_cmd_t cmd = {
            .command = "fs_cat",
            .help    = "Display file contents",
            .hint    = "<file_path>",
            .func    = &cmd_fs_cat,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Filesystem info command
    {
        const esp_console_cmd_t cmd = {
            .command = "fs_info",
            .help    = "Show filesystem usage information",
            .hint    = NULL,
            .func    = &cmd_fs_info,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Filesystem test command
    {
        const esp_console_cmd_t cmd = {
            .command = "fs_test",
            .help    = "Run filesystem read/write test",
            .hint    = NULL,
            .func    = &cmd_fs_test,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}
