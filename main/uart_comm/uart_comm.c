#include "uart_comm.h"
#include "actuator_control.h"
#include "power_monitor_HAL.h"
#include "distance_sensor.h"
#include "https_server.h"   // For start_calibrate_pod_routine
#include "sensors/sensor_api.h"  // For sensor manager API
#include "sensors/sensor_manager.h"  // For sensor manager debug functions
#include "sensors/i2c_scanner.h"  // For I2C bus scanner
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_dev.h"
#include "driver/uart_vfs.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pod_state.h"
#include "i2c_motor_driver.h"  // For motor direction configuration
#include "filesystem/filesystem_manager.h"  // For filesystem operations
#include "filesystem/config_manager.h"  // For plant info

static const char *TAG = "UART_COMM";

// Forward declarations for command handler functions
static int cmd_airpump(int argc, char **argv);
static int cmd_sourcepump(int argc, char **argv);
static int cmd_planterpump(int argc, char **argv);
static int cmd_drainpump(int argc, char **argv);
static int cmd_led(int argc, char **argv);
static int cmd_read_sensors(int argc, char **argv);
static int cmd_sensors(int argc, char **argv);          // NEW: Sensor manager test command
static int cmd_i2c_scan(int argc, char **argv);         // I2C bus scanner command
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
static int cmd_plant_set(int argc, char **argv);        // Set plant information
static int cmd_plant_info(int argc, char **argv);       // Display plant information
static int cmd_pwm_sweep(int argc, char **argv);        // PWM sweep test for motor shields

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
    uart_vfs_dev_use_driver(UART_PORT_NUM);

    // Set up the UART console line endings
    uart_vfs_dev_port_set_rx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_CRLF);

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
 * Updated to use centralized sensor manager API.
 */
static int cmd_read_sensors(int argc, char **argv) {
    esp_err_t ret;
    float current_mA, voltage_mV, power_mW;
    float temperature_c, humidity_rh;

    printf("\n=== Sensor Readings ===\n\n");

    // Read power monitor data through sensor manager
    ret = sensor_api_read_power_all(&current_mA, &voltage_mV, &power_mW);
    if (ret == ESP_OK) {
        printf("Power Monitor:\n");
        printf("  Current: %.2f mA\n", current_mA);
        printf("  Voltage: %.2f mV\n", voltage_mV);
        printf("  Power:   %.2f mW\n", power_mW);
    } else {
        printf("Power Monitor: Error reading (%s)\n", esp_err_to_name(ret));
    }
    printf("\n");

    // Read temperature and humidity through sensor manager (single read)
    ret = sensor_api_read_environment_all(&temperature_c, &humidity_rh);
    if (ret == ESP_OK) {
        printf("SHT45 Environment:\n");
        printf("  Temperature: %.2f °C\n", temperature_c);
        printf("  Humidity: %.1f %%RH\n", humidity_rh);
    } else {
        printf("SHT45 Environment: Error reading (%s)\n", esp_err_to_name(ret));
    }
    printf("\n");

    // Read light sensor data
    sensor_data_t light_data;
    ret = sensor_manager_get_data_cached(SENSOR_TYPE_LIGHT, &light_data, NULL);
    if (ret == ESP_OK) {
        printf("TSL2591 Light Sensor:\n");
        printf("  Illuminance: %.2f lux\n", light_data.light.lux);
        printf("  Visible: %u counts\n", light_data.light.visible);
        printf("  Infrared: %u counts\n", light_data.light.infrared);
    } else {
        printf("TSL2591 Light Sensor: Error reading (%s)\n", esp_err_to_name(ret));
    }
    printf("\n");

    // Water level (using non-blocking function) and percent full
    int dist_mm = distance_sensor_read_mm();
    printf("Water Level: %d mm\n", dist_mm);
    
    int fill_percent = pod_state_calc_fill_percent_int(&s_pod_state);
    if (fill_percent >= 0) {
        printf("Percent Full: %d%%\n", fill_percent);
    } else {
        printf("Percent Full: Not calibrated\n");
    }

    printf("\n======================\n\n");

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
 * Supports two formats:
 * 1. led <value>           - Sets all LEDs (legacy format)
 * 2. led <channel> <value> - Sets specific LED channel (1-4)
 */
static int cmd_led(int argc, char **argv) {
    actuator_command_t command;
    
    if (argc == 2) {
        // Legacy format: led <value>
        // Sets all LEDs (backward compatible)
        int value = atoi(argv[1]);
        
        // Validate range
        if (value < 0 || value > 100) {
            ESP_LOGW(TAG, "Invalid value. Must be between 0 and 100.");
            return 1;
        }

        command.cmd_type = ACTUATOR_CMD_LED_ARRAY_PWM;
        command.value = (uint32_t)value;
        command.channel = 0; // not used
        
        xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
        ESP_LOGI(TAG, "LED array set to %d%%", value);
        return 0;
        
    } else if (argc == 3) {
        // New format: led <channel> <value>
        int channel = atoi(argv[1]);
        int value = atoi(argv[2]);
        
        // Validate channel (1-4)
        if (channel < 1 || channel > 4) {
            ESP_LOGW(TAG, "Invalid channel. Must be between 1 and 4.");
            return 1;
        }
        
        // Validate value (0-100)
        if (value < 0 || value > 100) {
            ESP_LOGW(TAG, "Invalid value. Must be between 0 and 100.");
            return 1;
        }

        command.cmd_type = ACTUATOR_CMD_LED_CHANNEL_PWM;
        command.channel = (uint8_t)channel;
        command.value = (uint32_t)value;
        
        xQueueSend(get_actuator_queue(), &command, portMAX_DELAY);
        ESP_LOGI(TAG, "LED channel %d set to %d%%", channel, value);
        return 0;
        
    } else {
        // Invalid argument count
        ESP_LOGW(TAG, "Usage:");
        ESP_LOGW(TAG, "  led <value>           - Set all LEDs (0-100%%)");
        ESP_LOGW(TAG, "  led <channel> <value> - Set specific LED channel");
        ESP_LOGW(TAG, "Examples:");
        ESP_LOGW(TAG, "  led 100       - All LEDs to full brightness");
        ESP_LOGW(TAG, "  led 1 50      - Channel 1 to 50%% brightness");
        ESP_LOGW(TAG, "  led 4 100     - Channel 4 to full brightness");
        return 1;
    }
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

// Command handler: plant_set
static int cmd_plant_set(int argc, char **argv) {
    if (argc < 3) {
        printf("Usage: plant_set <plant_name> <start_date>\n");
        printf("  plant_name: Name of the plant (e.g., \"Tomato\")\n");
        printf("  start_date: Start date in YYYY-MM-DD format\n");
        printf("Example: plant_set \"Cherry Tomato\" 2025-11-22\n");
        return 1;
    }
    
    if (!filesystem_is_mounted()) {
        printf("Error: Filesystem not mounted\n");
        return 1;
    }
    
    plant_info_t info;
    strncpy(info.plant_name, argv[1], sizeof(info.plant_name) - 1);
    info.plant_name[sizeof(info.plant_name) - 1] = '\0';
    
    strncpy(info.start_date, argv[2], sizeof(info.start_date) - 1);
    info.start_date[sizeof(info.start_date) - 1] = '\0';
    
    // Get current timestamp
    time_t now;
    time(&now);
    info.start_timestamp = (int32_t)now;
    
    esp_err_t err = config_save_plant_info(&info);
    if (err != ESP_OK) {
        printf("Error saving plant info: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    printf("Plant information saved:\n");
    printf("  Name: %s\n", info.plant_name);
    printf("  Start Date: %s\n", info.start_date);
    printf("  Timestamp: %ld\n", (long)info.start_timestamp);
    
    return 0;
}

// Command handler: plant_info
static int cmd_plant_info(int argc, char **argv) {
    if (!filesystem_is_mounted()) {
        printf("Error: Filesystem not mounted\n");
        return 1;
    }
    
    plant_info_t info;
    esp_err_t err = config_load_plant_info(&info);
    
    if (err == ESP_ERR_NOT_FOUND) {
        printf("No plant information set.\n");
        printf("Use 'plant_set <name> <date>' to set plant info.\n");
        return 0;
    } else if (err != ESP_OK) {
        printf("Error loading plant info: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    printf("Current Plant Information:\n");
    printf("  Plant Name: %s\n", info.plant_name);
    printf("  Start Date: %s\n", info.start_date);
    printf("  Start Timestamp: %ld\n", (long)info.start_timestamp);
    
    int32_t days = config_get_days_growing(&info);
    if (days >= 0) {
        printf("  Days Growing: %ld\n", (long)days);
    } else {
        printf("  Days Growing: Invalid (check system time)\n");
    }
    
    return 0;
}

// Command handler: pwm_sweep
static int cmd_pwm_sweep(int argc, char **argv) {
    if (argc != 2) {
        printf("Usage: pwm_sweep <motor|led>\n");
        printf("  motor - Test pump motor shield (0x60)\n");
        printf("  led   - Test LED motor shield (0x61)\n");
        return 1;
    }
    
    uint8_t shield_addr;
    const char *shield_type = argv[1];
    
    if (strcmp(shield_type, "motor") == 0) {
        shield_addr = MOTOR_SHIELD_PUMP_ADDR;
        printf("Starting PWM sweep on pump motor shield (0x%02x)...\n", shield_addr);
    } else if (strcmp(shield_type, "led") == 0) {
        shield_addr = MOTOR_SHIELD_LED_ADDR;
        printf("Starting PWM sweep on LED motor shield (0x%02x)...\n", shield_addr);
    } else {
        printf("Error: Invalid shield type '%s'\n", shield_type);
        printf("Valid options: motor, led\n");
        return 1;
    }
    
    printf("Each channel will sweep from 0%% to 100%% and back over 2 seconds.\n");
    printf("Total test time: ~8 seconds (4 channels x 2 seconds)\n\n");
    
    esp_err_t err = i2c_motor_pwm_sweep(shield_addr);
    
    if (err != ESP_OK) {
        printf("Error during PWM sweep: %s\n", esp_err_to_name(err));
        return 1;
    }
    
    printf("\nPWM sweep completed successfully!\n");
    return 0;
}

/**
 * @brief Command handler for the 'sensors' command - demonstrates sensor manager
 */
static int cmd_sensors(int argc, char **argv) {
    ESP_LOGI(TAG, "=== SENSOR MANAGER TEST COMMAND ===");
    ESP_LOGI(TAG, "Reading all sensors through sensor manager API...");
    printf("\n");
    
    // This function will show all the logging and tracing
    sensor_api_print_all();
    
    printf("\n");
    ESP_LOGI(TAG, "Sensor manager internal state:");
    sensor_manager_print_debug_info();
    
    return 0;
}

/**
 * @brief Command handler for the 'i2c_scan' command
 * Scans the I2C bus and displays all detected devices
 */
static int cmd_i2c_scan(int argc, char **argv) {
    printf("\n=== I2C Bus Scanner ===\n");
    printf("Scanning I2C bus for connected devices...\n\n");
    
    // Use I2C_NUM_0 which is the default I2C port used by sensors
    i2c_scanner_scan(I2C_NUM_0);
    
    printf("\nCommon I2C addresses:\n");
    printf("  0x29 - TSL2591 (Light sensor)\n");
    printf("  0x40 - INA219/INA260 (Power monitor)\n");
    printf("  0x44 - SHT45 (Temperature/Humidity)\n");
    printf("  0x60 - PCA9685 (Motor shield 1)\n");
    printf("  0x61 - PCA9685 (Motor shield 2)\n");
    printf("  0x0B - LC709203F (Battery gauge)\n\n");
    
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

    // Sensors command (new sensor manager API)
    {
        const esp_console_cmd_t cmd = {
            .command = "sensors",
            .help = "Read all sensors via sensor manager (with detailed logging)",
            .hint = NULL,
            .func = &cmd_sensors,
            .argtable = NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // I2C Scanner command
    {
        const esp_console_cmd_t cmd = {
            .command = "i2c_scan",
            .help = "Scan I2C bus and display all detected devices",
            .hint = NULL,
            .func = &cmd_i2c_scan,
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

    // Plant set command
    {
        const esp_console_cmd_t cmd = {
            .command = "plant_set",
            .help    = "Set plant name and start date",
            .hint    = "<plant_name> <YYYY-MM-DD>",
            .func    = &cmd_plant_set,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // Plant info command
    {
        const esp_console_cmd_t cmd = {
            .command = "plant_info",
            .help    = "Display current plant information",
            .hint    = NULL,
            .func    = &cmd_plant_info,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }

    // PWM sweep command
    {
        const esp_console_cmd_t cmd = {
            .command = "pwm_sweep",
            .help    = "Perform PWM sweep test on motor shield",
            .hint    = "<motor|led>",
            .func    = &cmd_pwm_sweep,
            .argtable= NULL
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}
