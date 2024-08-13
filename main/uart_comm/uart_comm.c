#include "uart_comm.h"
#include <string.h> // Include the header for strlen
#include "actuator_control.h"

static const char *TAG = "UART_COMM";

void uart_comm_init(void) {
    ESP_LOGI(TAG, "Initializing UART communication");

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_PORT_NUM, 16384, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART communication initialized successfully");
}

void uart_send_data(const char* data) {
    ESP_LOGI(TAG, "Sending data: %s", data);
    uart_write_bytes(UART_PORT_NUM, data, strlen(data));
}

void uart_receive_data(char* data, uint16_t max_len) {
    int len = uart_read_bytes(UART_PORT_NUM, (uint8_t*)data, max_len, pdMS_TO_TICKS(1000));
    data[len] = '\0'; // Null-terminate the string
    ESP_LOGI(TAG, "Received data: %s", data);
}

void uart_echo_task(void *pvParameter) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI("UART_COMM", "Starting UART echo task");

    char data[UART_BUF_SIZE];
    int idx = 0;

    QueueHandle_t actuator_queue = get_actuator_queue();
    actuator_command_t command;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, (uint8_t*)&data[idx], 1, portMAX_DELAY); // Wait indefinitely for input
        if (len > 0) {
            uart_write_bytes(UART_PORT_NUM, (const char *)&data[idx], 1); // Echo character

            if (data[idx] == '\r' || data[idx] == '\n') {  // Detect 'Enter' key (newline)
                data[idx] = '\0'; // Null-terminate the string
                ESP_LOGI("UART_COMM", "Received command: %s", data);

                // Simple command parser (for demonstration purposes)
                if (strncmp(data, "airpump ", 8) == 0) {
                    command.cmd_type = ACTUATOR_CMD_AIR_PUMP_PWM;
                    command.value = atoi(&data[8]);
                    xQueueSend(actuator_queue, &command, portMAX_DELAY);
                } else if (strncmp(data, "waterpump ", 10) == 0) {
                    command.cmd_type = ACTUATOR_CMD_WATER_PUMP_PWM;
                    command.value = atoi(&data[10]);
                    xQueueSend(actuator_queue, &command, portMAX_DELAY);
                } else if (strncmp(data, "solenoid ", 9) == 0) {
                    command.cmd_type = ACTUATOR_CMD_SOLENOID_VALVE;
                    command.value = (data[9] == '1') ? 1 : 0;
                    xQueueSend(actuator_queue, &command, portMAX_DELAY);
                } else if (strncmp(data, "led ", 4) == 0) {
                    command.cmd_type = ACTUATOR_CMD_LED_ARRAY_BINARY;
                    command.value = (data[4] == '1') ? 1 : 0;
                    xQueueSend(actuator_queue, &command, portMAX_DELAY);
                } else {
                    ESP_LOGW("UART_COMM", "Unknown command: %s", data);
                }

                // Move to next line and reset index
                uart_write_bytes(UART_PORT_NUM, "\r\n", 2);  // Move to next line
                idx = 0; // Reset the index for the next input
            } else {
                idx++;
                // Ensure we don't overflow the buffer
                if (idx >= UART_BUF_SIZE - 1) {
                    ESP_LOGW("UART_COMM", "Input too long, resetting buffer.");
                    idx = 0; // Reset the index if the buffer limit is reached
                }
            }
        }
    }
}