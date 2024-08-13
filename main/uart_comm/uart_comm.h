#ifndef UART_COMM_H
#define UART_COMM_H

#include "driver/uart.h"
#include "esp_log.h"

// UART configurations
#define UART_PORT_NUM       UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_TX_GPIO        21
#define UART_RX_GPIO        20

#define UART_BUF_SIZE       1024

// Function prototypes
void uart_comm_init(void);
void uart_send_data(const char* data);
void uart_receive_data(char* data, uint16_t max_len);
void uart_echo_task(void *pvParameter);

#endif // UART_COMM_H
