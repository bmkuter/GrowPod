#ifndef UART_COMM_H
#define UART_COMM_H

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

// expose fill_pod API
#include "https_server.h"

// UART configurations
#define UART_PORT_NUM       UART_NUM_1
#define UART_BAUD_RATE      115200
#define UART_TX_GPIO        17
#define UART_RX_GPIO        16

// Function prototypes
void uart_comm_init(void);
void uart_console_task(void *pvParameter);

#endif // UART_COMM_H
