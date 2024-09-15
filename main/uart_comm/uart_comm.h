#ifndef UART_COMM_H
#define UART_COMM_H

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

// UART configurations
#define UART_PORT_NUM       UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_TX_GPIO        21
#define UART_RX_GPIO        20

// Function prototypes
void uart_comm_init(void);
void uart_console_task(void *pvParameter);

#endif // UART_COMM_H
