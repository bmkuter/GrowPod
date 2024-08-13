#include <stdio.h>
#include <string.h>
#include "ina260.h"
#include "actuator_control/actuator_control.h"
#include "uart_comm/uart_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void app_main(void) {
    // Create a FreeRTOS task for INA260
    xTaskCreate(&ina260_task, "INA260 Task", 4096, NULL, 5, NULL);

    printf("Hello world!\n");

    // Initialize actuators
    actuator_control_init();
    
    // Create a FreeRTOS task for actuator control
    xTaskCreate(&actuator_control_task, "Actuator Control Task", 4096, NULL, 5, NULL);

    // Initialize UART communication
    uart_comm_init();

    // Create a task for UART echo and command processing
    xTaskCreate(uart_echo_task, "UART Echo Task", 4096, NULL, 5, NULL);
}
