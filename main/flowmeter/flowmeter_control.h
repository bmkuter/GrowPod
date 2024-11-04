// flowmeter/flowmeter_control.h

#ifndef FLOWMETER_CONTROL_H
#define FLOWMETER_CONTROL_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"

// RMT channel definitions
#define RMT_CHANNEL_DRAIN       0
#define RMT_CHANNEL_SOURCE      1
#define RMT_CHANNEL_OVERFLOW    2  // New channel for overflow

// GPIO definitions
#define FLOWMETER_GPIO_DRAIN    GPIO_NUM_8  // Drain flowmeter
#define FLOWMETER_GPIO_SOURCE   GPIO_NUM_9  // Source flowmeter
#define FLOWMETER_GPIO_OVERFLOW GPIO_NUM_10 // Overflow flowmeter

// Function prototypes
esp_err_t flowmeter_init(void);
void flowmeter_task(void *pvParameter);
// float get_drain_flow_rate(void);
// float get_source_flow_rate(void);
// float get_overflow_flow_rate(void);

#endif // FLOWMETER_CONTROL_H
