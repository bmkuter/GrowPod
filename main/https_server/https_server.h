// https_server/https_server.h

#ifndef HTTPS_SERVER_H
#define HTTPS_SERVER_H

#include "stdint.h"

#define ROUTINE_MAX_PWM 50

void start_https_server(void);
void init_schedule_manager(void);
int  get_current_hour(void);

// allow console to start the fill_pod routine
void start_fill_pod_routine(void);
// allow console to start the empty_pod routine
void start_empty_pod_routine(void);
// allow console to start a 24‑h schedule routine
void start_light_schedule(uint8_t schedule[24]);
void start_planter_schedule(uint8_t schedule[24]);
void start_air_schedule(uint8_t schedule[24]);
// Print current LED, planter & air schedules to console log
void print_schedules(void);

extern void schedule_manager_task(void *pvParam);
extern void schedule_air_task(void *pvParam);
extern void schedule_led_task(void *pvParam); 
extern void schedule_planter_task(void *pvParam);

#endif // HTTPS_SERVER_H
