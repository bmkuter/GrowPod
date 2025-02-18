// https_server/https_server.h

#ifndef HTTPS_SERVER_H
#define HTTPS_SERVER_H

#define ROUTINE_MAX_PWM 50

void start_https_server(void);
void init_schedule_manager(void);
int get_current_hour(void);

#endif // HTTPS_SERVER_H
