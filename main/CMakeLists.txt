idf_component_register(SRCS "main.c" 
                            "ina260/ina260.c"
                            "actuator_control/actuator_control.c"
                            "uart_comm/uart_comm.c"
                    INCLUDE_DIRS "." 
                                  "ina260" 
                                  "actuator_control" 
                                  "uart_comm")
