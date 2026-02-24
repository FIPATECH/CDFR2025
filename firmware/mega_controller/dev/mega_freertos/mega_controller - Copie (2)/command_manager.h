#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "uart_handler.h"

extern volatile bool matchStopReceived;

// Prototype du stub moteur
void motor_set_velocity(int16_t v_l, int16_t v_r);

void command_manager_init();
void command_manager_task(void *pvParameters);

#endif  // COMMAND_MANAGER_H
