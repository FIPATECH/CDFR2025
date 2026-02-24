#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>

void servo_init();
void servo_open();
void servo_close();
void servo_task(void *pvParameters);

#endif  // SERVO_CONTROLLER_H
