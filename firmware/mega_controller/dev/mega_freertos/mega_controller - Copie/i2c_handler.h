#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>

void i2c_handler_init();
void receiveEvent(int len);
void requestEvent();

#endif
