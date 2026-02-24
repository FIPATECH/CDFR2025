#pragma once
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif
void odom_logger_task(void *pvParameters);
#ifdef __cplusplus
}
#endif
