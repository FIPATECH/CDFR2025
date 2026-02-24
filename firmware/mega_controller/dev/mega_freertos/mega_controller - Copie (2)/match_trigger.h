#ifndef MATCH_TRIGGER_H
#define MATCH_TRIGGER_H

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>

void match_trigger_task(void *pvParameters);

#endif  // MATCH_TRIGGER_H
