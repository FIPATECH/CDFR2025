#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

extern volatile bool matchStartRequested;
extern volatile bool matchStopReceived;

typedef struct {
  float vR;
  float vL;
  long  cR;
  long  cL;
} OdomData_t;

extern QueueHandle_t odomQueue;

/* --- nouvelle ligne --- */
#include "odom_logger.h"

static const uint8_t UNO_I2C_ADDR = 0x08;


#endif
