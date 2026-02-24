#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "circular_buffer.h"

#define FRAME_MAX_PAYLOAD 32
#define HEADER_BYTE       0x4A

typedef struct {
    uint8_t func;
    uint8_t length;
    uint8_t payload[FRAME_MAX_PAYLOAD];
} CommandFrame_t;

extern QueueHandle_t frameQueue;

void uart_init();
void uart_task(void *pvParameters);
void uart_send_start();
void uart_send_stop();
void uart_send_pong();

#endif
