#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "circular_buffer.h"
#include "uart_commands.h"

void uart_init();
void uart_task(void *pvParameters);

void UART_Encode_And_Send_Message(uint16_t msgFunction, uint16_t msgPayloadLength, const uint8_t *msgPayload);

// Helpers
void uart_send_start();
void uart_send_stop();
void uart_send_pong();

void UART_Process_Decoded_Message(uint16_t function, uint16_t payloadLength, const uint8_t *payload);

#endif  // UART_HANDLER_H
