#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <Arduino.h>
#include <stdint.h>

/* Choix explicite du port */
typedef enum {
  UART_PORT_2 = 2,  // Serial2  (STM32)
  UART_PORT_3 = 3   // Serial3  (Jetson Nano)
} uart_port_t;

/* ----- API TX sélective ----- */
void UART_Encode_And_Send_Message_Port(uart_port_t port,
                                       uint16_t fn,
                                       uint16_t len,
                                       const uint8_t *payload);
void UART_Send_Text_Port(uart_port_t port, const char *txt);

/* Compatibilité : versions “historiques” (=> Jetson) */
void UART_Encode_And_Send_Message(uint16_t fn,
                                  uint16_t len,
                                  const uint8_t *payload);
void UART_Send_Text(const char *txt);

/* Init + tâche FreeRTOS */
void uart_init(void);
void uart_task(void *arg);
void uart_send_start(void);
void uart_send_stop (void);
void uart_send_pong (void);

void uart_poll();
#endif
