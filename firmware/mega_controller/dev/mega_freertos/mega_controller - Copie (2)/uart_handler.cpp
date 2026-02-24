#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "uart_handler.h"
#include "crc16.h"
#include <string.h>

static CircularBuffer rxBuf;
QueueHandle_t frameQueue = NULL;

void uart_init() {
  circular_buffer_init(&rxBuf);
  frameQueue = xQueueCreate(10, sizeof(CommandFrame_t));
  Serial.println("uart_init: frameQueue créé");
}

void uart_task(void *pvParameters) {
  Serial.println("uart_task started");
  uint8_t state = 0, func = 0, length = 0, idx = 0;
  uint16_t crc_rx = 0;
  uint8_t payload[FRAME_MAX_PAYLOAD];

  for (;;) {
    while (Serial3.available()) {
      uint8_t b = Serial3.read();
      Serial.print("uart_task: raw=0x");
      if (b < 0x10) Serial.print('0');
      Serial.println(b, HEX);
      circular_buffer_push(&rxBuf, b);
    }
    uint8_t byte;
    while (circular_buffer_pop(&rxBuf, &byte)) {
      switch (state) {
        case 0:
          if (byte == HEADER_BYTE) state = 1;
          break;
        case 1:
          func = byte;
          state = 2;
          break;
        case 2:
          length = byte;
          idx = 0;
          state = (length <= FRAME_MAX_PAYLOAD) ? (length ? 3 : 4) : 0;
          break;
        case 3:
          payload[idx++] = byte;
          if (idx >= length) state = 4;
          break;
        case 4:
          crc_rx = ((uint16_t)byte) << 8;
          state = 5;
          break;
        case 5:
          crc_rx |= byte;
          {
            uint8_t buf[3 + length];
            buf[0] = HEADER_BYTE;
            buf[1] = func;
            buf[2] = length;
            if (length) memcpy(&buf[3], payload, length);
            uint16_t crc_calc = crc16(buf, 3 + length);
            Serial.print("uart_task: crc_calc=0x");
            Serial.print(crc_calc, HEX);
            Serial.print("  crc_rx=0x");
            Serial.println(crc_rx, HEX);
            if (crc_calc == crc_rx) {
              Serial.println("uart_task: Trame valide");
              CommandFrame_t frame = { func, length, { 0 } };
              if (length) memcpy(frame.payload, payload, length);
              xQueueSend(frameQueue, &frame, 0);
            } else {
              Serial.println("uart_task: CRC mismatch");
            }
          }
          state = 0;
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void uart_send_frame(uint8_t func, const uint8_t *pl, uint8_t len) {
  uint8_t buf[3 + FRAME_MAX_PAYLOAD + 2];
  buf[0] = HEADER_BYTE;
  buf[1] = func;
  buf[2] = len;
  if (len) memcpy(&buf[3], pl, len);
  uint16_t crc = crc16(buf, 3 + len);
  buf[3 + len] = crc >> 8;
  buf[4 + len] = crc & 0xFF;
  Serial3.write(buf, 5 + len);
}

void uart_send_start() {
  uart_send_frame(0x01, NULL, 0);
  Serial.println("uart_handler: START_MATCH envoyé");
}

void uart_send_stop() {
  uart_send_frame(0x02, NULL, 0);
  Serial.println("uart_handler: STOP_MATCH envoyé");
}

void uart_send_pong() {
  uart_send_frame(0x03, NULL, 0);
  Serial.println("uart_handler: PONG envoyé");
}
