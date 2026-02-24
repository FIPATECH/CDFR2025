#include "uart_handler.h"
#include "globals.h"
#include "command_manager.h"
#include "servo_controller.h"
// #include "motion_controller.h"
#include <string.h>

#define RX_CB_SIZE 512
#define TX_CB_SIZE 512

static CircularBuffer rxCB;
static CircularBuffer txCB;
static volatile bool isTransmitting = false;
static uint8_t rx_byte, tx_byte;

// États de réception
enum {
  RCV_WAIT = 0,
  RCV_FN_MSB,
  RCV_FN_LSB,
  RCV_LEN_MSB,
  RCV_LEN_LSB,
  RCV_PAYLOAD,
  RCV_CHECKSUM
};

static uint8_t rcvState = RCV_WAIT;
static uint16_t msgFunc = 0;
static uint16_t msgLen = 0;
static uint8_t msgPayload[256];
static uint16_t msgIndex = 0;

// Calcul local du checksum
static uint8_t calc_checksum(uint16_t function, uint16_t length, const uint8_t *payload) {
  uint8_t cs = 0x4A;
  cs ^= (uint8_t)(function >> 8);
  cs ^= (uint8_t)(function & 0xFF);
  cs ^= (uint8_t)(length >> 8);
  cs ^= (uint8_t)(length & 0xFF);
  for (uint16_t i = 0; i < length; i++) {
    cs ^= payload[i];
  }
  return cs;
}

void uart_init() {
  circular_buffer_init(&rxCB);
  circular_buffer_init(&txCB);
  Serial3.begin(115200);
  Serial3.setTimeout(1);
  Serial.println("uart_init: Serial3 @115200");
}

static void uart_start_tx() {
  if (!isTransmitting && circular_buffer_pop(&txCB, &tx_byte)) {
    isTransmitting = true;
    Serial.print(">> TX raw octet: 0x");
    if (tx_byte < 0x10) Serial.print('0');
    Serial.println(tx_byte, HEX);
    Serial3.write(&tx_byte, 1);
  }
}

void UART_Send_Bytes(const uint8_t *data, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    while (!circular_buffer_push(&txCB, data[i])) {}
  }
  uart_start_tx();
}

void UART_Send_Raw(const char *str) {
  UART_Send_Bytes((const uint8_t *)str, strlen(str));
}

void UART_Send_Text(const char *text) {
  uint16_t len = strlen(text);
  UART_Encode_And_Send_Message(UART_CMD_TEXT, len, (const uint8_t *)text);
}

void UART_Encode_And_Send_Message(uint16_t function, uint16_t length, const uint8_t *payload) {
  uint16_t total = 1 + 2 + 2 + length + 1;
  uint8_t buf[total];
  uint16_t pos = 0;
  buf[pos++] = 0x4A;             // HEADER
  buf[pos++] = function >> 8;    // FN MSB
  buf[pos++] = function & 0xFF;  // FN LSB
  buf[pos++] = length >> 8;      // LEN MSB
  buf[pos++] = length & 0xFF;    // LEN LSB
  if (length && payload) {
    memcpy(&buf[pos], payload, length);
    pos += length;
  }
  buf[pos++] = calc_checksum(function, length, payload);

  // Log complet de la trame
  Serial.print(">> TX frame: ");
  for (uint16_t i = 0; i < pos; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  UART_Send_Bytes(buf, pos);
}

void UART_Process_Decoded_Message(uint16_t function, uint16_t payloadLength, const uint8_t *payload) {
  switch (function) {
    case UART_CMD_PING:
      UART_Encode_And_Send_Message(UART_CMD_PONG, 0, NULL);
      break;

    case UART_CMD_PONG:
      break;

    case UART_CMD_TEXT:
      // ... inchangé ...
      break;

    case UART_CMD_START_MATCH:
      digitalWrite(LED_BUILTIN, HIGH);
      matchStartRequested = true;
      Serial.println("UART: START_MATCH reçu → LED ON");
      break;

    case UART_CMD_STOP_MATCH:
      matchStopReceived = true;
      Serial.println("UART: STOP_MATCH reçu");
      break;

    case UART_CMD_ACTION:
      {
        char cmd[129] = { 0 };
        memcpy(cmd, payload, min((uint16_t)128, payloadLength));
        CommandManager_Process_Command(cmd);
        break;
      }

    case UART_CMD_VEL:
      if (payloadLength == 12) {
        float v, w;
        uint32_t tick;
        memcpy(&v, payload + 0, 4);
        memcpy(&w, payload + 4, 4);
        memcpy(&tick, payload + 8, 4);
        // Motion_SetNewCommand(v, w);
      }
      break;

    default:
      break;
  }
}

void uart_task(void *pv) {
  uint8_t byte;
  for (;;) {
    while (Serial3.available()) {
      byte = Serial3.read();
      circular_buffer_push(&rxCB, byte);
    }
    while (circular_buffer_pop(&rxCB, &byte)) {
      switch (rcvState) {
        case RCV_WAIT:
          if (byte == 0x4A) rcvState = RCV_FN_MSB;
          break;
        case RCV_FN_MSB:
          msgFunc = (uint16_t)byte << 8;
          rcvState = RCV_FN_LSB;
          break;
        case RCV_FN_LSB:
          msgFunc |= byte;
          rcvState = RCV_LEN_MSB;
          break;
        case RCV_LEN_MSB:
          msgLen = (uint16_t)byte << 8;
          rcvState = RCV_LEN_LSB;
          break;
        case RCV_LEN_LSB:
          msgLen |= byte;
          msgIndex = 0;
          rcvState = msgLen ? RCV_PAYLOAD : RCV_CHECKSUM;
          break;
        case RCV_PAYLOAD:
          if (msgIndex < sizeof(msgPayload))
            msgPayload[msgIndex++] = byte;
          if (msgIndex >= msgLen) rcvState = RCV_CHECKSUM;
          break;
        case RCV_CHECKSUM:
          {
            uint8_t cs = calc_checksum(msgFunc, msgLen, msgPayload);
            if (cs == byte) {
              UART_Process_Decoded_Message(msgFunc, msgLen, msgPayload);
            } else {
              Serial.print("uart_task: CRC mismatch fn=0x");
              Serial.print(msgFunc, HEX);
              Serial.print(" calc=0x");
              Serial.print(cs, HEX);
              Serial.print(" rx=0x");
              Serial.println(byte, HEX);
            }
            rcvState = RCV_WAIT;
            break;
          }
      }
    }
    if (isTransmitting && Serial3.availableForWrite()) {
      isTransmitting = false;
      uart_start_tx();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void uart_send_start() {
  UART_Encode_And_Send_Message(UART_CMD_START_MATCH, 0, NULL);
}

void uart_send_stop() {
  UART_Encode_And_Send_Message(UART_CMD_STOP_MATCH, 0, NULL);
}

void uart_send_pong() {
  UART_Encode_And_Send_Message(UART_CMD_PONG, 0, NULL);
}
