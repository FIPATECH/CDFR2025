// ==================== uart_handler.cpp (Mega 2560) ====================
// Gestion UART bidirectionnelle (Serial2 ⇄ STM32, Serial3 ⇄ Jetson Nano)
// Désormais, la stratégie STM32 est reçue et affichée directement via le moniteur série
// -----------------------------------------------------------------------

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include "globals.h"

#include "uart_handler.h"
#include "uart_commands.h"
#include "circular_buffer.h"
#include <string.h>
#include "points_manager.h"
#include "command_manager.h"

// Énumération des couleurs pour debug
typedef enum {
  BLUE = 0,
  YELLOW = 1
} Color;

// ---------- constantes ----------
#define HEADER 0x4A
#define CB_SIZE 256  // taille buffer circulaire (doit correspondre à circular_buffer.h)

// ---------- buffers TX par port ----------
static CircularBuffer txCB2;  // Serial2 ↔ STM32
static CircularBuffer txCB3;  // Serial3 ↔ Jetson Nano
static volatile bool isTx2 = false, isTx3 = false;
static uint8_t tx_byte2 = 0, tx_byte3 = 0;

// ---------- décodeurs RX ----------
typedef struct {
  uint8_t state;
  uint16_t fn;
  uint16_t len;
  uint8_t payload[32];
  uint16_t idx;
} decoder_t;
static decoder_t dec2 = { 0 }, dec3 = { 0 };

// ---------- calcul checksum ----------
static uint8_t calc_cs(uint16_t fn, uint16_t len, const uint8_t *pl) {
  uint8_t cs = HEADER;
  cs ^= (uint8_t)(fn >> 8);
  cs ^= (uint8_t)fn;
  cs ^= (uint8_t)(len >> 8);
  cs ^= (uint8_t)len;
  for (uint16_t i = 0; i < len; ++i) cs ^= pl[i];
  return cs;
}

// ---------- prototypes internes ----------
static void UART_Send_Bytes_Port(uart_port_t port, const uint8_t *data, uint16_t len);
static void UART_Process_Decoded_Message_Port(uart_port_t src, uint16_t fn, uint16_t len, const uint8_t *pl);
static void decode_byte(decoder_t &d, uint8_t c, uart_port_t src);

// =========== API TX Haut niveau ===========
void UART_Encode_And_Send_Message_Port(uart_port_t port, uint16_t fn, uint16_t len, const uint8_t *pl) {
  uint16_t tot = 1 + 2 + 2 + len + 1;
  uint8_t buf[64];  // S'assurer que 1+2+2+max_payload+1 <= 64
  uint16_t p = 0;
  buf[p++] = HEADER;
  buf[p++] = (uint8_t)(fn >> 8);
  buf[p++] = (uint8_t)(fn & 0xFF);
  buf[p++] = (uint8_t)(len >> 8);
  buf[p++] = (uint8_t)(len & 0xFF);
  if (len && pl) {
    memcpy(&buf[p], pl, len);
    p += len;
  }
  buf[p++] = calc_cs(fn, len, pl);
  UART_Send_Bytes_Port(port, buf, p);
}

// Simplification : envoi sur Serial3 par défaut
void UART_Encode_And_Send_Message(uint16_t fn, uint16_t len, const uint8_t *pl) {
  UART_Encode_And_Send_Message_Port(UART_PORT_3, fn, len, pl);
}

void UART_Send_Text(const char *txt) {
  UART_Encode_And_Send_Message_Port(UART_PORT_3, UART_CMD_TEXT, strlen(txt), (const uint8_t *)txt);
}

// =========== TX bas niveau ===========
static void UART_Send_Bytes_Port(uart_port_t port, const uint8_t *data, uint16_t len) {
  CircularBuffer *cb = (port == UART_PORT_2) ? &txCB2 : &txCB3;
  volatile bool *flag = (port == UART_PORT_2) ? &isTx2 : &isTx3;
  uint8_t *first = (port == UART_PORT_2) ? &tx_byte2 : &tx_byte3;

  // Empile les octets
  for (uint16_t i = 0; i < len; ++i) {
    while (((cb->head + 1) % CB_SIZE) == cb->tail) { taskYIELD(); }
    circular_buffer_push(cb, data[i]);
  }
  // Lancement si libre
  if (!*flag && circular_buffer_pop(cb, first)) {
    *flag = true;
    if (port == UART_PORT_2) Serial2.write(first, 1);
    else Serial3.write(first, 1);
  }
}

// =========== Initialisation UART ===========
void uart_init() {
  circular_buffer_init(&txCB2);
  circular_buffer_init(&txCB3);
  Serial2.begin(115200, SERIAL_8N1);
  Serial3.begin(115200, SERIAL_8N2);
  Serial.begin(115200);
  Serial.println("uart_init: Serial2 & Serial3 prêts");
}

// =========== Décodage par octet ===========
static void decode_byte(decoder_t &d, uint8_t c, uart_port_t src) {
  enum { W,
         FMSB,
         FLSB,
         LMSB,
         LLSB,
         PAY,
         CS };
  switch (d.state) {
    case W:
      if (c == HEADER) {
        d.state = FMSB;
        d.idx = 0;
      }
      break;
    case FMSB:
      d.fn = (uint16_t)c << 8;
      d.state = FLSB;
      break;
    case FLSB:
      d.fn |= c;
      d.state = LMSB;
      break;
    case LMSB:
      d.len = (uint16_t)c << 8;
      d.state = LLSB;
      break;
    case LLSB:
      d.len |= c;
      d.state = d.len ? PAY : CS;
      break;
    case PAY:
      if (d.idx < sizeof(d.payload)) d.payload[d.idx++] = c;
      if (d.idx >= d.len) d.state = CS;
      break;
    case CS:
      {
        uint8_t cs = calc_cs(d.fn, d.len, d.payload);
        if (cs == c) UART_Process_Decoded_Message_Port(src, d.fn, d.len, d.payload);
        d.state = W;
        break;
      }
    default:
      d.state = W;
      break;
  }
}

// =========== Dispatch des messages décodés ===========
static void UART_Process_Decoded_Message_Port(uart_port_t src, uint16_t fn, uint16_t len, const uint8_t *pl) {
  switch (fn) {
    case UART_CMD_STRATEGY:
      {
        if (src == UART_PORT_2 && len > 0) {
          // copie et termine en chaîne C
          uint8_t buf[32];
          uint16_t copy_len = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
          memcpy(buf, pl, copy_len);
          buf[copy_len] = '\0';

          // décalage pour passer "STRATEGY:"
          const char *p = (const char *)buf + 9;
          uint8_t teamColor;
          if (sscanf(p, "%hhu", &teamColor) == 1) {
            // mapping vers le nom
            const char *colorName = "UNKNOWN";
            switch (teamColor) {
              case BLUE: colorName = "BLUE"; break;
              case YELLOW: colorName = "YELLOW"; break;
            }
            Serial.print("TeamColor reçu : ");
            Serial.println(colorName);
          } else {
            Serial.println("Erreur parsing STRATEGY");
          }
        }
        break;
      }

    case UART_CMD_PING:
      UART_Encode_And_Send_Message_Port(src, UART_CMD_PONG, 0, nullptr);
      break;

    case UART_CMD_STOP_MATCH:
      // Signal stop match
      extern volatile bool matchStopReceived;
      matchStopReceived = true;
      Serial.println("UART: STOP_MATCH reçu");
      break;

    case UART_CMD_ACTION:
      {
        // Copie payload dans chaîne C
        char command[128] = { 0 };
        size_t copy_length = (len < sizeof(command) - 1) ? len : (sizeof(command) - 1);
        memcpy(command, pl, copy_length);
        command[copy_length] = '\0';
        // Dispatch commande
        CommandManager_Process_Command(command);
        break;
      }

    default:
      break;
  }
}

// =========== Tâche FreeRTOS UART ===========
void uart_task(void *arg) {
  (void)arg;
  for (;;) {
    // Réception
    while (Serial2.available()) decode_byte(dec2, Serial2.read(), UART_PORT_2);
    while (Serial3.available()) decode_byte(dec3, Serial3.read(), UART_PORT_3);
    // Transmission
    if (isTx2 && Serial2.availableForWrite()) {
      if (circular_buffer_pop(&txCB2, &tx_byte2)) Serial2.write(&tx_byte2, 1);
      else isTx2 = false;
    }
    if (isTx3 && Serial3.availableForWrite()) {
      if (circular_buffer_pop(&txCB3, &tx_byte3)) Serial3.write(&tx_byte3, 1);
      else isTx3 = false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =========== Polling sans RTOS ===========
void uart_poll() {
  // Réception
  while (Serial2.available()) decode_byte(dec2, Serial2.read(), UART_PORT_2);
  while (Serial3.available()) decode_byte(dec3, Serial3.read(), UART_PORT_3);
  // Transmission
  if (isTx2 && Serial2.availableForWrite()) {
    if (circular_buffer_pop(&txCB2, &tx_byte2)) Serial2.write(&tx_byte2, 1);
    else isTx2 = false;
  }
  if (isTx3 && Serial3.availableForWrite()) {
    if (circular_buffer_pop(&txCB3, &tx_byte3)) Serial3.write(&tx_byte3, 1);
    else isTx3 = false;
  }
}
