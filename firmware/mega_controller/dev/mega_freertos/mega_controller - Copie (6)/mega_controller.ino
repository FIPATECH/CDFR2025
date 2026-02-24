#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h>

#include "globals.h"
#include "uart_handler.h"
#include "uart_commands.h"  // pour UART_CMD_POINTS
#include "command_manager.h"
#include "servo_controller.h"
#include "i2c_handler.h"
#include "match_trigger.h"
#include "points_manager.h"

// Flags partagés
volatile bool matchStartRequested = false;
volatile bool matchStopReceived = false;

/**
 * @brief Encode et transmet une trame UART sur Serial2, puis log la trame sur USB-Serial
 */
void sendFrameSerial2(uint16_t function, uint16_t length, const uint8_t *payload) {
  // Construction du buffer : header + function + length + payload + checksum
  uint16_t total = 1 + 2 + 2 + length + 1;
  uint8_t buf[20];
  uint16_t pos = 0;

  buf[pos++] = 0x4A;
  buf[pos++] = (function >> 8) & 0xFF;
  buf[pos++] = function & 0xFF;
  buf[pos++] = (length >> 8) & 0xFF;
  buf[pos++] = length & 0xFF;
  if (length && payload) {
    memcpy(&buf[pos], payload, length);
    pos += length;
  }
  uint8_t cs = buf[0];
  for (uint16_t i = 1; i < pos; i++) {
    cs ^= buf[i];
  }
  buf[pos++] = cs;

  // Envoi sur Serial2 (Mega TX2 → STM32 RX)
  Serial2.write(buf, pos);

  // Log sur USB-Serial
  Serial.print("Serial2 TX [");
  for (uint16_t i = 0; i < pos; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println("]");
}

// /**
//  * @brief Tâche FreeRTOS : envoi périodique des points
//  *        Incrémente de 5, wrap à 0 après 100.  
//  *        Log de démarrage pour vérifier qu'elle tourne.
//  */
// static void PointsSend_Task(void *pvParameters) {
//   (void)pvParameters;
//   uint32_t points = 0;

//   Serial.println(">> PointsSend_Task démarrée");

//   for (;;) {
//     Serial.print("PointsSend_Task: points=");
//     Serial.println(points);

//     uint8_t payload[4] = {
//       (uint8_t)(points >> 24),
//       (uint8_t)(points >> 16),
//       (uint8_t)(points >> 8),
//       (uint8_t)(points)
//     };

//     sendFrameSerial2(UART_CMD_POINTS, 4, payload);

//     points = (points + 5) % 100;

//     vTaskDelay(pdMS_TO_TICKS(1000));
//   }
// }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F(">> USB Serial OK"));

  // Port vers la STM32 (TX2↔RX2)
  Serial2.begin(115200);
  Serial.println(F(">> Serial2 (STM32) @115200"));

  // Port vers la Jetson Nano
  Serial3.begin(115200);
  Serial.println(F(">> Serial3 (Jetson) @115200"));

  Wire.begin();

  uart_init();
  command_manager_init();
  servo_init();
  i2c_handler_init(0x07);

  // Tâches existantes
  xTaskCreate(uart_task, "UART_RX", 512, NULL, 4, NULL);
  xTaskCreate(servo_task, "SERVO", 256, NULL, 2, NULL);
  xTaskCreate(match_trigger_task, "MATCH_TRIG", 512, NULL, 3, NULL);

  // Tâche POINTS en priorité 5 pour qu'elle puisse s'exécuter même si uart_task tourne sans délai
  PointsManager_Init();

  vTaskStartScheduler();
}

void loop() {
  // Ne doit jamais être atteint sous FreeRTOS
}
