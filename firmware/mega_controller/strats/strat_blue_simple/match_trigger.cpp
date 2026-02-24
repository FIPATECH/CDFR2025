#include "match_trigger.h"
#include "globals.h"
#include "uart_handler.h"
#include "uart_commands.h"
#include "servo_controller.h"
#include <Wire.h>
#include "points_manager.h"

// Fonctions externes
extern void I2C_Cmd_PlatformUp();
extern void sendFrameSerial2(uint16_t function, uint16_t length, const uint8_t *payload);
extern void moveDirect(int leftTicks, int rightTicks, unsigned long duration_ms);

void match_trigger_loop() {
  // if (!matchStartRequested) return;

  // matchStartRequested = false;
  // Serial.println("match_trigger: démarrage match (LED reste ON)");

  // // Ré-envoi START_MATCH UART pour synchro
  // uart_send_start();
  // sendFrameSerial2(UART_CMD_START_MATCH, 0, nullptr);

  // // Attendre STOP_MATCH
  // while (!matchStopReceived) {
  //   // simple pause non bloquante
  //   delay(10);
  // }

  // // Fin de match : LED off et reset flag
  // digitalWrite(LED_BUILTIN, LOW);
  // matchStopReceived = false;
  // Serial.println("match_trigger: FIN match → LED OFF");
}
