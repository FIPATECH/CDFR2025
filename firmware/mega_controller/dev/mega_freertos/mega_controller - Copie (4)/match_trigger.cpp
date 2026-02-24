#include "match_trigger.h"
#include "globals.h"
#include "uart_handler.h"
#include "servo_controller.h"
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>

// Déclare la fonction I2C vers l’UNO CNC Shield
extern void I2C_Cmd_PlatformUp();

void match_trigger_task(void *pvParameters) {
  Serial.println("match_trigger_task started");

  for (;;) {
    if (matchStartRequested) {
      matchStartRequested = false;

      // LED reste déjà allumée par uart_handler à la réception START_MATCH
      Serial.println("match_trigger: démarrage match (LED reste ON)");

      // Ré-envoi START_MATCH pour sécurité / synchro
      uart_send_start();
      Serial.println("match_trigger: START_MATCH UART envoyé");

      // Envoyer test_xy via I2C
      I2C_Cmd_PlatformUp();
      Serial.println("match_trigger: I2C PlatformUp");

      // Ouvrir la pince
      servo_open();

      // Attendre la commande STOP_MATCH
      while (!matchStopReceived) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      // Fin de match : éteindre LED et réarmer flag
      digitalWrite(LED_BUILTIN, LOW);
      matchStopReceived = false;
      Serial.println("match_trigger: FIN match → LED OFF");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
