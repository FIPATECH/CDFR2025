#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include "globals.h"
#include "uart_handler.h"
#include "command_manager.h"
#include "servo_controller.h"
#include "i2c_handler.h"
#include "match_trigger.h"

// Initialisation des flags
volatile bool matchStartRequested = false;
volatile bool matchStopReceived = false;

void setup() {
  // LED BUILTIN en OUTPUT, éteinte par défaut
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  Serial.println("Setup: démarrage firmware Mega");

  Serial3.begin(9600);
  Serial.println("Setup: Serial3 @9600");

  // Nécessaire pour I2C maître occasionnel
  Wire.begin();

  uart_init();
  command_manager_init();
  servo_init();

  // Initialise l’esclave I2C sur 0x07 pour recevoir “start”
  i2c_handler_init(0x07);

  // Création des tâches FreeRTOS
  xTaskCreate(uart_task, "UART_RX", 256, NULL, 4, NULL);
  xTaskCreate(command_manager_task, "CMD_MGR", 256, NULL, 3, NULL);
  xTaskCreate(servo_task, "SERVO", 128, NULL, 2, NULL);
  xTaskCreate(match_trigger_task, "MATCH_TRIG", 256, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Inatteignable : le scheduler tourne
}
