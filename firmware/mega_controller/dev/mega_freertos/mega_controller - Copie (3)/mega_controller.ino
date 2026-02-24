#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include "globals.h"
#include "uart_handler.h"
#include "command_manager.h"
#include "servo_controller.h"
#include "i2c_handler.h"
#include "match_trigger.h"

// Flags partagés
volatile bool matchStartRequested = false;
volatile bool matchStopReceived   = false;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    Serial.println("Setup: démarrage Mega");

    Serial3.begin(115200);
    Serial.println("Setup: Serial3 @115200");

    Wire.begin();

    uart_init();
    command_manager_init();   // <— maintenant déclaré
    servo_init();

    i2c_handler_init(0x07);

    xTaskCreate(uart_task,            "UART_RX",    512, NULL, 4, NULL);
    xTaskCreate(servo_task,           "SERVO",      256, NULL, 2, NULL);
    xTaskCreate(match_trigger_task,   "MATCH_TRIG", 512, NULL, 3, NULL);

    vTaskStartScheduler();
}

void loop() {
    // jamais atteint
}
