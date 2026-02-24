#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include "uart_handler.h"
#include "command_manager.h"
#include "servo_controller.h"
#include "match_trigger.h"
#include "i2c_handler.h"
#include "strategy_manager.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Setup: Serial initialized.");

    // Passe la liaison Jetson → Mega sur 9600 pour fiabilité Nano ↔ Mega
    Serial3.begin(9600);
    Serial.println("Setup: Serial3 @9600");
    Serial3.setTimeout(10);

    Wire.begin();
    Serial.println("Setup: I2C initialized.");

    uart_init();
    command_manager_init();
    servo_init();
    match_trigger_init();
    i2c_handler_init();
    Serial.println("Setup: Modules initialized.");

    if (xTaskCreate(uart_task,            "UART_RX",    256, NULL, 4, NULL) == pdPASS)
        Serial.println("Task created: uart_task");
    if (xTaskCreate(command_manager_task, "CMD_MGR",    256, NULL, 3, NULL) == pdPASS)
        Serial.println("Task created: command_manager_task");
    if (xTaskCreate(servo_task,           "SERVO",      128, NULL, 2, NULL) == pdPASS)
        Serial.println("Task created: servo_task");
    if (xTaskCreate(match_trigger_task,   "MATCH_TRIG", 128, NULL, 2, NULL) == pdPASS)
        Serial.println("Task created: match_trigger_task");

    Serial.println("Setup complete. Scheduler starting...");
}

void loop() {
    // Vide : le scheduler FreeRTOS tourne à la place
}
