#include <Arduino.h>
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include "match_trigger.h"
#include "globals.h"
#include "uart_handler.h"
#include "servo_controller.h"

void match_trigger_task(void *pvParameters) {
  Serial.println("match_trigger_task started");
  for (;;) {
    if (matchStartRequested) {
      matchStartRequested = false;

      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("match_trigger: LED ON");

      uart_send_start();

      Wire.beginTransmission(UNO_I2C_ADDR);
      Wire.write("test_xy\n");
      Wire.endTransmission();
      Serial.println("match_trigger: I2C test_xy");

      servo_open();

      while (!matchStopReceived) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      digitalWrite(LED_BUILTIN, LOW);
      matchStopReceived = false;
      Serial.println("match_trigger: LED OFF");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
