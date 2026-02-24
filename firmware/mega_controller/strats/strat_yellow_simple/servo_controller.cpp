#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Servo.h>
#include "servo_controller.h"
#include "points_manager.h"

static Servo gripper;
#define SERVO_PIN 9
#define OPEN_US 2100
#define CLOSE_US 900

void servo_init() {
  gripper.attach(SERVO_PIN);
  gripper.writeMicroseconds(CLOSE_US);
  Serial.println("servo_init: Gripper attaché");
}

void servo_open() {
  gripper.writeMicroseconds(OPEN_US);
  Serial.println("servo_controller: OPEN_GRIPPER exécuté");
}

void servo_close() {
  gripper.writeMicroseconds(CLOSE_US);
  Serial.println("servo_controller: CLOSE_GRIPPER exécuté");
}

void servo_task(void *pvParameters) {
  Serial.println("servo_task started");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
