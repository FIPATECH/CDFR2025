#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include "match_trigger.h"
#include "uart_handler.h"

#define MATCH_PIN 2
volatile bool match_started = false;

void match_isr() {
    match_started = !match_started;
}

void match_trigger_init() {
    pinMode(MATCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MATCH_PIN), match_isr, FALLING);
    Serial.println("match_trigger_init: Interrupt attachée");
}

void match_trigger_task(void *pvParameters) {
    Serial.println("match_trigger_task started");
    bool last = match_started;
    for (;;) {
        if (match_started != last) {
            last = match_started;
            if (match_started) {
                Serial.println("match_trigger: START_MATCH déclenché");
                uart_send_start();
            } else {
                Serial.println("match_trigger: STOP_MATCH déclenché");
                uart_send_stop();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
