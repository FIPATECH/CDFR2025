#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include "i2c_handler.h"

#define I2C_ADDR 0x08

void receiveEvent(int len) {
    Serial.print("i2c_handler: receiveEvent len=");
    Serial.println(len);
    while (Wire.available()) {
        uint8_t b = Wire.read();
        // traiter b
    }
}

void requestEvent() {
    uint8_t status = 0x00;
    Serial.println("i2c_handler: requestEvent envoi status");
    Wire.write(status);
}

void i2c_handler_init() {
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.println("i2c_handler_init: I2C esclave initialisé");
}
