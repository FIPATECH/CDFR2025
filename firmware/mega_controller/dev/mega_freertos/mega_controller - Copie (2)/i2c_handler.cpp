#include <Arduino.h>
#include <Wire.h>
#include "i2c_handler.h"
#include "globals.h"

static String lastCmd;

void i2c_handler_init(uint8_t slaveAddress) {
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.print("i2c_handler_init: I2C esclave @0x");
  Serial.println(slaveAddress, HEX);
}

void receiveEvent(int howMany) {
  lastCmd = "";
  while (Wire.available()) {
    lastCmd += (char)Wire.read();
  }
  lastCmd.trim();
  Serial.print("i2c_handler: receiveEvent cmd=");
  Serial.println(lastCmd);

  if (lastCmd.equalsIgnoreCase("start")) {
    matchStartRequested = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("i2c_handler: LED ON");
  } else {
    matchStartRequested = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("i2c_handler: LED OFF");
  }
}

void requestEvent() {
  uint8_t state = digitalRead(LED_BUILTIN);
  Serial.print("i2c_handler: requestEvent envoi status=");
  Serial.println(state);
  Wire.write(state);
}
