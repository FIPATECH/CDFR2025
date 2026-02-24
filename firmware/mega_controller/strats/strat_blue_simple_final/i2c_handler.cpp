// i2c_handler.cpp
#include "i2c_handler.h"
#include "globals.h"
#include <Wire.h>

static String buf;

void i2c_handler_init(uint8_t slaveAddress) {
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.print("i2c: esclave @0x");
  Serial.println(slaveAddress, HEX);
}

void receiveEvent(int howMany) {
  buf = "";
  while (Wire.available()) {
    buf += (char)Wire.read();
  }
  buf.trim();
  Serial.print("i2c_handler: reçu -> ");
  Serial.println(buf);

  if (buf.equalsIgnoreCase("start")) {
    matchStartRequested = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("i2c: LED ON");
  }
}

void requestEvent() {
  // Renvoie l'état de la LED BUILTIN
  Wire.write(digitalRead(LED_BUILTIN));
}

void sendI2CCommand(const char* cmd) {
  Wire.beginTransmission(UNO_I2C_ADDR);
  Wire.write(cmd);
  Wire.write('\n');
  Wire.endTransmission();
  Serial.print("i2c: sent -> ");
  Serial.println(cmd);
}