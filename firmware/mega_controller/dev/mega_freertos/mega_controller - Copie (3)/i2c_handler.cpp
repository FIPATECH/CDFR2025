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

// Envoie "test_xy\n" à l'UNO CNC Shield via I2C
void I2C_Cmd_PlatformUp() {
    Wire.beginTransmission(UNO_I2C_ADDR);
    Wire.write("test_xy\n");
    Wire.endTransmission();
    Serial.println("i2c: PlatformUp envoyé");
}
