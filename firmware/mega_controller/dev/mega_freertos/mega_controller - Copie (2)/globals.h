#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// Flags partagés
extern volatile bool matchStartRequested;
extern volatile bool matchStopReceived;

// Adresse I2C de l’Arduino UNO CNC Shield
static const uint8_t UNO_I2C_ADDR = 0x08;

#endif // GLOBALS_H
