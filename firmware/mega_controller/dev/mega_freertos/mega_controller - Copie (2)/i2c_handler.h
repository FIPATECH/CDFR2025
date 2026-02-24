#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "globals.h"

// Initialise l’esclave I2C sur l’adresse donnée
void i2c_handler_init(uint8_t slaveAddress);

// Callback esclave : réception de données
void receiveEvent(int howMany);

// Callback esclave : requête du maître
void requestEvent();

#endif  // I2C_HANDLER_H
