// i2c_handler.h
#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "globals.h"

// Initialise l’esclave I2C sur l’adresse spécifiée
void i2c_handler_init(uint8_t slaveAddress);

// Callback réception I2C
void receiveEvent(int howMany);

// Callback requête I2C
void requestEvent();

void sendI2CCommand(const char* cmd);


#endif // I2C_HANDLER_H
