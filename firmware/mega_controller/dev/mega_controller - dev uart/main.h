#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

// Broches “STM32-like”
#define LED_RED_Pin 13
#define LED_GREEN_Pin 12

// Flags externes
extern volatile uint8_t matchStopReceived;
extern volatile uint8_t startMatchFlag;

#endif  // MAIN_H
