#pragma once
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "globals.h"            // ← pour le type OdomData_t

/* ---------- PINOUT -------------------------------------------------- */
#define ENC_RA  2
#define ENC_LA  3
#define ENC_RB  4
#define ENC_LB  5
#define DIR_R   8
#define PWM_R   9
#define DIR_L   7
#define PWM_L  10

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise le module PID (broches, interruptions, tâches). */
void PIDControl_Init(void);

/** Change la consigne (tops/40 ms). */
void PIDControl_SetCommand(int16_t speedLeft, int16_t speedRight);

#ifdef __cplusplus
}
#endif
