#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

// Plages en microsecondes pour un servo standard :
// Pour un servo à rotation continue, 1000 pour tourner et 1500 pour stopper
#define SERVO_MIN_PULSE 1000  // 1 ms pour rotation (ex : OPEN_GRIPPER)
#define SERVO_STOP_PULSE 1500 // 1.5 ms pour stopper
#define SERVO_MAX_PULSE 2000  // 2 ms pour rotation dans l'autre sens
#define SERVO_PERIOD 20000    // 20 ms (50 Hz)

    void Servo_Init(void);
    void SetServoAngle(uint8_t angle);
    void Execute_Open_Gripper(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_CONTROLLER_H
