#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stm32f4xx_hal.h"

// Valeurs de PWM (µs) provenants des tests
#define NEUTRAL_POSITION_LEFT 1800
#define NEUTRAL_POSITION_RIGHT 1500

#define OPEN_POSITION_LEFT 1400
#define OPEN_POSITION_RIGHT 1800

#define CLOSE_POSITION_LEFT 1400
#define CLOSE_POSITION_RIGHT 1900

// Durée d'exécution des actions (ms)
#define OPEN_GRIPPER_DURATION 10000
#define CLOSE_GRIPPER_DURATION 10000

    void ServoController_Init(void);
    void Execute_Open_Gripper(void);
    void Execute_Close_Gripper(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROLLER_H */
