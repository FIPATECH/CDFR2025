// motor_controller.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Numéros de moteur
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define MOTOR_COUNT 2

// Broches DIR
#define DIR_LEFT_GPIO_PORT GPIOC
#define DIR_LEFT_PIN GPIO_PIN_1
#define DIR_RIGHT_GPIO_PORT GPIOC
#define DIR_RIGHT_PIN GPIO_PIN_3

// Inversion d'un moteur
#define INVERT_LEFT_MOTOR false
#define INVERT_RIGHT_MOTOR true

    /* gauche avance - droit recul*/
    // #define INVERT_LEFT_MOTOR false
    // #define INVERT_RIGHT_MOTOR false

    /* 2 a l'avant */
    // #define INVERT_LEFT_MOTOR true
    // #define INVERT_RIGHT_MOTOR true

    /* gauche avant - droit recule*/
    // #define INVERT_LEFT_MOTOR true
    // #define INVERT_RIGHT_MOTOR false

    /* 2 a l'avant */
    // #define INVERT_LEFT_MOTOR false
    // #define INVERT_RIGHT_MOTOR true

    /**
     * @brief Initialise le contrôleur de moteurs (PWM + DIR par défaut)
     */
    void MotorController_Init(void);

    /**
     * @brief Définit la vitesse et le sens d’un moteur
     * @param motor_index  MOTOR_LEFT ou MOTOR_RIGHT
     * @param speed_percent 0…100 %
     * @param dir GPIO_PIN_RESET = avant, GPIO_PIN_SET = arrière
     */
    void MotorController_SetSpeed(uint8_t motor_index, uint8_t speed_percent, GPIO_PinState dir);

    void MotorController_ReInit(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */
