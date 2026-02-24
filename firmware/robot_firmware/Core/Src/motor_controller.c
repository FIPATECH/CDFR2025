#include "motor_controller.h"

extern TIM_HandleTypeDef htim2;

static const uint32_t pwm_channel[MOTOR_COUNT] = {
    TIM_CHANNEL_1, // moteur 0 → TIM2_CH1 (PA5)
    TIM_CHANNEL_2  // moteur 1 → TIM2_CH2 (PB3)
};

static GPIO_TypeDef *dir_port[MOTOR_COUNT] = {
    DIR_LEFT_GPIO_PORT,
    DIR_RIGHT_GPIO_PORT};

static uint16_t dir_pin[MOTOR_COUNT] = {
    DIR_LEFT_PIN,
    DIR_RIGHT_PIN};

// 0 = sens normal, 1 = sens inversé
static const int invert_direction[MOTOR_COUNT] = {
    0, // moteur 0
    1  // moteur 1
};

// 1, 1 avance
// 0, 0 gauche avant droite recule
// 1, 0 gauche avant droite recule
// 0, 1 gauche avant droite recule

void MotorController_Init(void)
{
    // Démarre le PWM sur TIM2 pour les deux canaux
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        HAL_TIM_PWM_Start(&htim2, pwm_channel[i]);
        // initialise DIR à avant (LOW)
        HAL_GPIO_WritePin(dir_port[i], dir_pin[i], GPIO_PIN_RESET);
        // et vitesse 0 %
        __HAL_TIM_SET_COMPARE(&htim2, pwm_channel[i], 0);
    }
}

void MotorController_SetSpeed(uint8_t motor_index, uint8_t speed_percent, GPIO_PinState dir)
{
    if (motor_index >= MOTOR_COUNT || speed_percent > 100)
    {
        return;
    }

    // Applique l'inversion si nécessaire
    if (invert_direction[motor_index])
    {
        dir = (dir == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    // Règle la direction
    HAL_GPIO_WritePin(dir_port[motor_index], dir_pin[motor_index], dir);

    // Calcule le CCR à partir de l’ARR
    uint32_t arr = htim2.Instance->ARR;
    uint32_t pulse = (speed_percent * (arr + 1)) / 100;
    __HAL_TIM_SET_COMPARE(&htim2, pwm_channel[motor_index], pulse);
}

void MotorController_ReInit(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        HAL_TIM_PWM_Stop(&htim2, pwm_channel[i]);
        HAL_TIM_PWM_Start(&htim2, pwm_channel[i]);
        HAL_GPIO_WritePin(dir_port[i], dir_pin[i], GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, pwm_channel[i], 0);
        // Attends un tout petit délai pour que le driver “voit” le PWM OFF
        HAL_Delay(2);
    }
}
