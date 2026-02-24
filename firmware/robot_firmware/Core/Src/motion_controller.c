/* motion_controller.c -----------------------------------------------------*/
#include "motion_controller.h"
#include "motor_controller.h"
#include "speed_controller.h"
#include "odometry.h"    // Désactivé : utilisation directe des TIM encoders
#include "cmsis_os2.h"
#include <math.h>
#include "main.h" // pour les déclarations externes des TIM_HandleTypeDef

/* config robot -------------------------------------------------------------- */
#define LOOP_HZ 100.0f
#define MAX_LINEAR_V 0.50f
#define MAX_ANGULAR_W 1.00f

// Simple PWM mode flag
static bool simple_pwm_mode = true;
// Proportional gain for encoder correction
#define K_P 0.2f
// Clamp macro
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

static volatile float cmd_v = 0.0f;
static volatile float cmd_w = 0.0f;

extern TIM_HandleTypeDef htim3; // Encodeur gauche
extern TIM_HandleTypeDef htim4; // Encodeur droite

void Motion_SetNewCommand(float v, float w)
{
    cmd_v = v;
    cmd_w = w;
}

static void Motion_Task(void *arg)
{
    const TickType_t period = (TickType_t)(osKernelGetTickFreq() / LOOP_HZ);
    TickType_t next = osKernelGetTickCount();

    for (;;)
    {
        float v = cmd_v;
        float w = cmd_w;

        // Saturate commands
        v = (v > MAX_LINEAR_V) ? MAX_LINEAR_V : ((v < -MAX_LINEAR_V) ? -MAX_LINEAR_V : v);
        w = (w > MAX_ANGULAR_W) ? MAX_ANGULAR_W : ((w < -MAX_ANGULAR_W) ? -MAX_ANGULAR_W : w);

        // Compute wheel speeds (rad/s)
        float wl = (v - w * (WHEEL_BASE * 0.5f)) / WHEEL_RADIUS;
        float wr = (v + w * (WHEEL_BASE * 0.5f)) / WHEEL_RADIUS;

        // Convert to PWM %
        float rad_s_max = MAX_LINEAR_V / WHEEL_RADIUS;
        int pct_l = (int)(fabsf(wl) * 100.0f / rad_s_max);
        int pct_r = (int)(fabsf(wr) * 100.0f / rad_s_max);
        pct_l = (pct_l > 100) ? 100 : pct_l;
        pct_r = (pct_r > 100) ? 100 : pct_r;

        // --------- ANCIEN CODE (STM32 avec SpeedController) ---------
        /*
        // Démarre le régulateur PID
        SpeedController_SetReference(wl, wr);
        */
        // -------------------------------------------------------------

        // Mode simple PWM avec correction d'encodeurs
        if (simple_pwm_mode && fabsf(w) < 1e-3f)
        {
            // Lecture directe des compteurs TIM (32-bit)
            long count_l = (long)__HAL_TIM_GET_COUNTER(&htim3);
            long count_r = (long)__HAL_TIM_GET_COUNTER(&htim4);
            long error = count_l - count_r;
            int delta = (int)(K_P * error);

            pct_l = CLAMP(pct_l - delta, 0, 100);
            pct_r = CLAMP(pct_r + delta, 0, 100);

            // Remise à zéro si overflow
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            __HAL_TIM_SET_COUNTER(&htim4, 0);
        }

        // Direction moteurs
        GPIO_PinState dir_l = (wl >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        GPIO_PinState dir_r = (wr >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET;

        // Application aux moteurs
        MotorController_SetSpeed(0, pct_l, dir_l);
        MotorController_SetSpeed(1, pct_r, dir_r);

        // Attente jusqu'à la prochaine période
        next += period;
        osDelayUntil(next);
    }
}

void Motion_Init(void)
{
    // Initialisation du contrôleur de vitesse (PID)
    SpeedController_Init();

    const osThreadAttr_t attr = {
        .name = "Motion",
        .priority = osPriorityAboveNormal,
        .stack_size = 512};
    osThreadNew(Motion_Task, NULL, &attr);
}
