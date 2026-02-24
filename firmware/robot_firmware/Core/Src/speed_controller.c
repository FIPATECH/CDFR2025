#include "speed_controller.h" // <--- Assurez-vous que ce chemin est correct
#include "motor_controller.h"
#include "odometry.h"
#include "cmsis_os2.h"
#include "uart_handler.h"
#include "uart_commands.h"
#include <math.h>

/* variables statiques */
static volatile float refL_t = 0, refR_t = 0;
static uint16_t pidLogMs = UART_LOG_MS, pidLogCnt = 0;

static void Speed_Task(void *arg)
{
    TickType_t next = osKernelGetTickCount();
    TickType_t period = osKernelGetTickFreq() / (uint32_t)LOOP_HZ;

    float refL_s = 0, refR_s = 0;
    float cmdL = 0, cmdR = 0, ffL = 0, ffR = 0;
    float vl = 0, vr = 0, integL = 0, integR = 0;
    float prev_vl = 0, prev_vr = 0;

    for (;;)
    {
        /* 1) Rampe de consigne */
        if (refL_s + REF_SLEW < refL_t)
            refL_s += REF_SLEW;
        else if (refL_s - REF_SLEW > refL_t)
            refL_s -= REF_SLEW;
        else
            refL_s = refL_t;
        if (refR_s + REF_SLEW < refR_t)
            refR_s += REF_SLEW;
        else if (refR_s - REF_SLEW > refR_t)
            refR_s -= REF_SLEW;
        else
            refR_s = refR_t;

        /* 2) Mesure + filtre */
        float rawL, rawR;
        Odom_GetWheelSpeeds(&rawL, &rawR);
        vl += VEL_ALPHA * (rawL - vl);
        vr += VEL_ALPHA * (rawR - vr);

        /* 3) Feed-forward lissé */
        float ffLt = K_FF * refL_s;
        ffL += FF_ALPHA * (ffLt - ffL);
        float ffRt = K_FF * refR_s;
        ffR += FF_ALPHA * (ffRt - ffR);

        /* 4) PID gauche */
        float errL = refL_s - vl;
        integL += errL * DT;
        float iMaxL = OUT_MAX / (KI_L > 0 ? KI_L : 1.0f);
        integL = fminf(fmaxf(integL, -iMaxL), iMaxL);
        float dL = (vl - prev_vl) / DT;
        prev_vl = vl;
        float outL = ffL + KP_L * errL + KI_L * integL - KD_L * dL;

        /* 5) PID droite */
        float errR = refR_s - vr;
        integR += errR * DT;
        float iMaxR = OUT_MAX / (KI_R > 0 ? KI_R : 1.0f);
        integR = fminf(fmaxf(integR, -iMaxR), iMaxR);
        float dR = (vr - prev_vr) / DT;
        prev_vr = vr;
        float outR = ffR + KP_R * errR + KI_R * integR - KD_R * dR;

        /* 6) Plateau-hold */
        if (fabsf(errL) < ERR_HOLD)
            outL = cmdL;
        if (fabsf(errR) < ERR_HOLD)
            outR = cmdR;

        /* 7) Clamp */
        outL = fminf(fmaxf(outL, -OUT_MAX), OUT_MAX);
        outR = fminf(fmaxf(outR, -OUT_MAX), OUT_MAX);

        /* 8) Slew-rate PWM */
        cmdL += SLEW_ALPHA * (outL - cmdL);
        cmdR += SLEW_ALPHA * (outR - cmdR);

        /* 9) Envoi PWM */
        MotorController_SetSpeed(MOTOR_LEFT,
                                 (uint8_t)fabsf(cmdL),
                                 (cmdL >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        MotorController_SetSpeed(MOTOR_RIGHT,
                                 (uint8_t)fabsf(cmdR),
                                 (cmdR >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        /* 10) Logger UART */
        if (pidLogMs && (++pidLogCnt >= pidLogMs))
        {
            pidLogCnt = 0;
            float buf[6] = {refL_s, vl, refR_s, vr, cmdL, cmdR};
            UART_Encode_And_Send_Message(
                UART_CMD_PID, sizeof(buf), (uint8_t *)buf);
        }

        /* 11) Attente */
        next += period;
        osDelayUntil(next);
    }
}

void SpeedController_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "SpeedPID",
        .priority = osPriorityHigh,
        .stack_size = 512};
    osThreadNew(Speed_Task, NULL, &attr);
}

void SpeedController_SetReference(float wl, float wr)
{
    refL_t = CORR_L * wl;
    refR_t = CORR_R * wr;
}

void SpeedController_StartUartLogger(uint16_t period_ms)
{
    pidLogMs = period_ms;
    pidLogCnt = 0;
}
