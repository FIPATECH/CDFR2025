/* File: odometry.c */

#include "odometry.h"
#include "main.h" // pour les handles TIM
#include "cmsis_os.h"
#include "uart_handler.h"
#include "uart_commands.h"
#include <math.h>
#include <string.h>

#define CPR 1024.0f /* 256 CPR ×4 */
#define ODOM_FREQ_HZ 1000u
#define DT (1.0f / ODOM_FREQ_HZ)
#define FILT_SIZE 5

// Paramètres de détection de glitch
#define MAX_GLITCH 50      /* ticks par pas maxi admis */
#define GLITCH_FACTOR 2.0f /* tolérance en multiple de la moyenne */

// Handles externes déclarés dans main.c
extern TIM_HandleTypeDef htim3; /* encodeur roue gauche */
extern TIM_HandleTypeDef htim4; /* encodeur roue droite */
extern TIM_HandleTypeDef htim1; /* timer de synchronisation odom (1 kHz) */

// Sémaphore de synchronisation
SemaphoreHandle_t odomSem;

// Variables partagées
static volatile int32_t cntL32 = 0, cntR32 = 0;
static volatile float wL = 0, wR = 0;
static volatile pose_t pose = {0};

// Filtre glissant
static int32_t bufDL[FILT_SIZE] = {0}, bufDR[FILT_SIZE] = {0};
static int32_t sumDL = 0, sumDR = 0;
static uint8_t bufPos = 0;

// Logger UART
static uint16_t uartLogPeriodMs = 0, uartLogCounter = 0;

/**
 * diff32 : calcule la différence entre la valeur actuelle et la dernière
 * en gérant le rollover 32 bits.
 */
static inline int32_t diff32(uint32_t now, uint32_t *last)
{
    int64_t raw = (int64_t)now - (int64_t)*last;
    // correction rollover
    if (raw > INT32_MAX)
        raw -= (int64_t)UINT32_MAX + 1;
    if (raw < -INT32_MAX)
        raw += (int64_t)UINT32_MAX + 1;
    *last = now;
    return (int32_t)raw;
}

/**
 * Tâche d’odométrie, déclenchée à 1 kHz par TIM1.
 */
static void Odom_Task(void *arg)
{
    uint32_t lastL = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t lastR = __HAL_TIM_GET_COUNTER(&htim4);

    for (;;)
    {
        // attend le tick TIM1
        xSemaphoreTake(odomSem, portMAX_DELAY);

        // lecture brute encodeurs
        int32_t rawL = diff32(__HAL_TIM_GET_COUNTER(&htim3), &lastL);
        int32_t rawR = diff32(__HAL_TIM_GET_COUNTER(&htim4), &lastR);

        // on inverse le signe pour que raw > 0 => roue avance
        rawL = rawL;
        rawR = -rawR;

        // moyenne courante du filtre
        float avgL = (float)sumDL / FILT_SIZE;
        float avgR = (float)sumDR / FILT_SIZE;

        // détection et atténuation de glitch sur chaque roue
        int32_t dL = (fabsf(rawL - avgL) > MAX_GLITCH * GLITCH_FACTOR)
                         ? (int32_t)roundf(avgL)
                         : rawL;
        int32_t dR = (fabsf(rawR - avgR) > MAX_GLITCH * GLITCH_FACTOR)
                         ? (int32_t)roundf(avgR)
                         : rawR;

        // mise à jour du filtre glissant
        sumDL -= bufDL[bufPos];
        sumDR -= bufDR[bufPos];
        bufDL[bufPos] = dL;
        bufDR[bufPos] = dR;
        sumDL += dL;
        sumDR += dR;
        bufPos = (bufPos + 1) % FILT_SIZE;

        // valeurs filtrées
        // int32_t fL = sumDL / FILT_SIZE;
        // int32_t fR = sumDR / FILT_SIZE;

        // cumul 32 bits
        // cntL32 += fL;
        // cntR32 += fR;

        int32_t fL_i = sumDL / FILT_SIZE; // garde la version entière si tu la veux
        int32_t fR_i = sumDR / FILT_SIZE;
        float fL = (float)sumDL / (float)FILT_SIZE; // version *float*
        float fR = (float)sumDR / (float)FILT_SIZE;

        /* compteurs = delta dé-glitché non moyenné */
        cntL32 += dL;
        cntR32 += dR;

        // vitesses angulaires rad/s
        wL = fL * (2.0f * M_PI) / CPR / DT;
        wR = fR * (2.0f * M_PI) / CPR / DT;

        // intégration de la pose
        float v = WHEEL_RADIUS * 0.5f * (wL + wR);
        float omega = WHEEL_RADIUS / WHEEL_BASE * (wR - wL);

        pose.theta += omega * DT;
        float c = cosf(pose.theta);
        float s = sinf(pose.theta);
        pose.x += v * c * DT;
        pose.y += v * s * DT;

        // envoi UART si activé
        if (uartLogPeriodMs && (++uartLogCounter >= uartLogPeriodMs))
        {
            uartLogCounter = 0;
            uint8_t payload[28];
            int32_t cntL = cntL32, cntR = cntR32;
            float oL = wL, oR = wR,
                  px = pose.x, py = pose.y,
                  th = pose.theta;

            memcpy(&payload[0], &cntL, sizeof(cntL));
            memcpy(&payload[4], &cntR, sizeof(cntR));
            memcpy(&payload[8], &oL, sizeof(oL));
            memcpy(&payload[12], &oR, sizeof(oR));
            memcpy(&payload[16], &px, sizeof(px));
            memcpy(&payload[20], &py, sizeof(py));
            memcpy(&payload[24], &th, sizeof(th));

            UART_Encode_And_Send_Message(UART_CMD_ODOM, sizeof(payload), payload);
        }
    }
}

void Odom_Init(void)
{
    // démarrage des encodeurs
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // création du sémaphore
    odomSem = xSemaphoreCreateBinary();
    configASSERT(odomSem != NULL);

    // démarre TIM1 en interruption 1 kHz
    HAL_TIM_Base_Start_IT(&htim1);

    // création et démarrage de la tâche odom
    const osThreadAttr_t attr = {
        .name = "Odom",
        .priority = osPriorityHigh,
        .stack_size = 512};
    osThreadNew(Odom_Task, NULL, &attr);
}

void Odom_StartUartLogger(uint16_t period_ms)
{
    uartLogPeriodMs = period_ms;
    uartLogCounter = 0;
}

void Odom_GetPose(pose_t *p)
{
    if (p)
        *p = pose;
}

void Odom_GetWheelSpeeds(float *l, float *r)
{
    if (l)
        *l = wL;
    if (r)
        *r = wR;
}

void Odom_GetCounts(int32_t *l, int32_t *r)
{
    if (l)
        *l = cntL32;
    if (r)
        *r = cntR32;
}

void Odom_ResetPose(float x0, float y0, float t0)
{
    pose.x = x0;
    pose.y = y0;
    pose.theta = t0;
    cntL32 = cntR32 = 0;
    sumDL = sumDR = 0;
    memset(bufDL, 0, sizeof(bufDL));
    memset(bufDR, 0, sizeof(bufDR));
    bufPos = 0;
}
