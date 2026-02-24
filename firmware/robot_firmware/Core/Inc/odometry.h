/* File: odometry.h */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif

// --- Paramètres robot différentiel (mêmes unités que dans Motion & Speed) ---
#define WHEEL_BASE 0.165f    /**< distance entre roues [m] */
#define WHEEL_RADIUS 0.0422f /**< rayon de roue [m] */

    typedef struct
    {
        float x;
        float y;
        float theta;
    } pose_t;

    /** Sémaphore utilisé pour synchroniser le tick TIM1 à 1 kHz */
    extern SemaphoreHandle_t odomSem;

    /**
     * @brief  Initialise le module d’odométrie :
     *         - démarre les encodeurs TIM3/TIM4
     *         - crée le sémaphore de synchronisation
     *         - démarre TIM1 en interruption 1 kHz
     *         - crée et démarre la tâche FreeRTOS de calcul
     */
    void Odom_Init(void);

    /**
     * @brief  Active l’envoi périodique de la pose sur UART
     * @param  period_ms  Périodicité en millisecondes (0 pour désactiver)
     */
    void Odom_StartUartLogger(uint16_t period_ms);

    /**
     * @brief  Récupère la pose courante
     * @param  p  Pointeur vers structure pose_t à remplir
     */
    void Odom_GetPose(pose_t *p);

    /**
     * @brief  Récupère les vitesses angulaires des roues
     * @param  l  Pointeur pour vitesse roue gauche (rad/s)
     * @param  r  Pointeur pour vitesse roue droite (rad/s)
     */
    void Odom_GetWheelSpeeds(float *l, float *r);

    /**
     * @brief  Récupère les compteurs cumulés 32 bits des encodeurs
     * @param  l  Pointeur pour compteur roue gauche
     * @param  r  Pointeur pour compteur roue droite
     */
    void Odom_GetCounts(int32_t *l, int32_t *r);

    /**
     * @brief  Réinitialise la pose et les compteurs à une valeur donnée
     * @param  x0  Position X initiale (m)
     * @param  y0  Position Y initiale (m)
     * @param  t0  Orientation initiale (rad)
     */
    void Odom_ResetPose(float x0, float y0, float t0);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
