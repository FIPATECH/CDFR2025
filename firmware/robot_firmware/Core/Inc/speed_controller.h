/* speed_controller.h ------------------------------------------------------*/
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <stdint.h>

/* fréquence de boucle et pas de temps */
#define LOOP_HZ 50.0f
#define DT (1.0f / LOOP_HZ)

/* limites de sortie */
#define OUT_MAX 80.0f
#define MAX_RAD_S 12.0f /* consigne max en rad/s */

/* feed-forward */
#define K_FF (0.25f * OUT_MAX / MAX_RAD_S)
#define FF_ALPHA 0.06f

/* filtrage de la mesure */
#define VEL_ALPHA 0.50f

/* rampe de consigne (rad/s par itération) */
#define REF_SLEW 1.00f

/* slew-rate de la commande PWM */
#define SLEW_ALPHA 0.85f

/* seuil pour maintenir la commande en plateau */
#define ERR_HOLD 0.10f

/* Correction mécanique (roue droite boostée) */
#define CORR_L 1.00f
#define CORR_R 1.3f // +10% sur la consigne droite

/* logger UART : périodicité en ms (0 = off) */
#define UART_LOG_MS 20

/* ----- GAINS PID À TUNER ----- */
/* Roue gauche */
#define KP_L 2.50f
#define KI_L 0.20f
#define KD_L 0.10f

/* Roue gauche */
#define KP_R 2.50f
#define KI_R 0.20f
#define KD_R 0.10f

// /* Roue droite */
// #define KP_R 3.10f // un peu plus de P pour la droite
// #define KI_R 0.35f // un peu plus d’I
// #define KD_R 0.10f
/* ------------------------------ */

#ifdef __cplusplus
extern "C"
{
#endif

    void SpeedController_Init(void);
    void SpeedController_SetReference(float omega_left, float omega_right);
    void SpeedController_StartUartLogger(uint16_t period_ms);

#ifdef __cplusplus
}
#endif
#endif /* SPEED_CONTROLLER_H */
