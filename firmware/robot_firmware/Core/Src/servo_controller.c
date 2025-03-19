#include "servo_controller.h"

extern TIM_HandleTypeDef htim3;

/**
 * @brief Convertit un angle (0 à 180°) en largeur d'impulsion en microsecondes.
 * Pour un servo à rotation continue, on utilise :
 *  - 0°  -> SERVO_MIN_PULSE (pour tourner à pleine vitesse dans un sens)
 *  - 90° -> SERVO_STOP_PULSE (pour arrêter)
 *  - 180° -> SERVO_MAX_PULSE (pour tourner à pleine vitesse dans l'autre sens)
 */
static uint32_t AngleToPulse(uint8_t angle)
{
  if (angle > 180)
    angle = 180;
  return SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180;
}

void Servo_Init(void)
{
  // Démarre le PWM sur TIM3, Canal 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void SetServoAngle(uint8_t angle)
{
  uint32_t pulse = AngleToPulse(angle);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

/**
 * @brief Exécute l'action OPEN_GRIPPER : fait tourner le servo pendant 5 secondes puis arrête le servo.
 */
void Execute_Open_Gripper(void)
{
  // Commande pour tourner : ici, on choisit 0° (pulse = 1000 µs) pour "ouvrir la pince"
  SetServoAngle(0);
  // Attendre 5 secondes
  osDelay(5000);
  // Arrêter le servo en envoyant le signal neutre (90° -> 1500 µs)
  SetServoAngle(90);
}
