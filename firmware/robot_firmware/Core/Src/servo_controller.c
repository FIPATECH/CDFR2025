#include "servo_controller.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim9;  // Servo 1 sur TIM9_CH2 (PE6)
extern TIM_HandleTypeDef htim10; // Servo 2 sur TIM10_CH1 (PF6)

/**
 * @brief Initialise le PWM pour les deux servos.
 */
void ServoController_Init(void)
{
  // Démarre TIM9_CH2 (PE6)
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  // Démarre TIM10_CH1 (PF6)
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  // Position neutre des deux servos
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, NEUTRAL_POSITION_LEFT);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, NEUTRAL_POSITION_RIGHT);
}

/**
 * @brief Ouvre la pince :
 *        - servo gauche (TIM9_CH2) vers OPEN_POSITION_LEFT
 *        - servo droit   (TIM10_CH1) vers OPEN_POSITION_RIGHT
 *        attend OPEN_GRIPPER_DURATION ms,
 *        puis revient en neutre.
 */
void Execute_Open_Gripper(void)
{
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, OPEN_POSITION_LEFT);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, OPEN_POSITION_RIGHT);

  osDelay(OPEN_GRIPPER_DURATION);

  // Retour en position neutre
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, NEUTRAL_POSITION_LEFT);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, NEUTRAL_POSITION_RIGHT);

  // TODO: envoyer ACK OPEN_GRIPPER
}

/**
 * @brief Ferme la pince :
 *        - servo gauche (TIM9_CH2) vers CLOSE_POSITION_LEFT
 *        - servo droit   (TIM10_CH1) vers CLOSE_POSITION_RIGHT
 *        attend CLOSE_GRIPPER_DURATION ms,
 *        puis revient en neutre.
 */
void Execute_Close_Gripper(void)
{
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, CLOSE_POSITION_LEFT);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, CLOSE_POSITION_RIGHT);

  osDelay(CLOSE_GRIPPER_DURATION);

  // Retour en position neutre
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, NEUTRAL_POSITION_LEFT);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, NEUTRAL_POSITION_RIGHT);

  // TODO: envoyer ACK CLOSE_GRIPPER
}
