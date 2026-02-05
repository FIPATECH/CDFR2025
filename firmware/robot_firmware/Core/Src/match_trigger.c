#include "../Inc/match_trigger.h"
#include "../Inc/uart_handler.h"
#include "../Inc/uart_commands.h"
#include "cmsis_os.h"
#include "main.h"

volatile uint8_t matchStopReceived = 0;
volatile uint8_t startMatchFlag = 0;

#define TIRETTE_DEBOUNCE_MS 50

/**
 * @brief Vérifie avec un debounce si le bouton (tirette) est pressé.
 */
uint8_t IsTiretteRemoved(void)
{
  if (HAL_GPIO_ReadPin(TIRETTE_GPIO_PORT, TIRETTE_GPIO_PIN) == GPIO_PIN_RESET)
  {
    osDelay(TIRETTE_DEBOUNCE_MS);
    if (HAL_GPIO_ReadPin(TIRETTE_GPIO_PORT, TIRETTE_GPIO_PIN) == GPIO_PIN_RESET)
    {
      return 1;
    }
  }
  return 0;
}

/**
 * @brief Callback appelée par le module UART lorsqu'une commande STOP_MATCH est reçue.
 */
void MatchTrigger_StopCallback(void)
{
  matchStopReceived = 1;
}

/**
 * @brief Callback d'interruption EXTI appelé par HAL_GPIO_EXTI_IRQHandler.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BTN_Pin) // USER_BTN_Pin est défini comme GPIO_PIN_0 (USER_BTN)
  {
    // Lever le flag pour que la tâche MatchTriggerTask le traite.
    startMatchFlag = 1;
  }
}

/**
 * @brief Tâche FreeRTOS de gestion du déclenchement et de la fin du match.
 * Lorsqu'un appui est détecté (startMatchFlag levé), la LED rouge s'allume,
 * la commande START_MATCH est envoyée, et la tâche attend ensuite la réception de
 * STOP_MATCH (via MatchTrigger_StopCallback) pour éteindre la LED.
 * (La Jetson compte 100s avant d'envoyer STOP)
 */
void MatchTriggerTask(void *argument)
{
  /* Infinite loop */
  for (;;)
  {
    if (startMatchFlag)
    {
      startMatchFlag = 0;

      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);      // Allume la LED rouge
      UART_Encode_And_Send_Message(UART_CMD_START_MATCH, 0, NULL); // Envoie la commande START_MATCH

      // Attente de la réception de la commande STOP_MATCH
      while (matchStopReceived == 0)
      {
        osDelay(10);
      }
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET); // Éteint la LED rouge
      matchStopReceived = 0;
    }
    osDelay(10);
  }
}
