#include "match_trigger.h"
#include "uart_handler.h"
#include "uart_commands.h"
#include "i2c_handler.h"
#include "command_manager.h"
#include "servo_controller.h"
#include "cmsis_os.h"
#include "main.h"

volatile uint8_t matchStopReceived = 0;
volatile uint8_t startMatchFlag = 0;

#define BLUE_BTN_DEBOUNCE_MS 50

uint8_t IsTiretteRemoved(void)
{
  if (HAL_GPIO_ReadPin(BLUE_BTN_GPIO_PORT, BLUE_BTN_GPIO_PIN) == GPIO_PIN_RESET)
  {
    osDelay(BLUE_BTN_DEBOUNCE_MS);
    if (HAL_GPIO_ReadPin(BLUE_BTN_GPIO_PORT, BLUE_BTN_GPIO_PIN) == GPIO_PIN_RESET)
      return 1;
  }
  return 0;
}

void MatchTrigger_StopCallback(void)
{
  matchStopReceived = 1;
}

void MatchTrigger_EXTI(uint16_t pin)
{
  if (pin == USER_BTN_Pin)
  {
    if (startMatchFlag == 0)
      startMatchFlag = 1;
  }
}

void MatchTriggerTask(void *argument)
{
  for (;;)
  {
    if (startMatchFlag)
    {
      startMatchFlag = 0;

      // Allume la LED rouge
      // HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);

      // Envoie la commande START_MATCH sur l’UART
      UART_Encode_And_Send_Message(UART_CMD_START_MATCH, 0, NULL);

      // Envoie "test_xy" sur l’Arduino via I²C
      I2C_Cmd_PlatformUp();
      Execute_Open_Gripper();

          // Attend la commande STOP_MATCH
          while (!matchStopReceived)
              osDelay(10);

      // Éteint la LED rouge et réarme le flag
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      matchStopReceived = 0;
    }
    osDelay(10);
  }
}
