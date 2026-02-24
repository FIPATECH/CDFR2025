#include "command_manager.h"
#include "servo_controller.h"
#include "uart_commands.h"
#include "uart_handler.h"
#include "match_trigger.h" // pour extern volatile uint8_t startMatchFlag
#include <string.h>
#include <stdio.h>

osMessageQueueId_t actionQueueHandle;

void Action_Task(void *argument)
{
  char actionMsg[ACTION_MSG_SIZE];
  for (;;)
  {
    if (osMessageQueueGet(actionQueueHandle,
                          &actionMsg,
                          NULL,
                          osWaitForever) == osOK)
    {
      if (strcmp(actionMsg, "OPEN_GRIPPER") == 0)
        Execute_Open_Gripper();
      else if (strcmp(actionMsg, "CLOSE_GRIPPER") == 0)
        Execute_Close_Gripper();
      // … autres ACTION:… éventuelles …
    }
  }
}

void CommandManager_Process_Command(const char *command)
{
  char action[32];
  int x, y;
  if (sscanf(command, "ACTION:%31[^:]:%d:%d", action, &x, &y) == 3)
  {
    // Poste juste le nom de l’action
    osMessageQueuePut(actionQueueHandle, action, 0, 0);
  }
  else if (strcmp(command, "start\n") == 0)
  {
    extern volatile uint8_t startMatchFlag;
    if (startMatchFlag == 0)
      startMatchFlag = 1;
  }
  else
  {
    // Format inconnu, ignorer ou logger
  }
}
