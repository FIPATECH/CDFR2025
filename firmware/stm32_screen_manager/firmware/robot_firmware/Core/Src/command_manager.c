
#include "command_manager.h"
#include "uart_commands.h"
#include "uart_handler.h"
#include <string.h>
#include <stdio.h>

osMessageQueueId_t actionQueueHandle = NULL;

void Action_Task(void *argument)
{
  char actionMsg[ACTION_MSG_SIZE] = {0};

  for (;;)
  {
    if (osMessageQueueGet(actionQueueHandle,
                          &actionMsg,
                          NULL,
                          osWaitForever) == osOK)
    {
      /* TODO: execute action */
    }
  }
}

void CommandManager_Process_Command(const char *command)
{
  char action[32] = {0};
  int x = 0, y = 0;

  int ret = sscanf(command, "ACTION:%31[^:]:%d:%d", action, &x, &y);
  if (ret == 3)
  {
    if (strcmp(action, "OPEN_GRIPPER") == 0)
    {
      (void)osMessageQueuePut(actionQueueHandle, action, 0, 0);
    }

    else if (strcmp(action, "CLOSE_GRIPPER") == 0)
    {
      (void)osMessageQueuePut(actionQueueHandle, action, 0, 0);
    }
    else 
    {
      char msg[64];
      snprintf(msg, sizeof(msg), "Unknown action: %s", action);
      UART_Send_Raw(msg);
    }
  }
}
