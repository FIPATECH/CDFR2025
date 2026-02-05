#include "command_manager.h"
#include "servo_controller.h"
#include "uart_commands.h"
#include "uart_handler.h"
#include <string.h>
#include <stdio.h>

osMessageQueueId_t actionQueueHandle;

// Tâche dédiée qui attend dans la file et exécute les actions
void Action_Task(void *argument)
{
  char actionMsg[ACTION_MSG_SIZE] = {0};
  for (;;)
  {
    // Attendre qu'une commande soit reçue
    if (osMessageQueueGet(actionQueueHandle, &actionMsg, NULL, osWaitForever) == osOK)
    {
      if (strcmp(actionMsg, "OPEN_GRIPPER") == 0)
      {
        Execute_Open_Gripper();

        // TODO:
        // const char ack_msg[] = "ACK:OPEN_GRIPPER";
        // UART_Encode_And_Send_Message(UART_CMD_ACTION_ACK, strlen(ack_msg), (const uint8_t *)ack_msg);
      }
    }
  }
}

// Fonction de traitement d'une commande reçue
void CommandManager_Process_Command(const char *command)
{
  // Attente d'une trame de la forme "ACTION:<action>:<x>:<y>"
  char action[32] = {0};
  int x = 0, y = 0;
  int ret = sscanf(command, "ACTION:%31[^:]:%d:%d", action, &x, &y);
  if (ret == 3)
  {
    if (strcmp(action, "OPEN_GRIPPER") == 0)
    {
      // Poster la commande dans la file pour être traitée par la tâche Action_Task
      if (osMessageQueuePut(actionQueueHandle, action, 0, 0) != osOK)
      {
        // TODO: Gérer une erreur d'envoi dans la file (file pleine?)
      }
    }
    else
    {
      // Gérer d'autres actions -> SWITCH CASE
    }
  }
  else
  {
    // Format de commande invalide
  }
}
