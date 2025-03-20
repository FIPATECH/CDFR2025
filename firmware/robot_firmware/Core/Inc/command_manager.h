#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"

#define ACTION_MSG_SIZE 32
    // Déclaration de la file pour stocker les commandes d'action
    extern osMessageQueueId_t actionQueueHandle;

    // Fonction appelée pour traiter une commande reçue
    void CommandManager_Process_Command(const char *command);

    // Tâche dédiée au traitement des commandes d'action
    void Action_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_MANAGER_H
