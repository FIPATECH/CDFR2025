#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"

#define ACTION_MSG_SIZE 64

    extern osMessageQueueId_t actionQueueHandle;

    /**
     * @brief Tâche qui récupère les commandes dans la file d'action
     *        et exécute les actions correspondantes.
     */
    void Action_Task(void *argument);

    /**
     * @brief   Traite une commande brute reçue :
     *          - Si format ACTION:<name>:x:y → poste en file pour Action_Task
     *          - Sinon → exécution immédiate via I²C
     */
    void CommandManager_Process_Command(const char *command);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_MANAGER_H */
