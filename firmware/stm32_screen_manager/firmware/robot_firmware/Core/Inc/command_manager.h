
#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"

#define ACTION_MSG_SIZE 32

    extern osMessageQueueId_t actionQueueHandle;

    void CommandManager_Process_Command(const char *command);
    void Action_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_MANAGER_H */
