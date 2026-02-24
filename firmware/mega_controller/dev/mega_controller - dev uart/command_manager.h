#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Arduino.h>

// Flag levé par UART_CMD_STOP_MATCH
extern volatile bool matchStopReceived;

// Initialise le module Command Manager
void command_manager_init();

// Traitement des commandes textuelles reçues (ACTION)
void CommandManager_Process_Command(const char *command);

#endif // COMMAND_MANAGER_H
