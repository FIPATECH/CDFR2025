#include "command_manager.h"
#include "servo_controller.h"
#include <Arduino.h>

// Initialise le module : imprime un message dans le moniteur
void command_manager_init() {
  Serial.println("command_manager_init");
}

// Traite une commande textuelle reçue via UART_CMD_ACTION
void CommandManager_Process_Command(const char *command) {
  Serial.print("CommandManager: ACTION -> ");
  Serial.println(command);

  if (strcmp(command, "OPEN_GRIPPER") == 0) {
    servo_open();
  } else if (strcmp(command, "CLOSE_GRIPPER") == 0) {
    servo_close();
  } else {
    Serial.println("CommandManager: commande inconnue");
  }
}
