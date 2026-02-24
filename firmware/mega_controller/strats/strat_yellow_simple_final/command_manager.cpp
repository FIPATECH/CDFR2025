#include "command_manager.h"
#include "servo_controller.h"
#include <Arduino.h>

// Initialise le module : imprime un message dans le moniteur
void command_manager_init() {
  Serial.println("command_manager_init");
}

// Traite une commande textuelle reçue via UART_CMD_ACTION
// Définition complète de la fonction de traitement
void CommandManager_Process_Command(const char *command)
{
    char action[32] = {0};
    int x = 0, y = 0;

    // Parse "ACTION:<action>:<x>:<y>"
    int ret = sscanf(command, "ACTION:%31[^:]:%d:%d", action, &x, &y);
    if (ret != 3) {
        Serial.print("CommandManager: trame mal formée -> ");
        Serial.println(command);
        return;
    }

    // Log pour debug
    Serial.print("CommandManager: action = ");
    Serial.print(action);
    Serial.print(", x = ");
    Serial.print(x);
    Serial.print(", y = ");
    Serial.println(y);

    // Appel de la fonction correspondant à l'action
    if (strcmp(action, "OPEN_GRIPPER") == 0) {
        servo_open();
    }
    else if (strcmp(action, "CLOSE_GRIPPER") == 0) {
        servo_close();
    }
    else if (strcmp(action, "RELEASE") == 0) {
        // servo_release();
    }
    else if (strcmp(action, "RAISE_ARM") == 0) {
        // servo_raise_arm();
    }
    else if (strcmp(action, "LOWER_ARM") == 0) {
        // servo_lower_arm();
    }
    else if (strcmp(action, "MOVE") == 0) {
        // navigate_to(x, y);  // implémente ta fonction de déplacement
    }
    else {
        Serial.print("CommandManager: action inconnue -> ");
        Serial.println(action);
    }
}
