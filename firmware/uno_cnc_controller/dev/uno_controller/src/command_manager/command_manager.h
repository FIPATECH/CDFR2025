#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <Arduino.h>
#include "../stepper_motors/stepper_motors.h"

class CommandManager {
public:
  using CmdFn = void (*)();

  struct Command {
    const char* name;
    CmdFn fn;
  };

  void begin();
  bool executeCommand(const String& cmd);
};

#endif  // COMMAND_MANAGER_H
