#include "command_manager.h"

static const CommandManager::Command commands[] = {
  { "open_rack", rack_motor_deploy },
  { "close_rack", rack_motor_retract },
  { "lift_platform", lift_platform_up },
  { "lower_platform", lift_platform_down },
  { "test_xy", test_xy },
  { "test_x", test_x },
  { "test_y", test_y },
  { "test_z", test_z }
};
static const size_t numCommands = sizeof(commands) / sizeof(commands[0]);

void CommandManager::begin() {
}

bool CommandManager::executeCommand(const String& cmd) {
  for (size_t i = 0; i < numCommands; ++i) {
    if (cmd.equals(commands[i].name)) {
      commands[i].fn();
      return true;
    }
  }
  return false;
}
