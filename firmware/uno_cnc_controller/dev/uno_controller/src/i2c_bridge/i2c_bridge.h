#ifndef I2C_BRIDGE_H
#define I2C_BRIDGE_H

#include <Wire.h>
#include "../command_manager/command_manager.h"

class I2CBridge {
public:
  I2CBridge(uint8_t address, CommandManager& mgr);
  void begin();
  void process();

private:
  static void onReceive(int);
  static void onRequest();

  static CommandManager* cmdMgr;
  static String receivedCmd;
  static bool hasCmd;
  static uint8_t ackCode;

  uint8_t address;
};

#endif // I2C_BRIDGE_H
