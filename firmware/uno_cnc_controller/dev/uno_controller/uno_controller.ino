#include <Wire.h>
#include "src/stepper_motors/stepper_motors.h"
#include "src/i2c_bridge/i2c_bridge.h"
#include "src/command_manager/command_manager.h"

CommandManager cmdMgr;
I2CBridge i2cBridge(0x10, cmdMgr);

void setup()
{
  Serial.begin(115200);
  setupSteppers();

  cmdMgr.begin();
  i2cBridge.begin();
}

void loop()
{
  i2cBridge.process();

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.println("ACK_RECEIVED (SERIAL)");
    bool ok = cmdMgr.executeCommand(cmd);
    if (ok)
      Serial.println("ACK_EXECUTED (SERIAL)");
    else
      Serial.println("ERR_UNKNOWN_CMD");
  }
}
