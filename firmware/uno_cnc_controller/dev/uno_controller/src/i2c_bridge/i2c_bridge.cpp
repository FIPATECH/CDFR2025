#include "i2c_bridge.h"

CommandManager* I2CBridge::cmdMgr = nullptr;
String I2CBridge::receivedCmd = "";
bool I2CBridge::hasCmd = false;
uint8_t I2CBridge::ackCode = 0;

I2CBridge::I2CBridge(uint8_t address, CommandManager& mgr)
  : address(address) {
  cmdMgr = &mgr;
}

void I2CBridge::begin() {
  Wire.begin(address);
  Wire.onReceive(I2CBridge::onReceive);
  Wire.onRequest(I2CBridge::onRequest);
}

void I2CBridge::onReceive(int byteCount) {
  (void)byteCount;
  receivedCmd = "";
  while (Wire.available()) receivedCmd += (char)Wire.read();
  receivedCmd.trim();
  hasCmd = true;
  ackCode = 0x01;  // ACK reçu
}

void I2CBridge::process() {
  if (hasCmd) {
    bool ok = cmdMgr->executeCommand(receivedCmd);
    ackCode = ok ? 0x02 : 0xFF;
    hasCmd = false;
  }
}

void I2CBridge::onRequest() {
  Wire.write(ackCode);
  ackCode = 0x00;
}
