#include <Wire.h>
#include <AltSoftSerial.h>

// Adresse I2C de la Mega
static const uint8_t MEGA_I2C_ADDR = 0x08;

// Protocole UART
static const uint8_t HEADER_BYTE = 0x4A;
static const uint8_t CMD_MOVE    = 0x10;
static const uint8_t CMD_PING    = 0x20;

// UART Nano ↔ Mega
AltSoftSerial megaSerial; // D8=RX, D9=TX

// CRC-16-CCITT
uint16_t crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

// Envoi trame UART (avec log)
void sendFrame(uint8_t func, const uint8_t *pl, uint8_t len) {
  uint8_t buf[3 + len + 2];
  buf[0] = HEADER_BYTE;
  buf[1] = func;
  buf[2] = len;
  if (len) memcpy(&buf[3], pl, len);
  uint16_t crc = crc16(buf, 3 + len);
  buf[3 + len] = crc >> 8;
  buf[4 + len] = crc & 0xFF;

  Serial.print(">> Envoi frame: ");
  for (uint8_t i = 0; i < 3 + len; i++) {
    Serial.print("0x"); Serial.print(buf[i], HEX); Serial.print(' ');
  }
  Serial.print("| CRC=0x"); Serial.println(crc, HEX);

  megaSerial.write(buf, 5 + len);
}

void sendMove(int16_t v_l, int16_t v_r) {
  uint8_t payload[4] = {
    (uint8_t)(v_l >> 8), (uint8_t)v_l,
    (uint8_t)(v_r >> 8), (uint8_t)v_r
  };
  sendFrame(CMD_MOVE, payload, 4);
  Serial.print("[UART] MOVE envoyé v_l=");
  Serial.print(v_l);
  Serial.print(" v_r=");
  Serial.println(v_r);
}

void sendPing() {
  sendFrame(CMD_PING, nullptr, 0);
  Serial.println("[UART] PING envoyé");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // I²C master
  Wire.begin();
  Serial.println("Nano: I2C ready");

  // UART master vers Mega
  megaSerial.begin(9600);
  Serial.println("Nano: UART AltSoftSerial @9600 ready");
}

void loop() {
  // 1) Test UART
  sendMove(150, -150);
  delay(50);
  sendPing();
  delay(50);

  // 2) Test I²C
  Wire.beginTransmission(MEGA_I2C_ADDR);
  Wire.write(0xAA);
  Wire.endTransmission();
  Serial.println("[I2C] Commande 0xAA envoyée");

  // Lecture status
  uint8_t nb = 1;
  Wire.requestFrom(MEGA_I2C_ADDR, nb);
  if (Wire.available()) {
    uint8_t status = Wire.read();
    Serial.print("[I2C] Status reçu: 0x");
    Serial.println(status, HEX);
  } else {
    Serial.println("[I2C] Pas de status reçu");
  }

  delay(1000);
}
