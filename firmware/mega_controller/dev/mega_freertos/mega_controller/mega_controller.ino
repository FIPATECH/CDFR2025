#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h>

#include "globals.h"
#include "uart_handler.h"
#include "uart_commands.h"  // pour UART_CMD_POINTS
#include "command_manager.h"
#include "servo_controller.h"
#include "i2c_handler.h"
#include "match_trigger.h"
#include "points_manager.h"

// Flags partagés
volatile bool matchStartRequested = false;
volatile bool matchStopReceived   = false;

/* ---------- PINOUT -------------------------------------------------- */
#define encRA 2
#define encLA 3
#define encRB 4
#define encLB 5
#define DIR_R  8
#define PWM_R  9
#define DIR_L  7
#define PWM_L 10

/* ---------- Variables encodeurs ------------------------------------- */
volatile long countR = 0, countL = 0;
long prevCountR = 0, prevCountL = 0;

/* ---------- PID Gains ---------------------------------------------- */
// roue droite
const float P_R = 1;
const float I_R = 0.3;
const float D_R = 0.07;
// roue gauche
const float P_L = 1;
const float I_L = 0.3;
const float D_L = 0.07;

/* ---------- États du PID --------------------------------------------- */
float PErrR = 0, PErrL = 0;
long  IErrR   = 0, IErrL   = 0;

/* ---------- Commandes ------------------------------------------------ */
volatile int cmdVD = 0, cmdVG = 0;  // consignes reçues via USB-Serial
int rampD = 0, rampG = 0;
float pwmD = 0, pwmG = 0;

/* ---------- Params --------------------------------------------------- */
const float coeff = 100.0f / 50.0f;
const int   acc   = 20;

/* ---------- Buffer série --------------------------------------------- */
char     streamChar[32];
uint8_t  idx          = 0;
int      incomingByte = 0;

// Prototypes des tâches externes
extern void uart_task(void *pvParameters);
extern void servo_task(void *pvParameters);
extern void match_trigger_task(void *pvParameters);

// Prototypes des tâches internes
void TaskSerial(void *pvParameters);
void TaskControlLoop(void *pvParameters);

// Prototypes utilitaires
void sendFrameSerial2(uint16_t function, uint16_t length, const uint8_t *payload);
void decryptIncom();
void sendCmd(int left, int right);

// ---------------------------------------------------
// Interruptions quadrature (ISR très légère)
// ---------------------------------------------------
void interruptR() {
  if (digitalRead(encRA) == digitalRead(encRB)) countR++;
  else                                        countR--;
}
void interruptL() {
  if (digitalRead(encLA) == digitalRead(encLB)) countL--;
  else                                        countL++;
}

// ---------------------------------------------------
// Envoi trame UART vers STM32 puis log (non critique)
// ---------------------------------------------------
void sendFrameSerial2(uint16_t function, uint16_t length, const uint8_t *payload) {
  uint8_t buf[20];
  uint16_t pos = 0;
  buf[pos++] = 0x4A;
  buf[pos++] = (function >> 8) & 0xFF;
  buf[pos++] = function & 0xFF;
  buf[pos++] = (length >> 8) & 0xFF;
  buf[pos++] = length & 0xFF;
  if (length && payload) {
    memcpy(&buf[pos], payload, length);
    pos += length;
  }
  uint8_t cs = buf[0];
  for (uint16_t i = 1; i < pos; i++) cs ^= buf[i];
  buf[pos++] = cs;
  Serial2.write(buf, pos);
  Serial.print("Serial2 TX [");
  for (uint16_t i = 0; i < pos; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println("]");
}

// ---------------------------------------------------
// Décryptage des consignes reçues sur USB-Serial
// ---------------------------------------------------
void decryptIncom() {
  idx = 0; cmdVG = cmdVD = 0;
  bool neg = false;
  if (streamChar[idx] == '-') { neg = true; idx++; }
  while (isdigit(streamChar[idx])) cmdVG = cmdVG*10 + (streamChar[idx++] - '0');
  if (neg) cmdVG = -cmdVG;
  idx++;
  neg = false;
  if (streamChar[idx] == '-') { neg = true; idx++; }
  while (isdigit(streamChar[idx])) cmdVD = cmdVD*10 + (streamChar[idx++] - '0');
  if (neg) cmdVD = -cmdVD;
}

// ---------------------------------------------------
// Commande moteurs (direction + PWM)
// ---------------------------------------------------
void sendCmd(int left, int right) {
  // Moteur gauche (inversé)
  if (left == 0) {
    analogWrite(PWM_L, 0);
  } else {
    digitalWrite(DIR_L, left > 0 ? LOW : HIGH);
    analogWrite(PWM_L, min(abs(left), 255));
  }
  // Moteur droit
  if (right == 0) {
    analogWrite(PWM_R, 0);
  } else {
    digitalWrite(DIR_R, right > 0 ? HIGH : LOW);
    analogWrite(PWM_R, min(abs(right), 255));
  }
}

// ---------------------------------------------------
// setup()
// ---------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F(">> USB Serial OK"));

  Serial2.begin(115200);
  Serial.println(F(">> Serial2 (STM32) @115200"));

  Serial3.begin(115200);
  Serial.println(F(">> Serial3 (Jetson) @115200"));

  Wire.begin();

  uart_init();
  command_manager_init();
  servo_init();
  i2c_handler_init(0x07);

  // Config. pins moteurs et encodeurs
  pinMode(DIR_R, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT); pinMode(PWM_L, OUTPUT);
  pinMode(encRA, INPUT_PULLUP);
  pinMode(encRB, INPUT_PULLUP);
  pinMode(encLA, INPUT_PULLUP);
  pinMode(encLB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLA), interruptL, CHANGE);

  // Création des tâches FreeRTOS
  xTaskCreate(TaskSerial,          "Serial_RX",   128, NULL, 1, NULL);
  xTaskCreate(uart_task,           "UART_RX",     512, NULL, 2, NULL);
  xTaskCreate(servo_task,          "SERVO",       256, NULL, 3, NULL);
  xTaskCreate(match_trigger_task,  "MATCH_TRIG",  512, NULL, 3, NULL);
  PointsManager_Init(); // Task interne prio 5

  // Tâche unique de contrôle (mesure+rampe+PID)
  xTaskCreate(TaskControlLoop,     "ControlLoop", 256, NULL, 6, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Ne doit jamais être atteint sous FreeRTOS
}

// ---------------------------------------------------
// TaskSerial: réception asynchrone USB-Serial (~100 Hz)
// ---------------------------------------------------
void TaskSerial(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    if (Serial.available()) {
      incomingByte      = Serial.read();
      streamChar[idx++] = (char)incomingByte;
      if (incomingByte == '\n') {
        streamChar[idx] = '\0';
        decryptIncom();
        idx = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------------------------------------------
// TaskControlLoop: mesure, rampe et PID (~25 Hz, prio 6)
// ---------------------------------------------------
void TaskControlLoop(void *pvParameters) {
  (void)pvParameters;
  const TickType_t period   = pdMS_TO_TICKS(40);
  TickType_t       lastWake = xTaskGetTickCount();

  for (;;) {
    // Timing strict
    vTaskDelayUntil(&lastWake, period);

    // Mesure
    long deltaR = countR - prevCountR;
    long deltaL = countL - prevCountL;
    prevCountR  = countR;
    prevCountL  = countL;
    float speedR = deltaR * coeff;
    float speedL = deltaL * coeff;

    // Rampe
    rampD += constrain(cmdVD - rampD, -acc, acc);
    rampG += constrain(cmdVG - rampG, -acc, acc);

    // PID droite
    float errR    = rampD - speedR;
    float derivR  = errR - PErrR;
    long  iTmpR   = IErrR + errR;
    float uR      = P_R*errR + I_R*iTmpR + D_R*derivR;
    if (abs(uR) < 255) IErrR = iTmpR;
    pwmD         = constrain(uR, -255, 255);
    PErrR        = errR;
    if (cmdVD == 0) { IErrR = 0; PErrR = 0; }

    // PID gauche
    float errL    = rampG - speedL;
    float derivL  = errL - PErrL;
    long  iTmpL   = IErrL + errL;
    float uL      = P_L*errL + I_L*iTmpL + D_L*derivL;
    if (abs(uL) < 255) IErrL = iTmpL;
    pwmG         = constrain(uL, -255, 255);
    PErrL        = errL;
    if (cmdVG == 0) { IErrL = 0; PErrL = 0; }

    // Application des commandes moteurs
    sendCmd((int)pwmG, (int)pwmD);
  }
}
