#include "PID_control.h"
#include <ctype.h>
#include <util/atomic.h>

/* ---------- compat ESP / AVR ---------- */
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/* ---------- Variables codeurs ---------- */
static volatile long countR = 0, countL = 0;
static long prevCountR = 0, prevCountL = 0;

/* ---------- PID params ---------- */
static constexpr float P_R = 1, I_R = 0.3, D_R = 0.07;
static constexpr float P_L = 1, I_L = 0.3, D_L = 0.07;
static float PErrR = 0, PErrL = 0;
static long IErrR = 0, IErrL = 0;

/* ---------- Consignes & rampe ---------- */
static volatile int16_t cmdVD = 0, cmdVG = 0;
static int16_t rampD = 0, rampG = 0;
static float pwmD = 0, pwmG = 0;
static bool pidEnabled = false;

/* ---------- Constantes internes ---------- */
static constexpr float coeff = 100.0f / 50.0f;  // tops/40 ms → tops/s
static constexpr int acc = 20;                  // rampe

/* ---------- Prototypes tâches ---------- */
static void TaskSerial(void *);
static void TaskControl(void *);

/* ---------- ISRs quadrature ---------- */
static void IRAM_ATTR interruptR() {
  if (digitalRead(ENC_RA) == digitalRead(ENC_RB)) ++countR;
  else --countR;
}
static void IRAM_ATTR interruptL() {
  if (digitalRead(ENC_LA) == digitalRead(ENC_LB)) --countL;
  else ++countL;
}

/* ---------- Helper PWM ---------- */
static inline void sendCmd(int16_t left, int16_t right) {
  if (left == 0) analogWrite(PWM_L, 0);
  else {
    digitalWrite(DIR_L, left > 0 ? LOW : HIGH);
    analogWrite(PWM_L, min(abs(left), 255));
  }
  if (right == 0) analogWrite(PWM_R, 0);
  else {
    digitalWrite(DIR_R, right > 0 ? HIGH : LOW);
    analogWrite(PWM_R, min(abs(right), 255));
  }
}

/* ---------- Parsing UART ---------- */
static char buf[32];
static uint8_t idx = 0;
static void decrypt() {
  idx = 0;
  cmdVG = cmdVD = 0;
  bool neg = false;
  if (buf[idx] == '-') {
    neg = true;
    ++idx;
  }
  while (isdigit(buf[idx])) cmdVG = cmdVG * 10 + (buf[idx++] - '0');
  if (neg) cmdVG = -cmdVG;
  ++idx;
  neg = false;
  if (buf[idx] == '-') {
    neg = true;
    ++idx;
  }
  while (isdigit(buf[idx])) cmdVD = cmdVD * 10 + (buf[idx++] - '0');
  if (neg) cmdVD = -cmdVD;
}

/* ========== API Publique ========== */
void PIDControl_SetCommand(int16_t l, int16_t r) {
  taskENTER_CRITICAL();
  cmdVG = l;
  cmdVD = r;
  pidEnabled = (l != 0 || r != 0);
  taskEXIT_CRITICAL();
}

void PIDControl_Init(void) {
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_RA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), interruptL, CHANGE);

  sendCmd(0, 0);
  pidEnabled = false;

  xTaskCreate(TaskSerial, "PID_RX", 128, nullptr, 5, nullptr);
  xTaskCreate(TaskControl, "PID_CTRL", 256, nullptr, 6, nullptr);
}

/* ========== Tasks ========== */

static void TaskSerial(void *) {
  for (;;) {
    while (Serial.available()) {
      int c = Serial.read();
      buf[idx++] = (char)c;
      if (c == '\n') {
        buf[idx] = '\0';
        decrypt();
        pidEnabled = (cmdVG != 0 || cmdVD != 0);
        idx = 0;
      }
    }
    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void TaskControl(void *) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(40);

  for (;;) {
    vTaskDelayUntil(&last, period);

    if (!pidEnabled) {
      sendCmd(0, 0);
      continue;
    }

    long cL, cR;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      cL = countL;
      cR = countR;
    }

    float dL = (cL - prevCountL) * coeff;
    float dR = (cR - prevCountR) * coeff;
    prevCountL = cL;
    prevCountR = cR;

    if (rampG < cmdVG) rampG = min(rampG + acc, cmdVG);
    else if (rampG > cmdVG) rampG = max(rampG - acc, cmdVG);
    if (rampD < cmdVD) rampD = min(rampD + acc, cmdVD);
    else if (rampD > cmdVD) rampD = max(rampD - acc, cmdVD);

    float errR = rampD - dR;
    float derivR = errR - PErrR;
    long IErrR_tmp = IErrR + errR;
    float outR = P_R * errR + I_R * IErrR_tmp + D_R * derivR;
    if (abs(outR) < 255) IErrR = IErrR_tmp;
    pwmD = constrain(outR, -255, 255);
    PErrR = errR;

    float errL = rampG - dL;
    float derivL = errL - PErrL;
    long IErrL_tmp = IErrL + errL;
    float outL = P_L * errL + I_L * IErrL_tmp + D_L * derivL;
    if (abs(outL) < 255) IErrL = IErrL_tmp;
    pwmG = constrain(outL, -255, 255);
    PErrL = errL;

    if (cmdVD == 0) {
      IErrR = 0;
      PErrR = 0;
    }
    if (cmdVG == 0) {
      IErrL = 0;
      PErrL = 0;
    }

    sendCmd((int16_t)pwmG, (int16_t)pwmD);

    /* ----  nouvel envoi dans la queue ---- */
    OdomData_t o = { dR, dL, cR, cL };
    xQueueOverwrite(odomQueue, &o);  // 1 seul slot suffirait mais on a 8

    Serial.print(F("ODOM vR="));
    Serial.print(dR, 2);
    Serial.print(F("\tvL="));
    Serial.print(dL, 2);
    Serial.print(F("\tcountR="));
    Serial.print(cR);
    Serial.print(F("\tcountL="));
    Serial.println(cL);
  }
}
