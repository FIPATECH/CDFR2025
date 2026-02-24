/* ---------------------------------------------------------------
   Robot différentiel – PID + Odométrie
   Séquence finale (toutes les pauses = 2 s sauf indication) :
     1)  recule 12 cm
     2)  open_rack → banner_down → banner_up → close_rack
     3)  avance 45 cm (axe Y)
     4)  pause
     5)  rotation −90° (horaire)
     6)  pause
     7)  avance 10 cm (relatif)
     8)  pause
     9)  rotation +90° (antihoraire)
    10)  pause
    11)  open_grap   (I²C)
    12)  pause
    13)  avance 20 cm (relatif)
    14)  pause
    15)  setZ_cm (24.5) (I²C) – pause 4 s
    16)  recule 20 cm (relatif)
    17)  pause
    18)  rotation −90° (horaire)
    19)  pause
    20)  avance 20 cm (relatif)
    21)  pause
    22)  rotation −90° (horaire)
    23)  pause
    24)  avance 38 cm (relatif)
    25)  pause
    26)  setZ_cm (22.0) (I²C)
    27)  pause
    28)  recule 30 cm (relatif)
    29)  pause
    30)  close_grap  (I²C)
    31)  rotation −190° (horaire)
    32)  pause
    33)  avance 80 cm (relatif)
    34)  pause
    35)  rotation +10° (antihoraire)
    36)  attend match ≥ 95 s
    37)  avance 40 cm (relatif)
    38)  FINISHED
   --------------------------------------------------------------- */


/* ---------- PINOUT -------------------------------------------- */
#define encRA 2
#define encLA 3
#define encRB 4
#define encLB 5
#define DIR_R 8
#define PWM_R 9
#define DIR_L 7
#define PWM_L 10
const uint8_t START_PIN = 11;
const int TRIG_PIN = 31, ECHO_PIN = 29, SEUIL_CM = 20;
const int TRIG_PIN_REAR = 53, ECHO_PIN_REAR = 51;

/* ---------- I²C ---------------------------------------------- */
#include <Wire.h>
#include "i2c_handler.h"
#include <Servo.h>

/* ---------- Encodeurs / PID ----------------------------------- */
volatile long countR = 0, countL = 0;
long prevCountR = 0, prevCountL = 0;

static bool started = false;
static unsigned long matchStart = 0UL;

const float P_R = 1.0f, I_R = 0.30f, D_R = 0.07f;
const float P_L = 1.0f, I_L = 0.40f, D_L = 0.07f;
float PErrR = 0, PErrL = 0;
long IErrR = 0, IErrL = 0;

int cmdVD = 0, cmdVG = 0, rampD = 0, rampG = 0;
float pwmD = 0, pwmG = 0;

/* Pinces */
Servo pinceGauche;  // pin 46
Servo pinceDroite;  // pin 44

const uint8_t PIN_PINCE_G = 46;
const uint8_t PIN_PINCE_D = 44;

/* angles mécaniques — à ajuster */
const int angleFermeG = 50;   // pince gauche fermée
const int angleFermeD = 170;  // pince droite fermée
const int angleOuvreG = 170;  // pince gauche ouverte
const int angleOuvreD = 60;   // pince droite ouverte
void open_grap();
void close_grap();

/* ---------- Odométrie ---------------------------------------- */
const float coeff = 100.0f / 50.0f;  // ticks/50 ms → ticks/100 ms
const int acc = 20;                  // rampe
#define TICK_CM_R 37.0f
#define TICK_CM_L 37.0f
#define TICKS90_R (1025.0f / 2.0f)
#define TICKS90_L (1025.0f / 2.0f)

float X = 0.0f, Y = 0.0f, Z = 0.0f;
unsigned long previousMillis = 0;

/* ---------- États haut niveau -------------------------------- */
enum Phase {
  IDLE,
  BACKWARD_12,
  PAUSE_1,
  OPEN_RACK,
  BANNER_DOWN,
  BANNER_UP,
  CLOSE_RACK,
  FORWARD_45_Y,
  PAUSE_2,
  ROTATE_90_CW1,
  PAUSE_3,
  FORWARD_10,
  PAUSE_4,
  ROTATE_90_CCW,
  PAUSE_5,
  OPEN_GRAP,
  PAUSE_6,
  FORWARD_20A,
  PAUSE_7,
  SET_Z_245,
  PAUSE_7B,  // pause 4 s
  BACKWARD_20,
  PAUSE_8,
  ROTATE_90_CW2,
  PAUSE_9,
  FORWARD_20B,
  PAUSE_10,
  ROTATE_90_CW3,
  PAUSE_11,
  FORWARD_38,
  PAUSE_12,
  SET_Z_22,
  PAUSE_13,
  BACKWARD_30,
  PAUSE_14,
  CLOSE_GRAP,
  PAUSE_15,
  ROTATE_190_CW,
  PAUSE_16,
  FORWARD_80,
  PAUSE_17,
  ROTATE_10_CCW,
  PAUSE_MATCH,
  FORWARD_40_END,
  OBSTACLE_PAUSE,
  FINISHED
};
Phase phase = IDLE, savedPhase = IDLE;

/* ---------- Constantes de trajet ----------------------------- */
float Yt = 0.0f, Zstart = 0.0f;
unsigned long pauseStart = 0;
const int CRUISE_TICKS = 15;
const int ROT_TICKS = 15;
const float POS_THRESH = 0.5f;
const float ANG_THRESH_RAD = 0.05f;
const unsigned long PAUSE_OBSTACLE_MS = 2000UL;

/* ---------- Distance relative -------------------------------- */
float segTarget = 0.0f;  // cm
float segDone = 0.0f;    // cm

/* ---------- Override (obstacle) ------------------------------ */
volatile bool overrideRequested = false;
unsigned long overridePauseMs = 0;

/* ---------- Buffer série ------------------------------------- */
char streamChar[32];
uint8_t idx = 0;
int incomingByte = 0;

/* ---------- Prototypes --------------------------------------- */
int readUltrasonic();
int readUltrasonicRear();
void sendPWM(int l, int r);
void decryptIncom();
void updateHighLevel();
void setCommande(int l, int r, unsigned long ms);

/* ---------- Helper nom d’état -------------------------------- */
const char* phaseName(Phase p) {
  switch (p) {
    case IDLE: return "IDLE";
    case BACKWARD_12: return "BACKWARD_12";
    case PAUSE_1: return "PAUSE_1";
    case OPEN_RACK: return "OPEN_RACK";
    case BANNER_DOWN: return "BANNER_DOWN";
    case BANNER_UP: return "BANNER_UP";
    case CLOSE_RACK: return "CLOSE_RACK";
    case FORWARD_45_Y: return "FORWARD_45_Y";
    case PAUSE_2: return "PAUSE_2";
    case ROTATE_90_CW1: return "ROTATE_90_CW1";
    case PAUSE_3: return "PAUSE_3";
    case FORWARD_10: return "FORWARD_10";
    case PAUSE_4: return "PAUSE_4";
    case ROTATE_90_CCW: return "ROTATE_90_CCW";
    case PAUSE_5: return "PAUSE_5";
    case OPEN_GRAP: return "OPEN_GRAP";
    case PAUSE_6: return "PAUSE_6";
    case FORWARD_20A: return "FORWARD_20A";
    case PAUSE_7: return "PAUSE_7";
    case SET_Z_245: return "SET_Z_245";
    case PAUSE_7B: return "PAUSE_7B";
    case BACKWARD_20: return "BACKWARD_20";
    case PAUSE_8: return "PAUSE_8";
    case ROTATE_90_CW2: return "ROTATE_90_CW2";
    case PAUSE_9: return "PAUSE_9";
    case FORWARD_20B: return "FORWARD_20B";
    case PAUSE_10: return "PAUSE_10";
    case ROTATE_90_CW3: return "ROTATE_90_CW3";
    case PAUSE_11: return "PAUSE_11";
    case FORWARD_38: return "FORWARD_38";
    case PAUSE_12: return "PAUSE_12";
    case SET_Z_22: return "SET_Z_22";
    case PAUSE_13: return "PAUSE_13";
    case BACKWARD_30: return "BACKWARD_30";
    case PAUSE_14: return "PAUSE_14";
    case CLOSE_GRAP: return "CLOSE_GRAP";
    case PAUSE_15: return "PAUSE_15";
    case ROTATE_190_CW: return "ROTATE_190_CW";
    case PAUSE_16: return "PAUSE_16";
    case FORWARD_80: return "FORWARD_80";
    case PAUSE_17: return "PAUSE_17";
    case ROTATE_10_CCW: return "ROTATE_10_CCW";
    case PAUSE_MATCH: return "PAUSE_MATCH";
    case FORWARD_40_END: return "FORWARD_40_END";
    case OBSTACLE_PAUSE: return "OBSTACLE_PAUSE";
    case FINISHED: return "FINISHED";
    default: return "?";
  }
}


/* ---------------- SETUP -------------------------------------- */
void setup() {
  Serial.begin(115200);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(encRA, INPUT_PULLUP);
  pinMode(encRB, INPUT_PULLUP);
  pinMode(encLA, INPUT_PULLUP);
  pinMode(encLB, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLA), interruptL, CHANGE);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_REAR, OUTPUT);
  pinMode(ECHO_PIN_REAR, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  i2c_handler_init(0x07);
  sendI2CCommand("banner_up");
  /* pinces ---------------------------------------------------- */
  pinceGauche.attach(PIN_PINCE_G);
  pinceDroite.attach(PIN_PINCE_D);
  close_grap();  // état sûr au démarrage

  previousMillis = millis();
  Serial.println("Robot prêt !");
}

/* ---------------- LOOP --------------------------------------- */
void loop() {
  /* RX série -------------------------------------------------- */
  while (Serial.available()) {
    incomingByte = Serial.read();
    streamChar[idx++] = (char)incomingByte;
    if (incomingByte == '\n' || idx >= 31) {
      streamChar[idx] = '\0';
      decryptIncom();
      idx = 0;
      incomingByte = 0;
    }
  }

  /* Démarrage ------------------------------------------------- */
  if (!started && digitalRead(START_PIN) == LOW) {
    started = true;
    matchStart = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    Yt = Y - 12.0f;
    phase = BACKWARD_12;
    cmdVG = cmdVD = 0;
  }

  /* Boucle 40 ms --------------------------------------------- */
  unsigned long now = millis();
  if (now - previousMillis < 40) return;
  previousMillis = now;

  /* odométrie ------------------------------------------------- */
  float dL = (countL - prevCountL) * coeff;
  float dR = (countR - prevCountR) * coeff;
  prevCountL = countL;
  prevCountR = countR;

  float dD = 0.5f * (dR / TICK_CM_R + dL / TICK_CM_L);  // cm
  float rR = dR / TICKS90_R * (PI / 2.0f);
  float rL = dL / TICKS90_L * (PI / 2.0f);
  float dZ = 0.5f * (rR - rL);  // rad
  float midZ = Z + 0.5f * dZ;
  Y += dD * cos(midZ);
  X += dD * sin(midZ);
  Z += dZ;
  if (Z > PI) Z -= 2 * PI;
  if (Z < -PI) Z += 2 * PI;

  /* distance relative ---------------------------------------- */
  if (phase == FORWARD_10 || phase == FORWARD_20A || phase == FORWARD_20B || phase == FORWARD_38 || phase == FORWARD_80 || phase == FORWARD_40_END || phase == BACKWARD_20 || phase == BACKWARD_30)
    segDone += fabs(dD);


  /* rampe ----------------------------------------------------- */
  rampG = (rampG < cmdVG) ? min(rampG + acc, cmdVG) : (rampG > cmdVG) ? max(rampG - acc, cmdVG)
                                                                      : rampG;
  rampD = (rampD < cmdVD) ? min(rampD + acc, cmdVD) : (rampD > cmdVD) ? max(rampD - acc, cmdVD)
                                                                      : rampD;

  /* PID droite ------------------------------------------------ */
  float errR = rampD - dR, derivR = errR - PErrR;
  long IErrR_tmp = IErrR + errR;
  float outR = P_R * errR + I_R * IErrR_tmp + D_R * derivR;
  if (abs(outR) < 255) IErrR = IErrR_tmp;
  pwmD = constrain(outR, -255, 255);
  PErrR = errR;

  /* PID gauche ------------------------------------------------ */
  float errL = rampG - dL, derivL = errL - PErrL;
  long IErrL_tmp = IErrL + errL;
  float outL = P_L * errL + I_L * IErrL_tmp + D_L * derivL;
  if (abs(outL) < 255) IErrL = IErrL_tmp;
  pwmG = constrain(outL, -255, 255);
  PErrL = errL;

  /* anti-windup ---------------------------------------------- */
  if (!cmdVD) { IErrR = PErrR = 0; }
  if (!cmdVG) { IErrL = PErrL = 0; }

  /* Lecture ultrasons ---------------------------------------- */
  int frontDist = readUltrasonic();
  int rearDist = readUltrasonicRear();

  bool movingForward =
    (phase == FORWARD_45_Y || phase == FORWARD_10 || phase == FORWARD_20A || phase == FORWARD_20B || phase == FORWARD_38 || phase == FORWARD_80 || phase == FORWARD_40_END);

  bool movingBackwardRear =
    (cmdVG < 0 && cmdVD < 0 && (phase == BACKWARD_20 || phase == BACKWARD_30));

  /* Gestion d’obstacle --------------------------------------- */
  if (phase != OBSTACLE_PAUSE) {
    if ((movingForward && frontDist < SEUIL_CM) || (movingBackwardRear && rearDist < SEUIL_CM)) {
      Serial.println(">> Obstacle pause");
      pauseStart = now;
      savedPhase = phase;
      phase = OBSTACLE_PAUSE;

      sendPWM(0, 0);
      rampG = rampD = 0;
      IErrL = IErrR = 0;
      PErrL = PErrR = 0;
      cmdVG = cmdVD = 0;
    } else {
      sendPWM((int)pwmG, (int)pwmD);
      updateHighLevel();
    }
  } else {
    sendPWM(0, 0);
    cmdVG = cmdVD = 0;
    if (now - pauseStart >= (overrideRequested ? overridePauseMs : PAUSE_OBSTACLE_MS)) {
      overrideRequested = false;
      phase = savedPhase;
    }
  }

  /* debug ----------------------------------------------------- */
  Serial.print("dD=");
  Serial.print(dD, 2);
  Serial.print(" X=");
  Serial.print(X, 1);
  Serial.print(" Y=");
  Serial.print(Y, 1);
  Serial.print(" Z=");
  Serial.print(Z * 180.0f / PI, 1);
  Serial.print("° phase=");
  Serial.println(phaseName(phase));
}

/* ---------- FSM haut niveau ---------------------------------- */
void updateHighLevel() {
  switch (phase) {

      /* 1 – recule 12 cm ------------------------------------------------ */
    case BACKWARD_12:
      {
        float err = Yt - Y;
        if (fabs(err) > POS_THRESH) {
          int s = (err > 0 ? CRUISE_TICKS : -CRUISE_TICKS);
          cmdVG = cmdVD = s;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_1;
        }
        break;
      }

      /* 2 – open_rack → banner_down → banner_up → close_rack ----------- */
    case PAUSE_1:
      if (millis() - pauseStart >= 2000UL) {
        sendI2CCommand("open_rack");
        phase = OPEN_RACK;
        pauseStart = millis();
      }
      break;
    case OPEN_RACK:
      if (millis() - pauseStart >= 2000UL) {
        sendI2CCommand("banner_down");
        phase = BANNER_DOWN;
        pauseStart = millis();
      }
      break;
    case BANNER_DOWN:
      if (millis() - pauseStart >= 2000UL) {
        sendI2CCommand("banner_up");
        phase = BANNER_UP;
        pauseStart = millis();
      }
      break;
    case BANNER_UP:
      if (millis() - pauseStart >= 2000UL) {
        sendI2CCommand("close_rack");
        phase = CLOSE_RACK;
        pauseStart = millis();
      }
      break;
    case CLOSE_RACK:
      if (millis() - pauseStart >= 2000UL) {
        Yt = Y + 45.0f;  // avance absolue de 45 cm
        phase = FORWARD_45_Y;
      }
      break;

      /* 3 – avance 45 cm (axe Y) -------------------------------------- */
    case FORWARD_45_Y:
      {
        float err = Yt - Y;
        if (fabs(err) > POS_THRESH) {
          int s = (err > 0 ? CRUISE_TICKS : -CRUISE_TICKS);
          cmdVG = cmdVD = s;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_2;
        }
        break;
      }

      /* 5 – rotation −90° CW ------------------------------------------ */
    case PAUSE_2:
      if (millis() - pauseStart >= 2000UL) {
        Zstart = Z;
        phase = ROTATE_90_CW1;
      }
      break;
    case ROTATE_90_CW1:
      {
        float Zc = Zstart - PI / 2.0f;
        if (Zc < -PI) Zc += 2 * PI;
        float err = Zc - Z;
        if (err > PI) err -= 2 * PI;
        if (err < -PI) err += 2 * PI;
        if (fabs(err) > ANG_THRESH_RAD) {
          int m = (err > 0 ? ROT_TICKS : -ROT_TICKS);
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_3;
        }
        break;
      }

      /* 7 – avance 10 cm relatif -------------------------------------- */
    case PAUSE_3:
      if (millis() - pauseStart >= 2000UL) {
        segTarget = 10.0f;
        segDone = 0.0f;
        phase = FORWARD_10;
      }
      break;
    case FORWARD_10:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_4;
      }
      break;

      /* 9 – rotation +90° CCW ---------------------------------------- */
    case PAUSE_4:
      if (millis() - pauseStart >= 2000UL) {
        Zstart = Z;
        phase = ROTATE_90_CCW;
      }
      break;
    case ROTATE_90_CCW:
      {
        float Zc = Zstart + PI / 2.0f;
        if (Zc > PI) Zc -= 2 * PI;
        float err = Zc - Z;
        if (err > PI) err -= 2 * PI;
        if (err < -PI) err += 2 * PI;
        if (fabs(err) > ANG_THRESH_RAD) {
          int m = (err > 0 ? ROT_TICKS : -ROT_TICKS);
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_5;
        }
        break;
      }

      /* 11 – open_grap I²C ------------------------------------------- */
    case PAUSE_5:
      if (millis() - pauseStart >= 2000UL) {
        open_grap();
        phase = OPEN_GRAP;
        pauseStart = millis();
      }
      break;
    case OPEN_GRAP:
      if (millis() - pauseStart >= 2000UL) {
        pauseStart = millis();
        phase = PAUSE_6;
      }
      break;

      /* 13 – avance 20 cm -------------------------------------------- */
    case PAUSE_6:
      if (millis() - pauseStart >= 2000UL) {
        segTarget = 20.0f;
        segDone = 0.0f;
        phase = FORWARD_20A;
      }
      break;
    case FORWARD_20A:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_7;
      }
      break;

      /* 15 – setZ_cm(24.5) + pause 4 s ------------------------------- */
    case PAUSE_7:
      if (millis() - pauseStart >= 2000UL) {
        sendI2CCommand("setZ_cm(24.5)");
        phase = SET_Z_245;
        pauseStart = millis();
      }
      break;
    case SET_Z_245:
      if (millis() - pauseStart >= 4000UL) {
        pauseStart = millis();
        phase = PAUSE_7B;
      }
      break;

      /* 16 – recule 20 cm -------------------------------------------- */
    case PAUSE_7B:
      /* direct : pas de délai supplémentaire */
      segTarget = 20.0f;
      segDone = 0.0f;
      phase = BACKWARD_20;
      break;
    case BACKWARD_20:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = -CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_8;
      }
      break;

      /* 18 – rotation −90° CW ---------------------------------------- */
    case PAUSE_8:
      if (millis() - pauseStart >= 2000UL) {
        Zstart = Z;
        phase = ROTATE_90_CW2;
      }
      break;
    case ROTATE_90_CW2:
      {
        float Zc = Zstart - PI / 2.0f;
        if (Zc < -PI) Zc += 2 * PI;
        float err = Zc - Z;
        if (err > PI) err -= 2 * PI;
        if (err < -PI) err += 2 * PI;
        if (fabs(err) > ANG_THRESH_RAD) {
          int m = (err > 0 ? ROT_TICKS : -ROT_TICKS);
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_9;
        }
        break;
      }

      /* 20 – avance 20 cm -------------------------------------------- */
    case PAUSE_9:
      if (millis() - pauseStart >= 2000UL) {
        segTarget = 20.0f;
        segDone = 0.0f;
        phase = FORWARD_20B;
      }
      break;
    case FORWARD_20B:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_10;
      }
      break;

      /* 22 – rotation −90° CW ---------------------------------------- */
    case PAUSE_10:
      if (millis() - pauseStart >= 2000UL) {
        Zstart = Z;
        phase = ROTATE_90_CW3;
      }
      break;
    case ROTATE_90_CW3:
      {
        float Zc = Zstart - PI / 2.0f;
        if (Zc < -PI) Zc += 2 * PI;
        float err = Zc - Z;
        if (err > PI) err -= 2 * PI;
        if (err < -PI) err += 2 * PI;
        if (fabs(err) > ANG_THRESH_RAD) {
          int m = (err > 0 ? ROT_TICKS : -ROT_TICKS);
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_11;
        }
        break;
      }

      /* 24 – avance 38 cm -------------------------------------------- */
    case PAUSE_11:
      if (millis() - pauseStart >= 2000UL) {
        segTarget = 38.0f;
        segDone = 0.0f;
        phase = FORWARD_38;
      }
      break;
    case FORWARD_38:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_12;
      }
      break;

      /* 26 – setZ_cm(22.0) ------------------------------------------- */
    case PAUSE_12:
      if (millis() - pauseStart >= 2000UL) {
        sendI2CCommand("setZ_cm(22.0)");
        phase = SET_Z_22;
        pauseStart = millis();
      }
      break;
    case SET_Z_22:
      if (millis() - pauseStart >= 2000UL) {
        pauseStart = millis();
        phase = PAUSE_13;
      }
      break;

      /* 28 – recule 30 cm -------------------------------------------- */
    case PAUSE_13:
      segTarget = 30.0f;
      segDone = 0.0f;
      phase = BACKWARD_30;
      break;
    case BACKWARD_30:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = -CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_14;
      }
      break;

      /* 30 – close_grap --------------------------------------------- */
    case PAUSE_14:
      if (millis() - pauseStart >= 2000UL) {
        close_grap();
        phase = CLOSE_GRAP;
        pauseStart = millis();
      }
      break;
    case CLOSE_GRAP:
      if (millis() - pauseStart >= 2000UL) {
        Zstart = Z;
        phase = ROTATE_190_CW;
      }
      break;

      /* 31 – rotation −190° CW -------------------------------------- */
    case ROTATE_190_CW:
      {
        float Zc = Zstart - (190.0f * PI / 180.0f);
        if (Zc < -PI) Zc += 2 * PI;
        float err = Zc - Z;
        if (err > PI) err -= 2 * PI;
        if (err < -PI) err += 2 * PI;
        if (fabs(err) > ANG_THRESH_RAD) {
          int m = (err > 0 ? ROT_TICKS : -ROT_TICKS);
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_16;
        }
        break;
      }

      /* 33 – avance 80 cm ------------------------------------------- */
    case PAUSE_16:
      if (millis() - pauseStart >= 2000UL) {
        segTarget = 80.0f;
        segDone = 0.0f;
        phase = FORWARD_80;
      }
      break;
    case FORWARD_80:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_17;
      }
      break;

      /* 35 – rotation +10° CCW -------------------------------------- */
    case PAUSE_17:
      if (millis() - pauseStart >= 2000UL) {
        Zstart = Z;
        phase = ROTATE_10_CCW;
      }
      break;
    case ROTATE_10_CCW:
      {
        float Zc = Zstart + (10.0f * PI / 180.0f);
        if (Zc > PI) Zc -= 2 * PI;
        float err = Zc - Z;
        if (err > PI) err -= 2 * PI;
        if (err < -PI) err += 2 * PI;
        if (fabs(err) > ANG_THRESH_RAD) {
          int m = (err > 0 ? ROT_TICKS : -ROT_TICKS);
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          phase = PAUSE_MATCH;
        }
        break;
      }

      /* 36 – attendre ≥95 s ----------------------------------------- */
    case PAUSE_MATCH:
      if (millis() - matchStart >= 95000UL) {
        segTarget = 40.0f;
        segDone = 0.0f;
        phase = FORWARD_40_END;
      }
      break;

      /* 37 – avance finale 40 cm ------------------------------------ */
    case FORWARD_40_END:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        phase = FINISHED;
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println(">>> Séquence terminée !");
      }
      break;

    default: break;
  }
}

/* ---------- Ultrason ------------------------------------------ */
int readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH);
  return dur * 0.034f / 2.0f;
}
int readUltrasonicRear() {
  digitalWrite(TRIG_PIN_REAR, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_REAR, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_REAR, LOW);
  long dur = pulseIn(ECHO_PIN_REAR, HIGH);
  return dur * 0.034f / 2.0f;
}

/* ---------- PWM moteurs --------------------------------------- */
void sendPWM(int l, int r) {
  if (!l) analogWrite(PWM_L, 0);
  else {
    digitalWrite(DIR_L, l > 0 ? LOW : HIGH);
    analogWrite(PWM_L, min(abs(l), 255));
  }
  if (!r) analogWrite(PWM_R, 0);
  else {
    digitalWrite(DIR_R, r > 0 ? HIGH : LOW);
    analogWrite(PWM_R, min(abs(r), 255));
  }
}

/* ---------- Override ------------------------------------------ */
void setCommande(int l, int r, unsigned long ms) {
  overrideRequested = true;
  overridePauseMs = ms;
  cmdVG = l;
  cmdVD = r;
}

/* ---------- RX série (vitesse) -------------------------------- */
void decryptIncom() {
  if (streamChar[0] == 'P' || streamChar[0] == 'p') {
    idx = 0;
    return;
  }
  idx = 0;
  cmdVG = cmdVD = 0;
  bool neg = false;
  if (streamChar[idx] == '-') {
    neg = true;
    idx++;
  }
  while (isdigit(streamChar[idx])) cmdVG = cmdVG * 10 + (streamChar[idx++] - '0');
  if (neg) cmdVG = -cmdVG;
  idx++;
  neg = false;
  if (streamChar[idx] == '-') {
    neg = true;
    idx++;
  }
  while (isdigit(streamChar[idx])) cmdVD = cmdVD * 10 + (streamChar[idx++] - '0');
  if (neg) cmdVD = -cmdVD;
}

/* ---------- ISRs ---------------------------------------------- */
void interruptR() {
  digitalRead(encRA) == digitalRead(encRB) ? countR++ : countR--;
}
void interruptL() {
  digitalRead(encLA) == digitalRead(encLB) ? countL-- : countL++;
}

void open_grap() {
  pinceGauche.write(angleOuvreG);
  pinceDroite.write(angleOuvreD);
}
void close_grap() {
  pinceGauche.write(angleFermeG);
  pinceDroite.write(angleFermeD);
}
