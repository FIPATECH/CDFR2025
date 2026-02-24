/* ---------------------------------------------------------------
   Robot différentiel – PID + Odométrie
   Séquence finale :
     1)  recule 12 cm
     2)  open_rack → banner_down → banner_up → close_rack (3 s chacun)
     3)  avance 50 cm (axe Y)
     4)  pause 3 s
     5)  rotation −90 ° (horaire)
     6)  pause 3 s
     7)  avance 50 cm  (distance relative)
     8)  pause 3 s
     9)  rotation −90 ° (horaire)
    10)  pause 3 s
    11)  avance 38 cm  (distance relative)
    12)  pause 2 s
    13)  recule 20 cm
    14)  pause 2 s
    15)  rotation −190 ° (horaire)
    16)  pause 2 s
    17)  avance 45 cm
    18)  pause 2 s
    19)  rotation +20 ° (antihoraire)
    20)  attend le temps-match ≥ 95 s
    21)  avance 50 cm
    22)  FINISHED
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
const int TRIG_PIN = 31, ECHO_PIN = 29, SEUIL_CM = 20;  // avant
const int TRIG_PIN_REAR = 53, ECHO_PIN_REAR = 51;       // arrière

const unsigned long MATCH_DURATION_MS = 100000UL;  // 100 s

/* ---------- I²C ---------------------------------------------- */
#include <Wire.h>
#include "i2c_handler.h"

/* ---------- Encodeurs / PID ---------------------------------- */
volatile long countR = 0, countL = 0;
long prevCountR = 0, prevCountL = 0;

static bool started = false;
static unsigned long matchStart = 0UL;
static bool matchAlreadyRun = false;

const float P_R = 1.0f, I_R = 0.30f, D_R = 0.07f;
const float P_L = 1.0f, I_L = 0.40f, D_L = 0.07f;
float PErrR = 0, PErrL = 0;
long IErrR = 0, IErrL = 0;

int cmdVD = 0, cmdVG = 0, rampD = 0, rampG = 0;
float pwmD = 0, pwmG = 0;

/* ---------- Odométrie ---------------------------------------- */
const unsigned long PID_INTERVAL_MS = 40;
const float coeff = 100.0f / (float)PID_INTERVAL_MS;  // ticks/40 ms → ticks/100 ms
const int ACC_RAMP = 20;                              // rampe d’accélération (ticks/40 ms)
#define TICK_CM_R 19.0f
#define TICK_CM_L 19.0f
#define TICKS90_R 275.0f
#define TICKS90_L 275.0f

float X = 0.0f, Y = 0.0f, Z = 0.0f;
unsigned long previousMillis = 0;

/* ---------- Suivi de cap & rotation proportionnelle ----------- */
// --- Suivi de cap (corrige les segments rectilignes) ---
const float K_CAP = 50.0f;                    // gain correcteur cap
const float HEADING_CORRECTION_AUTH = 0.75f;  // autorité max (fraction CRUISE_TICKS)
const float HEADING_DEADBAND_RAD = 0.0087f;   // zone morte (~0,5°)
float headingTarget = 0.0f;

// --- Rotation proportionnelle ---
const float K_ROT_P = 60.0f;        // gain P rotation → vitesse ticks
const int MIN_ROT_SPEED_TICKS = 4;  // vitesse mini pour vaincre friction
const uint8_t STABLE_STEPS = 6;     // cycles PID stables avant fin rotation

/* ---------- États haut niveau -------------------------------- */
enum Phase {
  IDLE,
  BACKWARD_12,
  PAUSE_1,
  OPEN_RACK,
  BANNER_DOWN,
  BANNER_UP,
  CLOSE_RACK,
  FORWARD_50_Y,
  PAUSE_6,
  ROTATE_90_CW1,
  PAUSE_7,
  FORWARD_50,
  PAUSE_8,
  ROTATE_90_CW2,
  PAUSE_9,
  FORWARD_40,
  PAUSE_10,
  BACKWARD_20,
  PAUSE_11,
  ROTATE_200_CW,
  PAUSE_12,
  FORWARD_45,
  PAUSE_13,
  ROTATE_20_CCW,  // Probablement confondu avec ROTATE_94_CCW
  PAUSE_MATCH,
  FORWARD_60_END,
  OBSTACLE_PAUSE,
  FINISHED,
  ROTATE_94_CCW,    // Ajout
  PAUSE_14,         // Ajout
  FORWARD_65,       // Ajout
  PAUSE_15,         // Ajout
  ROTATE_90_CCWW,   // Ajout
  PAUSE_16,         // Ajout
  FORWARD_100,      // Ajout
  PAUSE_17,         // Ajouté : pause 2s avant BACKWARD_20_BIS
  BACKWARD_20_BIS,  // Ajouté : recule 20 cm (étape 13)
  PAUSE_18,         // Ajouté : pause 2s après BACKWARD_20_BIS
  ROTATE_220_CW,    // Ajouté : rotation -190° (étape 15)
  PAUSE_19,         // Ajouté : pause 2s après ROTATE_220_CW
  FORWARD_120,      // Ajouté : avance 120 cm (étape 17)
  PAUSE_20,
  ROTATE_20_BEFORE_END
};

Phase phase = IDLE, savedPhase = IDLE;
static Phase prevPhase = IDLE;

/* ---------- Constantes de trajet ----------------------------- */
float Yt = 0.0f,
      Zstart = 0.0f;
unsigned long pauseStart = 0;
const int CRUISE_TICKS = 30;
const int ROT_TICKS = 25;
const float POS_THRESH = 0.5f;
const float ANG_THRESH_RAD = 0.05f;
const unsigned long PAUSE_OBSTACLE_MS = 1000UL;

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
inline float normRad(float a);

/* ---------- Helper nom d’état -------------------------------- */
const char* phaseName(Phase p) {
  switch (p) {
    case IDLE:
      return "IDLE";
    case BACKWARD_12:
      return "BACKWARD_12";
    case PAUSE_1:
      return "PAUSE_1";
    case OPEN_RACK:
      return "OPEN_RACK";
    case BANNER_DOWN:
      return "BANNER_DOWN";
    case BANNER_UP:
      return "BANNER_UP";
    case CLOSE_RACK:
      return "CLOSE_RACK";
    case FORWARD_50_Y:
      return "FORWARD_50_Y";
    case PAUSE_6:
      return "PAUSE_6";
    case ROTATE_90_CW1:
      return "ROTATE_90_CW1";
    case PAUSE_7:
      return "PAUSE_7";
    case FORWARD_50:
      return "FORWARD_50";
    case PAUSE_8:
      return "PAUSE_8";
    case ROTATE_90_CW2:
      return "ROTATE_90_CW2";
    case PAUSE_9:
      return "PAUSE_9";
    case FORWARD_40:
      return "FORWARD_40";
    case PAUSE_10:
      return "PAUSE_10";
    case BACKWARD_20:
      return "BACKWARD_20";
    case PAUSE_11:
      return "PAUSE_11";
    case ROTATE_200_CW:
      return "ROTATE_200_CW";
    case PAUSE_12:
      return "PAUSE_12";
    case FORWARD_45:
      return "FORWARD_45";
    case PAUSE_13:
      return "PAUSE_13";
    case ROTATE_20_CCW:
      return "ROTATE_20_CCW";
    case PAUSE_MATCH:
      return "PAUSE_MATCH";
    case FORWARD_60_END:
      return "FORWARD_60_END";
    case OBSTACLE_PAUSE:
      return "OBSTACLE_PAUSE";
    case FINISHED:
      return "FINISHED";
    case ROTATE_94_CCW: return "ROTATE_94_CCW";
    case PAUSE_14: return "PAUSE_14";
    case FORWARD_65: return "FORWARD_65";
    case PAUSE_15: return "PAUSE_15";
    case ROTATE_90_CCWW: return "ROTATE_90_CCWW";
    case ROTATE_20_BEFORE_END: return "ROTATE_20_BEFORE_END";
    case PAUSE_16: return "PAUSE_16";
    case PAUSE_20: return "PAUSE_20";
    case FORWARD_100: return "FORWARD_100";

    default:
      return "?";
  }
}

/* ---------- Normalisation angle ------------------------------ */
inline float normRad(float a) {
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
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

  // 2) Lecture du temps courant une seule fois
  unsigned long now = millis();

  // 3) Arrêt forcé à 100 s de match (quel que soit l’état)
  if (started && !matchAlreadyRun && now - matchStart >= MATCH_DURATION_MS) {
    phase = FINISHED;
    matchAlreadyRun = true;
    // Remise à zéro de toutes les commandes/rampe/PID
    cmdVG = cmdVD = 0;
    rampG = rampD = 0;
    IErrL = IErrR = 0;
    PErrL = PErrR = 0;
    sendPWM(0, 0);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(">>> Fin forcée du match à 100 s !");
    return;
  }


  // 4) Détection du bouton de démarrage (une seule fois)
  if (!started && !matchAlreadyRun && digitalRead(START_PIN) == LOW) {
    started = true;
    matchStart = now;  // utilise la même variable now
    digitalWrite(LED_BUILTIN, HIGH);
    Yt = Y - 12.0f;
    phase = BACKWARD_12;
    cmdVG = cmdVD = 0;
  }

  /* Boucle PID 40 ms ----------------------------------------- */
  if (now - previousMillis < PID_INTERVAL_MS)
    return;
  previousMillis = now;

  /* odométrie ------------------------------------------------- */
  float dL_ticks = countL - prevCountL;
  float dR_ticks = countR - prevCountR;
  prevCountL = countL;
  prevCountR = countR;

  float dL = dL_ticks * coeff;
  float dR = dR_ticks * coeff;

  float dD = 0.5f * (dR_ticks / TICK_CM_R + dL_ticks / TICK_CM_L);  // cm
  float rR = dR_ticks / TICKS90_R * (PI / 2.0f);
  float rL = dL_ticks / TICKS90_L * (PI / 2.0f);
  float dZ = 0.5f * (rR - rL);
  float midZ = Z + 0.5f * dZ;
  Y += dD * cos(midZ);
  X += dD * sin(midZ);
  Z = normRad(Z + dZ);

  /* distance relative ---------------------------------------- */
  /* distance relative */
  if (phase == FORWARD_50
      || phase == FORWARD_40
      || phase == BACKWARD_20
      || phase == BACKWARD_20_BIS  // ← nouvel état
      || phase == FORWARD_45
      || phase == FORWARD_60_END
      || phase == FORWARD_65
      || phase == FORWARD_100
      || phase == FORWARD_120)  // ← nouvel état
  {
    segDone += fabs(dD);
  }

  /* rampe ----------------------------------------------------- */
  rampG += (rampG < cmdVG) ? min(ACC_RAMP, cmdVG - rampG) : (rampG > cmdVG) ? -min(ACC_RAMP, rampG - cmdVG)
                                                                            : 0;
  rampD += (rampD < cmdVD) ? min(ACC_RAMP, cmdVD - rampD) : (rampD > cmdVD) ? -min(ACC_RAMP, rampD - cmdVD)
                                                                            : 0;

  /* PID droite ------------------------------------------------ */
  float errR = rampD - dR, derivR = errR - PErrR;
  long IErrR_tmp = IErrR + errR;
  float outR = P_R * errR + I_R * IErrR_tmp + D_R * derivR;
  if (abs(outR) < 255)
    IErrR = IErrR_tmp;
  pwmD = constrain(outR, -255, 255);
  PErrR = errR;

  /* PID gauche ------------------------------------------------ */
  float errL = rampG - dL, derivL = errL - PErrL;
  long IErrL_tmp = IErrL + errL;
  float outL = P_L * errL + I_L * IErrL_tmp + D_L * derivL;
  if (abs(outL) < 255)
    IErrL = IErrL_tmp;
  pwmG = constrain(outL, -255, 255);
  PErrL = errL;

  /* anti-windup ---------------------------------------------- */
  if (!cmdVD && !rampD) {
    IErrR = PErrR = 0;
  }
  if (!cmdVG && !rampG) {
    IErrL = PErrL = 0;
  }

  /* Lecture ultrasons ---------------------------------------- */
  int frontDist = readUltrasonic();
  int rearDist = readUltrasonicRear();
  bool movingForward = (phase == FORWARD_50_Y || phase == FORWARD_50 || phase == FORWARD_40 || phase == FORWARD_45 || phase == FORWARD_60_END);
  bool movingBackwardRear = (cmdVG < 0 && cmdVD < 0 && phase != BACKWARD_12 && phase != BACKWARD_20);

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
  /* Détection de changement d’état pour initialiser le cap cible */
  if (phase != prevPhase) {
    if (phase == FORWARD_50_Y || phase == FORWARD_50 || phase == FORWARD_40 || phase == FORWARD_45 || phase == FORWARD_60_END) {
      headingTarget = Z;  // mémorise la direction actuelle
    }
    prevPhase = phase;
  }

  switch (phase) {
    /* ---------- bloc initial déjà existant --------- */
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

    /* séquence I²C 4× */
    case PAUSE_1:
      if (millis() - pauseStart >= 1000UL) {
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
      if (millis() - pauseStart >= 1500UL) {
        sendI2CCommand("banner_up");
        phase = BANNER_UP;
        pauseStart = millis();
      }
      break;
    case BANNER_UP:
      if (millis() - pauseStart >= 1000UL) {
        sendI2CCommand("close_rack");
        phase = CLOSE_RACK;
        pauseStart = millis();
      }
      break;
    case CLOSE_RACK:
      if (millis() - pauseStart >= 1000UL) {
        Yt = Y + 48.0f;
        phase = FORWARD_50_Y;
      }
      break;

      /* avance 50 cm (axe Y) */
    case FORWARD_50_Y:
      {
        float err = Yt - Y;
        if (fabs(err) > POS_THRESH) {
          int spd = (err > 0 ? CRUISE_TICKS : -CRUISE_TICKS);
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH), (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_6;
        }
        break;
      }

    /* --- rotation -90° (CW) améliorée ------------ */
    case PAUSE_6:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_90_CW1;
      }
      break;
    case ROTATE_90_CW1:
      {
        static uint8_t okCnt1 = 0;
        float Zc = normRad(Zstart + PI / 2.0f);
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt1 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt1 >= STABLE_STEPS) {
            okCnt1 = 0;
            pauseStart = millis();
            phase = PAUSE_7;
          }
        }
        break;
      }

    /* --- Avance 50 cm relatif ---------- */
    case PAUSE_7:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 43.0f;
        segDone = 0.0f;
        phase = FORWARD_50;
        headingTarget = Z;
      }
      break;
    case FORWARD_50:
      {
        float remaining = segTarget - segDone;
        if (remaining > POS_THRESH) {
          int spd = CRUISE_TICKS;
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH), (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_8;
        }
        break;
      }

    /* --- deuxième rotation -90° (CW) améliorée --- */
    case PAUSE_8:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_90_CW2;
      }
      break;
    case ROTATE_90_CW2:
      {
        static uint8_t okCnt2 = 0;
        float Zc = normRad(Zstart + PI / 2.0f);
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt2 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt2 >= STABLE_STEPS) {
            okCnt2 = 0;
            pauseStart = millis();
            phase = PAUSE_9;
          }
        }
        break;
      }

      /* préparation avance 40 cm relatif */
    case PAUSE_9:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 35.0f;
        segDone = 0.0f;
        phase = FORWARD_40;
        headingTarget = Z;
      }
      break;
    case FORWARD_40:
      {
        float remaining = segTarget - segDone;
        if (remaining > POS_THRESH) {
          int spd = CRUISE_TICKS;
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH), (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_10;
        }
        break;
      }

    /* ---------- suite séquence (étapes 12‑20) ---------- */
    /* pause 2 s */
    case PAUSE_10:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 26.0f;
        segDone = 0.0f;
        phase = BACKWARD_20;
      }
      break;

    case BACKWARD_20:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = -CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_11;
      }
      break;

      /* pause 2 s */
    case PAUSE_11:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_200_CW;
      }
      break;

    /* rotation -190 */
    case ROTATE_200_CW:
      {
        static uint8_t okCnt3 = 0;
        float Zc = normRad(Zstart + (210.0f * PI / 180.0f));
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt3 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt3 >= STABLE_STEPS) {
            okCnt3 = 0;
            pauseStart = millis();
            phase = PAUSE_12;
          }
        }
        break;
      }

      /* pause 2 s */
    case PAUSE_12:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 70.0f;
        segDone = 0.0f;
        phase = FORWARD_45;
        headingTarget = Z;
      }
      break;

    case FORWARD_45:
      {
        float remaining = segTarget - segDone;
        if (remaining > POS_THRESH) {
          int spd = CRUISE_TICKS;
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH),
                             (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_13;
        }
        break;
      }

      /* pause 2 s */
    case PAUSE_13:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_94_CCW;
      }
      break;

    // on se met a 90 deg pour se raligner
    case ROTATE_94_CCW:
      {
        static uint8_t okCnt3 = 0;
        float Zc = normRad(Zstart - (100.0f * PI / 180.0f));
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt3 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt3 >= STABLE_STEPS) {
            okCnt3 = 0;
            pauseStart = millis();
            phase = PAUSE_14;
          }
        }
        break;
      }

      /* new */
    case PAUSE_14:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 51.0f;
        segDone = 0.0f;
        phase = FORWARD_65;
        headingTarget = Z;
      }
      break;

    case FORWARD_65:
      {
        float remaining = segTarget - segDone;
        if (remaining > POS_THRESH) {
          int spd = CRUISE_TICKS;
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH), (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_15;
        }
        break;
      }

      /* new */
    case PAUSE_15:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_90_CCWW;
      }
      break;

    /* rotation -20° CW proportionnelle */
    case ROTATE_90_CCWW:
      {
        static uint8_t okCnt3 = 0;
        float Zc = normRad(Zstart - (90.0f * PI / 180.0f));
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt3 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt3 >= STABLE_STEPS) {
            okCnt3 = 0;
            pauseStart = millis();
            phase = PAUSE_16;
          }
        }
        break;
      }

    /* retour canette */
    case PAUSE_16:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 85.0f;
        segDone = 0.0f;
        phase = FORWARD_100;
        headingTarget = Z;
      }
      break;

    case FORWARD_100:
      {
        float remaining = segTarget - segDone;
        if (remaining > POS_THRESH) {
          int spd = CRUISE_TICKS;
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH), (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_17;
        }
        break;
      }

      /*robot canette good*/

    /* pause 2 s */
    case PAUSE_17:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 20.0f;
        segDone = 0.0f;
        phase = BACKWARD_20_BIS;
      }
      break;

    case BACKWARD_20_BIS:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = -CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        pauseStart = millis();
        phase = PAUSE_18;
      }
      break;

      /* pause 2 s */
    case PAUSE_18:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_220_CW;
      }
      break;

    /* rotation -190 */
    case ROTATE_220_CW:
      {
        static uint8_t okCnt3 = 0;
        float Zc = normRad(Zstart + (220.0f * PI / 180.0f));
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt3 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt3 >= STABLE_STEPS) {
            okCnt3 = 0;
            pauseStart = millis();
            phase = PAUSE_19;
          }
        }
        break;
      }

      /* pause 2 s */
    case PAUSE_19:
      if (millis() - pauseStart >= 1000UL) {
        segTarget = 40.0f;
        segDone = 0.0f;
        phase = FORWARD_120;
        headingTarget = Z;
      }
      break;

    case FORWARD_120:
      {
        float remaining = segTarget - segDone;
        if (remaining > POS_THRESH) {
          int spd = CRUISE_TICKS;
          float errCap = normRad(headingTarget - Z);
          int corr = 0;
          if (fabs(errCap) > HEADING_DEADBAND_RAD) {
            corr = (int)(K_CAP * errCap);
            corr = constrain(corr, -(int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH), (int)(CRUISE_TICKS * HEADING_CORRECTION_AUTH));
          }
          cmdVG = spd - corr;
          cmdVD = spd + corr;
        } else {
          cmdVG = cmdVD = 0;
          pauseStart = millis();
          phase = PAUSE_20;
        }
        break;
      }

    case PAUSE_20:
      if (millis() - pauseStart >= 1000UL) {
        Zstart = Z;
        phase = ROTATE_20_BEFORE_END;
      }
      break;

    /* rotation -190 */
    case ROTATE_20_BEFORE_END:
      {
        static uint8_t okCnt3 = 0;
        float Zc = normRad(Zstart - (20.0f * PI / 180.0f));
        float err = normRad(Zc - Z);
        if (fabs(err) > ANG_THRESH_RAD) {
          okCnt3 = 0;
          int mUnsigned = (int)(K_ROT_P * fabs(err));
          mUnsigned = constrain(mUnsigned, MIN_ROT_SPEED_TICKS, ROT_TICKS);
          int m = (err > 0) ? mUnsigned : -mUnsigned;
          cmdVG = -m;
          cmdVD = m;
        } else {
          cmdVG = cmdVD = 0;
          if (++okCnt3 >= STABLE_STEPS) {
            okCnt3 = 0;
            pauseStart = millis();
            phase = PAUSE_MATCH;
          }
        }
        break;
      }

      /* attendre ≥95 s de match */
    case PAUSE_MATCH:
      if (millis() - matchStart >= 95000UL) {
        segTarget = 80.0f;
        segDone = 0.0f;
        phase = FORWARD_60_END;
      }
      break;

    /* avance finale 60 cm */
    case FORWARD_60_END:
      if (segDone < segTarget - POS_THRESH) {
        cmdVG = cmdVD = CRUISE_TICKS;
      } else {
        cmdVG = cmdVD = 0;
        phase = FINISHED;
        matchAlreadyRun = true;
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println(">>> Séquence terminée !");
      }

      break;

    default:
      cmdVG = cmdVD = 0;
      break;
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
  if (!l)
    analogWrite(PWM_L, 0);
  else {
    digitalWrite(DIR_L, l > 0 ? LOW : HIGH);
    analogWrite(PWM_L, min(abs(l), 255));
  }
  if (!r)
    analogWrite(PWM_R, 0);
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
  while (isdigit(streamChar[idx]))
    cmdVG = cmdVG * 10 + (streamChar[idx++] - '0');
  if (neg)
    cmdVG = -cmdVG;
  idx++;
  neg = false;
  if (streamChar[idx] == '-') {
    neg = true;
    idx++;
  }
  while (isdigit(streamChar[idx]))
    cmdVD = cmdVD * 10 + (streamChar[idx++] - '0');
  if (neg)
    cmdVD = -cmdVD;
}

/* ---------- ISRs ---------------------------------------------- */
void interruptR() {
  digitalRead(encRA) == digitalRead(encRB) ? countR++ : countR--;
}
void interruptL() {
  digitalRead(encLA) == digitalRead(encLB) ? countL-- : countL++;
}
