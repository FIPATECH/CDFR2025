#include <Wire.h>
#include <AccelStepper.h>
#include <Servo.h>

// ======= Déclaration des moteurs pas-à-pas =======
AccelStepper stepX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepZ(AccelStepper::DRIVER, 4, 7);

// ======= Déclaration des servos =======
Servo servoLeft;
Servo servoRight;

// ======= Constantes de micro-pas & vitesses =======
const int MICROSTEP = 16;
const long STEPS_PER_REV = 200L * MICROSTEP;
const long RACK_STEPS = 170L * MICROSTEP;

const float FACTOR = 2.0;
const float SPEED_MULT = 24.0;

// Vitesse et accélération pour XY (lent / rapide)
const float XY_SPEED_SLOW = 800.0 * FACTOR * SPEED_MULT;
const float XY_ACCEL_SLOW = 400.0 * FACTOR * SPEED_MULT;
const float XY_SPEED_FAST = 1200.0 * FACTOR * SPEED_MULT;
const float XY_ACCEL_FAST = 600.0 * FACTOR * SPEED_MULT;

// Vitesse et accélération pour Z (normal / rapide)
const float Z_SPEED_NORMAL = 600.0 * FACTOR * SPEED_MULT;
const float Z_ACCEL_NORMAL = 300.0 * FACTOR * SPEED_MULT;
const float Z_SPEED_FAST = 1000.0 * FACTOR * SPEED_MULT;
const float Z_ACCEL_FAST = 600.0 * FACTOR * SPEED_MULT;

// ===== Calcul pour déplacement en cm =====
const float WHEEL_DIAMETER_MM = 24.0;
const float WHEEL_CIRCUM_MM = WHEEL_DIAMETER_MM * PI;
const float STEPS_PER_MM = STEPS_PER_REV / WHEEL_CIRCUM_MM;
const float CALIBRATION = 1.6667;  // mesuré : 50 mm commandés → 30 mm réels
const float STEPS_PER_MM_CAL = STEPS_PER_MM * CALIBRATION;

// Position réelle actuelle (en cm)
float posZ_cm = 7.0;
const float Z_MIN_CM = 0.0;
const float Z_MAX_CM = 30.0;

// ======= Tirette =======
const int endstopZ = 11;
int lastSwitchState = HIGH;

// Tampon de réception I2C
String i2cBuffer;

// Fonctions utilitaires
void moveY(long s) {
  stepY.move(-s);
}

void syncXY() {
  while (stepX.distanceToGo() || stepY.distanceToGo()) {
    stepX.run();
    stepY.run();
  }
}

void syncX() {
  while (stepX.distanceToGo())
    stepX.run();
}

void syncY() {
  while (stepY.distanceToGo())
    stepY.run();
}

void platform_down() {
  stepX.move(+STEPS_PER_REV);
  moveY(+STEPS_PER_REV);
  syncXY();
}

void platform_up() {
  stepX.move(-STEPS_PER_REV);
  moveY(-STEPS_PER_REV);
  syncXY();
}

void rack_motor_deploy() {
  stepZ.move(-RACK_STEPS);
  while (stepZ.distanceToGo())
    stepZ.run();
}

void rack_motor_retract() {
  stepZ.move(+RACK_STEPS);
  while (stepZ.distanceToGo())
    stepZ.run();
}

void test_xy() {
  stepX.setMaxSpeed(XY_SPEED_FAST);
  stepX.setAcceleration(XY_ACCEL_FAST);
  stepY.setMaxSpeed(XY_SPEED_FAST);
  stepY.setAcceleration(XY_ACCEL_FAST);
  stepX.move(+STEPS_PER_REV);
  moveY(+STEPS_PER_REV);
  syncXY();
  delay(100);
  stepX.move(-STEPS_PER_REV);
  moveY(-STEPS_PER_REV);
  syncXY();
  stepX.setMaxSpeed(XY_SPEED_SLOW);
  stepX.setAcceleration(XY_ACCEL_SLOW);
  stepY.setMaxSpeed(XY_SPEED_SLOW);
  stepY.setAcceleration(XY_ACCEL_SLOW);
}

void test_x() {
  stepX.setMaxSpeed(XY_SPEED_FAST);
  stepX.setAcceleration(XY_ACCEL_FAST);
  stepX.move(+STEPS_PER_REV);
  syncX();
  delay(100);
  stepX.move(-STEPS_PER_REV);
  syncX();
  stepX.setMaxSpeed(XY_SPEED_SLOW);
  stepX.setAcceleration(XY_ACCEL_SLOW);
}

void test_y() {
  stepY.setMaxSpeed(XY_SPEED_FAST);
  stepY.setAcceleration(XY_ACCEL_FAST);
  moveY(+STEPS_PER_REV);
  syncY();
  delay(100);
  moveY(-STEPS_PER_REV);
  syncY();
  stepY.setMaxSpeed(XY_SPEED_SLOW);
  stepY.setAcceleration(XY_ACCEL_SLOW);
}

void test_z() {
  stepZ.setMaxSpeed(Z_SPEED_FAST);
  stepZ.setAcceleration(Z_ACCEL_FAST);
  rack_motor_deploy();
  delay(100);
  rack_motor_retract();
  stepZ.setMaxSpeed(Z_SPEED_NORMAL);
  stepZ.setAcceleration(Z_ACCEL_NORMAL);
}

void banner_up() {
  servoLeft.write(0);
  servoRight.write(180);
}

void banner_down() {
  servoLeft.write(180);
  servoRight.write(0);
}

// Déplace la plateforme verticalement de dz_cm (cm)
void plateform_x_cm(float dz_cm) {
  // conversion cm → mm → pas (avec calibration)
  long steps = lround(dz_cm * 10.0 * STEPS_PER_MM_CAL);
  // positif = monter
  stepX.move(-steps);
  moveY(-steps);
  syncXY();
}

// Déplace vers la hauteur absolue z_cm (entre 0 et 30 cm)
void setZ_cm(float z_cm) {
  // Clamp de la consigne dans la course mécanique
  if (z_cm < Z_MIN_CM) z_cm = Z_MIN_CM;
  if (z_cm > Z_MAX_CM) z_cm = Z_MAX_CM;

  float dz = z_cm - posZ_cm;    // delta à parcourir
  if (fabs(dz) < 1e-3) return;  // si quasi nul, on sort

  Serial.print("Moving from ");
  Serial.print(posZ_cm);
  Serial.print(" cm to ");
  Serial.print(z_cm);
  Serial.println(" cm");

  plateform_x_cm(dz);  // utilise votre fonction existante
  posZ_cm = z_cm;      // met à jour la position courante
}



// Callback I2C : lecture jusqu'à '\n'
void receiveEvent(int howMany) {
  i2cBuffer = "";
  while (Wire.available()) {
    char c = (char)Wire.read();
    if (c == '\n') break;
    i2cBuffer += c;
  }
  i2cBuffer.trim();
  // Log I2C
  Serial.print("I2C Rx: ");
  Serial.println(i2cBuffer);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Init Tirette
  pinMode(endstopZ, INPUT_PULLUP);

  // Initialisation moteurs XY
  stepX.setMaxSpeed(XY_SPEED_SLOW);
  stepX.setAcceleration(XY_ACCEL_SLOW);
  stepY.setMaxSpeed(XY_SPEED_SLOW);
  stepY.setAcceleration(XY_ACCEL_SLOW);
  // Initialisation moteur Z
  stepZ.setMaxSpeed(Z_SPEED_NORMAL);
  stepZ.setAcceleration(Z_SPEED_NORMAL);
  // Initialisation servos
  servoLeft.attach(12);
  servoRight.attach(13);

  // Esclave I2C adresse 0x08
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);

  Serial.println("Arduino I2C Slave Initialized at address 0x08");

  banner_up();

  // Une fois en butée, on considère que la plateforme est à 7 cm (point de départ)
  posZ_cm = 7.0;
  stepX.setCurrentPosition(lround(-posZ_cm * 10.0 * STEPS_PER_MM_CAL));
  // ou, si vous n'avez pas de CAL mais STEPS_PER_MM:
  //   lround(-posZ_cm * 10.0 * STEPS_PER_MM)
}

void loop() {
  // 1) Détection du capteur → envoi "start\n" au STM32
  int currentState = digitalRead(endstopZ);
  if (currentState != lastSwitchState) {
    delay(50);  // anti-rebond
    if (currentState == LOW) {
      Serial.println("Capteur activé → envoi start");
      // Passe en maître I2C pour envoyer le démarrage
      Wire.beginTransmission(0x07);
      Wire.write("start\n");
      Wire.endTransmission();
    }
    lastSwitchState = currentState;
  }

  // 2) Traitement prioritaire : I2C reçu en esclave
  String cmd;
  if (i2cBuffer.length()) {
    Serial.print("Loop: processing I2C cmd -> ");
    Serial.println(i2cBuffer);
    cmd = i2cBuffer;
    i2cBuffer = "";
  }
  // 3) Sinon, traitement Série manuel
  else if (Serial.available()) {
    Serial.print("Loop: processing Serial cmd -> ");
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.println(cmd);
  } else {
    return;
  }

  // 4) Exécution des commandes
  if (cmd == "banner_up") {
    Serial.println("Exec: banner_up()");
    banner_up();
  } else if (cmd == "banner_down") {
    Serial.println("Exec: banner_down()");
    banner_down();
  } else if (cmd == "open_rack") {
    Serial.println("Exec: open_rack()");
    rack_motor_deploy();
  } else if (cmd == "close_rack") {
    Serial.println("Exec: close_rack()");
    rack_motor_retract();
  } else if (cmd == "platform_down") {
    Serial.println("Exec: platform_down()");
    platform_down();
  } else if (cmd == "platform_up") {
    Serial.println("Exec: platform_up()");
    platform_up();
  } else if (cmd == "test_xy") {
    Serial.println("Exec: test_xy()");
    test_xy();
  } else if (cmd == "test_x") {
    Serial.println("Exec: test_x()");
    test_x();
  } else if (cmd == "test_y") {
    Serial.println("Exec: test_y()");
    test_y();
  } else if (cmd == "test_z") {
    Serial.println("Exec: test_z()");
    test_z();
  } else if (cmd.startsWith("plateform_x_cm")) {
    int p1 = cmd.indexOf('(');
    int p2 = cmd.indexOf(')');
    if (p1 > 0 && p2 > p1) {
      float dz = cmd.substring(p1 + 1, p2).toFloat();
      Serial.print("Exec: plateform_x_cm(");
      Serial.print(dz);
      Serial.println(")");
      plateform_x_cm(dz);
    } else {
      Serial.println("Format attendu : plateform_x_cm(valeur_cm)");
    }
  }

  else if (cmd.startsWith("setZ_cm")) {
    int p1 = cmd.indexOf('(');
    int p2 = cmd.indexOf(')');
    if (p1 > 0 && p2 > p1) {
      float target = cmd.substring(p1 + 1, p2).toFloat();
      Serial.print("Exec: setZ_cm(");
      Serial.print(target);
      Serial.println(")");
      setZ_cm(target);
    } else {
      Serial.println("Format attendu : setZ_cm(valeur_cm)");
    }
  } else {
    Serial.print("Unknown cmd: ");
    Serial.println(cmd);
  }
}
