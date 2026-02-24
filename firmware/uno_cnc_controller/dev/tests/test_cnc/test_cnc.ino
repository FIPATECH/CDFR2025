#include <AccelStepper.h>
#include <MultiStepper.h>

// --- Définition des drivers STEP/DIR pour chaque axe ---
AccelStepper stepX(AccelStepper::DRIVER, 2, 5);  // STEP→D2, DIR→D5
AccelStepper stepY(AccelStepper::DRIVER, 3, 6);  // STEP→D3, DIR→D6
AccelStepper stepZ(AccelStepper::DRIVER, 4, 7);  // STEP→D4, DIR→D7

MultiStepper XY_steppers;

void setup() {
  // ---- Configuration des vitesses et accélérations ----
  // X et Y (synchronisés)
  stepX.setMaxSpeed(800);       // max 800 pas/s
  stepX.setAcceleration(400);   // 400 pas/s²
  stepY.setMaxSpeed(800);
  stepY.setAcceleration(400);

  // Z (moteur ponctuel)
  stepZ.setMaxSpeed(600);
  stepZ.setAcceleration(300);

  // Ajout de X et Y à MultiStepper pour synchronisation
  XY_steppers.addStepper(stepX);
  XY_steppers.addStepper(stepY);
}

void loop() {
  const long fullRev = 200;  // 360° = 200 pas (1,8°/pas)

  // 1) X + Y → +360°
  {
    long targets[2] = { fullRev, fullRev };
    XY_steppers.moveTo(targets);          // fixe la cible pour X et Y
    XY_steppers.runSpeedToPosition();     // déplace X & Y en parallèle (bloquant) :contentReference[oaicite:1]{index=1}
  }
  delay(200);

  // 2) Z → +360°
  stepZ.moveTo(fullRev);                  // fixe la cible pour Z
  while (stepZ.distanceToGo() != 0) {
    stepZ.run();                          // exécute Z pas à pas avec rampe
  }
  delay(200);

  // 3) X + Y → –360°
  {
    long targets_neg[2] = { -fullRev, -fullRev };
    XY_steppers.moveTo(targets_neg);
    XY_steppers.runSpeedToPosition();     // inverse X & Y en parallèle
  }
  delay(200);

  // 4) Z → –360°
  stepZ.moveTo(-fullRev);
  while (stepZ.distanceToGo() != 0) {
    stepZ.run();
  }

  delay(1000);  // pause avant répétition
}
