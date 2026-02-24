#include <Encoder.h>  // https://github.com/PaulStoffregen/Encoder

// Broches
const uint8_t PIN_ENC_A   = 2;  // D2 (INT0)
const uint8_t PIN_ENC_B   = 3;  // D3 (INT1)
const uint8_t PIN_INDEX   = 4;  // D4, lecture “à la demande”

// Encoder quadrature
Encoder enc(PIN_ENC_A, PIN_ENC_B);

// Variables pour mesure de ticks/s
unsigned long lastTime  = 0;
long         lastPos    = 0;

// Index
volatile bool gotIndex = false;
void indexISR() {
  gotIndex = true;
}

void setup() {
  Serial.begin(115200);

  // Mode pull-up pour réduire le bruit sur A/B
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // Index en entrée (si bruit, on peut aussi mettre INPUT_PULLUP)
  pinMode(PIN_INDEX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INDEX), indexISR, RISING);

  // Initialisation du timer
  lastTime = millis();
  lastPos  = enc.read();
}

void loop() {
  unsigned long now = millis();

  // 1. Mesure de ticks/s toutes les secondes
  if (now - lastTime >= 1000) {
    long currentPos = enc.read();
    long dTicks     = currentPos - lastPos;
    Serial.print("ticks/s = ");
    Serial.println(dTicks);

    lastPos  = currentPos;
    lastTime = now;
  }

  // 2. Gestion de l’Index
  if (gotIndex) {
    Serial.println("Index détecté !");
    // Remise à zéro du compteur quadrature
    enc.write(0);
    // Réinitialiser la référence pour la mesure de cadence
    lastPos = 0;
    gotIndex = false;
  }

  // Autres tâches non bloquantes possibles…
}
