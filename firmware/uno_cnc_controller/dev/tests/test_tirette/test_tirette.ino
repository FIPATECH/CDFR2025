// Configuration du bouton sur la broche 2 avec pull-up interne
const int buttonPin = 11;

// Variables d'état
int buttonState = HIGH;
int lastButtonState = HIGH;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // pull-up interne
  Serial.begin(9600);
}

void loop() {
  // Lecture de l’état actuel
  buttonState = digitalRead(buttonPin);

  // Détection de la transition (edge detection)
  if (buttonState != lastButtonState) {
    // Petit délai pour antirebond
    delay(50);

    // Action selon la nouvelle valeur
    if (buttonState == LOW) {
      Serial.println("Changement d'état → PRESSE");  // passage HIGH→LOW
    } else {
      Serial.println("Changement d'état → RELÂCHE");  // passage LOW→HIGH
    }

    // Mise à jour pour la prochaine comparaison
    lastButtonState = buttonState;
  }
}
