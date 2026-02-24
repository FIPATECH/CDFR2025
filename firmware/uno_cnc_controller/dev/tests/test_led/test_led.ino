// Exemple de sketch Arduino Nano pour piloter une bande de LEDs WS2812B avec FastLED

#include <FastLED.h>

// Paramètres de la bande de LEDs
#define NUM_LEDS 3        // Nombre de LEDs sur la bande
#define DATA_PIN 6         // Broche numérique D6 (pas A6) connectée à DIN de la bande
#define LED_TYPE WS2812B   // Type de LEDs
#define COLOR_ORDER GRB    // Ordre des couleurs sur la bande
CRGB leds[NUM_LEDS];      // Tableau de LEDs FastLED

// Couleur statique souhaitée (R, G, B)
#define STATIC_R 0
#define STATIC_G 94
#define STATIC_B 255

void setup() {
  // Initialisation de FastLED
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255); // Réglage de la luminosité (0-255)
}

void loop() {
  staticColor(STATIC_R, STATIC_G, STATIC_B);  // Affiche la couleur statique définie
}

// Fonction : affiche une couleur fixe sur toute la bande
void staticColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
  // Aucun délai ni animation pour maintenir la couleur en continu
}
