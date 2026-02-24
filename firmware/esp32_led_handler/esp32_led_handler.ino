#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <FastLED.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Configuration LEDs WS2812B
#define LED_PIN     4
#define NUM_LEDS    45
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// Réseau AP
const char* ssid     = "LEDController";
const char* password = "fipoutech";

const byte DNS_PORT = 53;
DNSServer dnsServer;

AsyncWebServer server(80);

Preferences prefs;

bool ledsOn;
CRGB currentColor;
String animMode;
uint8_t hue = 0;

bool clientConnected = false;
bool builtinManual = false;
bool builtinOn;
unsigned long previousBlink = 0;
const unsigned long blinkInterval = 500;
bool blinkState = false;

// Format HEX
String colorToHex(const CRGB& c) {
  char buf[8];
  sprintf(buf, "#%02X%02X%02X", c.r, c.g, c.b);
  return String(buf);
}

// Animations FastLED
void juggle() {
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for (int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

// Handler événements AP
void onAPEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    clientConnected = true;
    builtinManual = false;
    builtinOn = true;
    digitalWrite(2, HIGH);
    prefs.putBool("builtinOn", true);
    Serial.println("[AP] Client connecté");
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    clientConnected = false;
    builtinManual = false;
    builtinOn = false;
    digitalWrite(2, LOW);
    prefs.putBool("builtinOn", false);
    previousBlink = millis();
    Serial.println("[AP] Client déconnecté");
  }
}

// Endpoint /state
void handleState(AsyncWebServerRequest* req) {
  Serial.println("[HTTP] /state requested");
  StaticJsonDocument<128> doc;
  doc["ledsOn"] = ledsOn;
  doc["builtinOn"] = builtinOn;
  doc["color"] = colorToHex(currentColor);
  doc["anim"] = animMode;
  String out;
  serializeJson(doc, out);
  req->send(200, "application/json", out);
}

// Page HTML embarquée
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Contrôle des LED</title>
  <style>
    body { background: #fff; display: flex; flex-direction: column; align-items: center; height: 100vh; margin: 0; font-family: 'Segoe UI', Roboto, sans-serif; color: #0e1a28; }
    .title { font-size: 2em; font-weight: bold; margin: 20px 0 30px; }
    .toggle-container { display: flex; align-items: center; margin: 10px 0; }
    .toggle-label { width: 50px; text-align: center; }
    .switch { position: relative; width: 60px; height: 34px; margin: 0 10px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background: #ccc; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ''; height: 26px; width: 26px; left: 4px; bottom: 4px; background: #fff; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background: #50B1DD; }
    input:checked + .slider:before { transform: translateX(26px); }
    #colorSection, #animSection { display: flex; flex-direction: column; align-items: center; margin-bottom: 20px; }
    input[type="color"] { width: 120px; height: 40px; border: none; border-radius: 8px; cursor: pointer; }
    .button-send { margin-top: 10px; padding: 6px 12px; font-size: 1em; color: #fff; background: #1e2a38; border: none; border-radius: 4px; cursor: pointer; }
    .anim-buttons { display: flex; justify-content: space-between; width: 60%; }
    .anim-buttons button { flex: 1; margin: 0 5px; color: #fff; background: #1e2a38; border: none; padding: 10px 0; border-radius: 5px; }
  </style>
</head>
<body>
  <div class="title">Contrôle des LED</div>
  <div class="toggle-container">
    <span class="toggle-label">OFF</span>
    <label class="switch"><input type="checkbox" id="toggle"><span class="slider"></span></label>
    <span class="toggle-label">ON</span>
  </div>
  <div class="mode-section">
    <label><input type="radio" name="mode" id="modeStatic" value="static"> Couleur statique</label>
  </div>
  <div id="colorSection">
    <input type="color" id="colorPicker">
    <button class="button-send" id="sendColor">Envoyer</button>
  </div>
  <div class="mode-section">
    <label><input type="radio" name="mode" id="modeAnim" value="anim"> Animation</label>
  </div>
  <div id="animSection">
    <div class="anim-buttons">
      <button onclick="setAnim('rainbow')">Rainbow</button>
      <button onclick="setAnim('juggle')">Juggle</button>
    </div>
  </div>
  <script>
    const toggle = document.getElementById('toggle');
    const colorPicker = document.getElementById('colorPicker');
    const sendColor = document.getElementById('sendColor');
    const modeStatic = document.getElementById('modeStatic');
    const modeAnim = document.getElementById('modeAnim');

    console.log('[JS] Initialisation UI');
    fetch('/state')
      .then(resp => resp.json())
      .then(s => {
        console.log('[JS] État reçu:', s);
        toggle.checked = s.ledsOn;
        colorPicker.value = s.color;
        modeStatic.checked = (s.anim === 'static');
        modeAnim.checked = (s.anim !== 'static');
      })
      .catch(e => console.error('[JS] Erreur /state:', e));

    toggle.onchange = () => {
      console.log('[JS] toggle:', toggle.checked);
      fetch(toggle.checked ? '/on' : '/off')
        .then(() => console.log('[JS] /' + (toggle.checked ? 'on' : 'off') + ' envoyé'))
        .catch(e => console.error('[JS] Erreur toggle:', e));
    };

    modeStatic.onchange = () => {
      if (modeStatic.checked) {
        console.log('[JS] mode static sélectionné');
        fetch('/anim?mode=static')
          .then(() => console.log('[JS] /anim?mode=static envoyé'))
          .catch(e => console.error('[JS] Erreur anim static:', e));
      }
    };

    sendColor.onclick = () => {
      const val = colorPicker.value;
      const encoded = encodeURIComponent(val);
      console.log('[JS] Envoi couleur:', val, 'encoded:', encoded);
      const url = `/color?value=${encoded}`;
      console.log('[JS] URL =>', url);
      fetch(url)
        .then(resp => resp.text())
        .then(txt => console.log('[JS] /color réponse:', txt))
        .catch(e => console.error('[JS] Erreur color:', e));
    };

    function setAnim(mode) {
      console.log('[JS] setAnim:', mode);
      fetch(`/anim?mode=${mode}`)
        .then(() => console.log('[JS] /anim?mode=' + mode + ' envoyé'))
        .catch(e => console.error('[JS] Erreur anim:', e));
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  prefs.begin("ledCtrl", false);
  ledsOn = prefs.getBool("ledsOn", true);
  uint32_t col = prefs.getUInt("color", 0xFFFFFF);
  currentColor = CRGB((col >> 16) & 0xFF, (col >> 8) & 0xFF, col & 0xFF);
  animMode = prefs.getString("animMode", "static");
  builtinOn = prefs.getBool("builtinOn", false);

  Serial.begin(115200);
  delay(100);
  Serial.println("[Setup] Démarrage");

  pinMode(2, OUTPUT);
  digitalWrite(2, builtinOn ? HIGH : LOW);

  WiFi.onEvent(onAPEvent, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent(onAPEvent, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  WiFi.softAP(ssid, password);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  Serial.println(String("[WiFi] AP démarré: ") + ssid);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  if (ledsOn) {
    Serial.println("[LED] LEDs ON, animMode=" + animMode);
    if (animMode == "static") fill_solid(leds, NUM_LEDS, currentColor);
    FastLED.show();
  }

  server.onNotFound([](AsyncWebServerRequest* r) { r->redirect("/"); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html", index_html); });
  server.on("/state", HTTP_GET, handleState);
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest* r) {
    ledsOn = true; prefs.putBool("ledsOn", true);
    Serial.println("[HTTP] /on reçu");
    r->send(200, "text/plain", "OK");
  });
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest* r) {
    ledsOn = false; prefs.putBool("ledsOn", false);
    Serial.println("[HTTP] /off reçu");
    r->send(200, "text/plain", "OK");
  });
  server.on("/color", HTTP_GET, [](AsyncWebServerRequest* r) {
    Serial.println("[HTTP] /color demandé");
    if (r->hasParam("value")) {
      String v = r->getParam("value")->value();
      Serial.println("[HTTP] valeur reçue: " + v);
      long n = strtol(v.c_str() + (v.startsWith("#")?1:0), NULL, 16);
      currentColor = CRGB((n >> 16) & 0xFF, (n >> 8) & 0xFF, n & 0xFF);
      animMode = "static";
      prefs.putUInt("color", (uint32_t)n);
      prefs.putString("animMode", "static");
      Serial.println("[LED] Couleur mise à jour: " + v);
      r->send(200, "text/plain", "OK");
    } else {
      Serial.println("[HTTP] /color ERR: paramètre manquant");
      r->send(400, "text/plain", "ERR");
    }
  });
  server.on("/anim", HTTP_GET, [](AsyncWebServerRequest* r) {
    Serial.println("[HTTP] /anim demandé");
    if (r->hasParam("mode")) {
      animMode = r->getParam("mode")->value();
      prefs.putString("animMode", animMode);
      Serial.println("[LED] Anim mis à jour: " + animMode);
      r->send(200, "text/plain", "OK");
    } else {
      Serial.println("[HTTP] /anim ERR: mode manquant");
      r->send(400, "text/plain", "ERR");
    }
  });

  server.begin();
  previousBlink = millis();
}

void loop() {
  dnsServer.processNextRequest();
  if (!clientConnected && !builtinManual && (millis() - previousBlink) >= blinkInterval) {
    previousBlink = millis();
    blinkState = !blinkState;
    digitalWrite(2, blinkState);
  }
  if (ledsOn) {
    if (animMode == "static") {
      fill_solid(leds, NUM_LEDS, currentColor);
    } else if (animMode == "rainbow") {
      fill_rainbow(leds, NUM_LEDS, hue++, 7);
    } else if (animMode == "juggle") {
      juggle();
    }
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
  FastLED.show();
  delay(20);
}