#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

const char* ssid     = "Orange-642A";
const char* password = "3gGE9hFRLigA";

const int LED_PIN = 2;
const unsigned long BLINK_MS = 1000;   // change this value to test OTA

unsigned long lastToggle = 0;
bool ledState = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnecté, IP : " + WiFi.localIP().toString());

  ArduinoOTA.setHostname("esp32-ota");
  ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]()  { Serial.println("OTA start"); });
  ArduinoOTA.onEnd([]()    { Serial.println("\nOTA end"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    digitalWrite(LED_PIN, (p / 20000) % 2);   // LED flickers during upload
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("Erreur[%u]\n", e);
  });

  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();

  if (millis() - lastToggle >= BLINK_MS) {
    lastToggle = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}