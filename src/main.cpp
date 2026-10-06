#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

const char* ssid     = "Orange-642A";
const char* password = "3gGE9hFRLigA";
#define AIN1 16
#define BIN1 18
#define AIN2 17
#define BIN2 19
#define PWMA 21
#define PWMB 22


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnecté, IP : " + WiFi.localIP().toString());

  ArduinoOTA.setHostname("esp32-ota");
  ArduinoOTA.setPassword("admin");   // choisis le tien

  ArduinoOTA.onStart([]()  { Serial.println("OTA start"); });
  ArduinoOTA.onEnd([]()    { Serial.println("\nOTA end"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("Progression : %u%%\r", (p * 100) / t);
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("Erreur[%u]\n", e);
  });

  ArduinoOTA.begin();
}

void loop() {
  
  ArduinoOTA.handle();
}