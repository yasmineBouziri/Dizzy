#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

const char* ssid     = "Orange-642A";
const char* password = "3gGE9hFRLigA";

#define enableA 11 
#define enableB 10
#define direction1motorA 8
#define direction2motorA 9
#define direction1motorB 12
#define direction2motorB 13

void setup() {
    Serial.begin(115200);
    pinMode(enableA, OUTPUT);
    pinMode(enableB, OUTPUT);
    pinMode(direction1motorA, OUTPUT);
    pinMode(direction2motorA, OUTPUT);
    pinMode(direction1motorB, OUTPUT);
    pinMode(direction2motorB, OUTPUT); 
}

void loop() {
    // Example motor control logic
    digitalWrite(direction1motorA, HIGH);
    digitalWrite(direction2motorA, LOW);
    analogWrite(enableA, 128); // Set speed for motor A

    digitalWrite(direction1motorB, HIGH);
    digitalWrite(direction2motorB, LOW);
    analogWrite(enableB, 128); // Set speed for motor B

    delay(2000); // Run motors for 2 seconds

    // Stop motors
    analogWrite(enableA, 0);
    analogWrite(enableB, 0);

    delay(2000); // Wait for 2 seconds before next loop
}