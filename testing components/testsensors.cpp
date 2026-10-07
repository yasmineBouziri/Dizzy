#include <Arduino.h>
const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23; 
int sensorMin[8], sensorMax[8]; // filled in during calibration
int sensorValue[8];

void calibrateSensors();
void waitForButtonPress();

  void setup() {
   Serial.begin(115200);  
  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  pinMode(startButtonPin, INPUT_PULLUP);
  
  Serial.println("Press the start button to begin calibration.");
  calibrateSensors();
  Serial.println("Calibration done.");
}

void loop() {
  // Nothing to do here
}

void calibrateSensors() {
  int BSAMPLES = 0;
  int WSAMPLES = 0;
  long blackSum[8] = {0};
  long whiteSum[8] = {0};

  waitForButtonPress();

  Serial.println("Sampling BLACK surface...");
  while(millis() < 2000) {
    BSAMPLES++;
    for (uint8_t i = 0; i < 8; i++) {
      blackSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }
Serial.println("Black sampling done.");
  waitForButtonPress();

  Serial.println("Sampling WHITE surface...");
  while(millis() < 2000) {
    WSAMPLES++;
    for (uint8_t i = 0; i < 8; i++) {
      whiteSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }
    for (uint8_t i = 0; i < 8; i++) {
      whiteSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }
  Serial.println("White sampling done.");

  for (uint8_t i = 0; i < 8; i++) {
    int avgBlack = blackSum[i] / BSAMPLES;
    int avgWhite = whiteSum[i] / WSAMPLES;

    // Dynamically assign min/max regardless of sensor polarity
    sensorMin[i] = min(avgBlack, avgWhite);
    sensorMax[i] = max(avgBlack, avgWhite);

    // Guard against divide-by-zero during mapping
    if (sensorMax[i] - sensorMin[i] < 100) {
      sensorMax[i] = sensorMin[i] + 100;
    }

    int avgThreshold = (avgBlack + avgWhite) / 2;

    Serial.printf("Sensor %d | Black: %4d | White: %4d | Threshold: %4d\n", 
                  i, avgBlack, avgWhite, avgThreshold);
  }
}

void waitForButtonPress() {
  while (digitalRead(startButtonPin) == HIGH) { delay(10); } 
  delay(50);                                                  
  while (digitalRead(startButtonPin) == LOW)  { delay(10); } 
  delay(200);                                                 
}