// CONFIG
// Sensor pins, LEFT to RIGHT physically on the robot 

const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23;

const uint8_t PWMA = 21, AIN1 = 16, AIN2 = 17;  // LEFT motor
const uint8_t PWMB = 22,  BIN1 = 18, BIN2 = 19;  // RIGHT motor

// PID tuning
float Kp = 0.045;
float Ki = 0.0;
float Kd = 0.25;

// Speed settings
// baseSpeed is how fast it drives straight (0-255).
int baseSpeed    = 150;
int maxSpeed     = 255;
int minTurnSpeed = 60;   

// Marker detection threshold 
const int MARKER_SENSOR_THRESHOLD = 8;
const int SENSOR_BLACK_THRESHOLD  = 500;  // 0-1000 scale, "is this black?"


// INTERNAL VARIABLES

int sensorMin[8], sensorMax[8]; // filled in during calibration
int sensorValue[8]; // normalized 0 (white) - 1000 (black)

float lastError = 0;
float integral  = 0;
int   lastKnownPos = 0;

int blackoutEventCount = 0;
bool inBlackout = false;

const int pwmFreq = 5000; // how many times per second it switches on/off
const int pwmResBits = 8; // means speed will be expressed as a number from 0 to 255 (2^8 = 256 steps)
const int pwmChA = 0, pwmChB = 1; // assigned channels to differentiate


void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // PWM works by switching power on/off very fast, the % of time it's "on" controls the effective speed.
  ledcSetup(pwmChA, pwmFreq, pwmResBits);
  ledcSetup(pwmChB, pwmFreq, pwmResBits);
  ledcAttachPin(PWMA, pwmChA);
  ledcAttachPin(PWMB, pwmChB);

  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  pinMode(startButtonPin, INPUT_PULLUP);

  // For uploading new code over WiFi, hold the start button down while powering on the robot.
  delay(50);
  if (digitalRead(startButtonPin) == LOW) {
    enterOTAMode();
  }

  calibrateSensors();

  Serial.println("Calibration done.");
  Serial.println("Press the start button to begin driving.");
  while (digitalRead(startButtonPin) == HIGH) {
    delay(10);   
  }
  Serial.println("GO!");
  delay(300);  
}


void loop() {
  int position = readLinePosition();   // -1000 (far left)  +1000 (far right)
  checkBlackoutEvents();               // detects wide black zones (markers)

  float error = position;
  integral += error;
  integral = constrain(integral, -2000, 2000);  // stop it building up forever
  float derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  driveMotor(true,  leftSpeed);
  driveMotor(false, rightSpeed);

  // Uncomment this line while tuning to see live sensor + position data:
  // printDebug(position, correction);
}


// HELPER FUNCTIONS

/* Reads all 8 sensors, normalizes each to 0-1000, and returns a single
   "where is the line" number: negative = line is to the left, positive =
   line is to the right, 0 = centered. */
int readLinePosition() {
  long weightedSum = 0;
  long total = 0;
  bool lineSeen = false;

  const int weight[8] = {-1000, -714, -428, -142, 142, 428, 714, 1000};

  for (uint8_t i = 0; i < 8; i++) {
    int raw = analogRead(sensorPins[i]);
    int norm = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
    norm = constrain(norm, 0, 1000);
    sensorValue[i] = norm;

    if (norm > SENSOR_BLACK_THRESHOLD) lineSeen = true;

    weightedSum += (long)norm * weight[i];
    total += norm;
  }

  if (!lineSeen) {
    return lastKnownPos;
  }

  int pos = weightedSum / total;
  lastKnownPos = pos;
  return pos;
}

// Counts how many sensors currently see black.
int countActiveSensors() {
  int count = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (sensorValue[i] > SENSOR_BLACK_THRESHOLD) count++;
  }
  return count;
}

// Detects entering/leaving a "wide black zone" (marker) and reacts based on which one this is in sequence.
void checkBlackoutEvents() {
  bool isBlackout = (countActiveSensors() >= MARKER_SENSOR_THRESHOLD);

  if (isBlackout && !inBlackout) {
    inBlackout = true;  // just entered a wide black zone
  }

  if (!isBlackout && inBlackout) {
    inBlackout = false;  // just exited, count this as one event
    blackoutEventCount++;
    Serial.printf("Blackout event #%d\n", blackoutEventCount);

    if (blackoutEventCount == 1) {
      simulateValveClose();   // the diamond / leak marker
    } else if (blackoutEventCount == 2) {
      driveMotor(true, 0);
      driveMotor(false, 0);
      Serial.println("Finished.");
      while (true) { delay(1000); }  // stop here permanently
    }
  }
}

// Stop -> reverse briefly -> forward again, simulating the valve closing
// at a leak marker, as described in the rulebook.
void simulateValveClose() {
  driveMotor(true, 0);  driveMotor(false, 0);
  delay(300);
  driveMotor(true, -120); driveMotor(false, -120);
  delay(250);
  driveMotor(true, 0);  driveMotor(false, 0);
  delay(200);
  driveMotor(true, baseSpeed); driveMotor(false, baseSpeed);
  delay(300);
}

// Sweeps sensors for 4 seconds at startup to learn each one's black/white
// range, since every sensor reads slightly differently.
void calibrateSensors() {
  for (uint8_t i = 0; i < 8; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  Serial.println("Calibrating... slide the robot over the line now.");
  unsigned long t0 = millis();
  while (millis() - t0 < 4000) {
    for (uint8_t i = 0; i < 8; i++) {
      int v = analogRead(sensorPins[i]);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
    delay(5);
  }

  for (uint8_t i = 0; i < 8; i++) {
    if (sensorMax[i] - sensorMin[i] < 50) sensorMax[i] = sensorMin[i] + 50;
    Serial.printf("Sensor %d (pin %d): min=%d max=%d\n",
                  i, sensorPins[i], sensorMin[i], sensorMax[i]);
  }
}

// Sends a speed command to one motor.
// isLeft: true = left motor, false = right motor.
// speed: -255 to 255. Positive = forward, negative = reverse.
void driveMotor(bool isLeft, int speed) {
  bool forward = speed >= 0;
  int pwm = constrain(abs(speed), 0, 255);
  if (pwm > 0 && pwm < minTurnSpeed) pwm = minTurnSpeed;

  if (isLeft) {
    digitalWrite(AIN1, forward ? HIGH : LOW);
    digitalWrite(AIN2, forward ? LOW  : HIGH);
    ledcWrite(pwmChA, pwm);
  } else {
    digitalWrite(BIN1, forward ? HIGH : LOW);
    digitalWrite(BIN2, forward ? LOW  : HIGH);
    ledcWrite(pwmChB, pwm);
  }
}

// Optional: print live data to Serial Monitor while tuning PID.
void printDebug(int position, float correction) {
  Serial.printf("pos=%d  corr=%.1f\n", position, correction);
}

// ===================== OTA (WiFi upload) MODE ===========================
// >>> ADJUST: fill in your WiFi network name and password below before
// using this. Only needed if you actually want wireless uploads — for
// day-to-day testing, USB is simpler and doesn't need a network nearby.

#include <WiFi.h>
#include <ArduinoOTA.h>

const char* otaSSID     = "YOUR_WIFI_NAME";      // >>> ADJUST
const char* otaPassword = "YOUR_WIFI_PASSWORD";  // >>> ADJUST

void enterOTAMode() {
  Serial.println("Entering OTA upload mode (motors disabled, sensors unused).");

  WiFi.mode(WIFI_STA);
  WiFi.begin(otaSSID, otaPassword);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname("line-follower");
  ArduinoOTA.begin();

  Serial.println("Ready for wireless upload. Waiting...");

  while (true) {
    ArduinoOTA.handle();  // this is what actually receives the new code
    delay(10);
  }
}