// CONFIG
// Sensor pins, LEFT to RIGHT physically on the robot 

const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23; //pullup pin, pressed = LOW

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
bool invertedLine = false; // set to true if the line is white on black instead of black on white

const int pwmFreq = 5000; // how many times per second it switches on/off
const int pwmResBits = 8; // means speed will be expressed as a number from 0 to 255 (2^8 = 256 steps)
const int pwmChA = 0, pwmChB = 1; // assigned channels to differentiate

// Function Declarations
void waitForButtonPress();
void calibrateSensors();
int readLinePosition();
int countActiveSensors();
void checkBlackoutEvents();
void simulateValveClose();
void detectInvertedLine();
void driveMotor(bool isLeft, int speed);


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

  calibrateSensors();

  Serial.println("Calibration done.");
  Serial.println("Press the start button to begin driving.");

  waitForButtonPress();

  Serial.println("GO!");
  delay(300);  
}


void loop() {
  detectInvertedLine();
  int position = readLinePosition();   // -1000 (far left)  +1000 (far right)
  checkBlackoutEvents(); 

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

void waitForButtonPress() {
  while (digitalRead(startButtonPin) == HIGH) { delay(10); } 
  delay(50);                                                  
  while (digitalRead(startButtonPin) == LOW)  { delay(10); } 
  delay(200);                                                 
}

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

    if (invertedLine) norm = 1000 - norm;

    if (norm > SENSOR_BLACK_THRESHOLD) lineSeen = true;

    weightedSum += (long)norm * weight[i];
    total += norm;
  }

  if (!lineSeen || total==0) {
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

void detectInvertedLine() {
  if (sensorValue[0]>=SENSOR_BLACK_THRESHOLD && sensorValue[1]>=SENSOR_BLACK_THRESHOLD && sensorValue[2]>=SENSOR_BLACK_THRESHOLD && sensorValue[3]<SENSOR_BLACK_THRESHOLD && sensorValue[4]<SENSOR_BLACK_THRESHOLD && sensorValue[5]>=SENSOR_BLACK_THRESHOLD && sensorValue[6]>=SENSOR_BLACK_THRESHOLD && sensorValue[7]>=SENSOR_BLACK_THRESHOLD) invertedLine = true; 
  if (sensorValue[0]<SENSOR_BLACK_THRESHOLD && sensorValue[1]<SENSOR_BLACK_THRESHOLD && sensorValue[2]<SENSOR_BLACK_THRESHOLD && sensorValue[3]>=SENSOR_BLACK_THRESHOLD && sensorValue[4]>=SENSOR_BLACK_THRESHOLD && sensorValue[5]<SENSOR_BLACK_THRESHOLD && sensorValue[6]<SENSOR_BLACK_THRESHOLD && sensorValue[7]<SENSOR_BLACK_THRESHOLD) invertedLine = false;
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

    if (blackoutEventCount <= 2 || (blackoutEventCount >= 4 && blackoutEventCount <= 7)) {
      return;  // first two markers are ignored, just drive forward through them

    } else if (blackoutEventCount == 3) {
      // Drive forward slightly to clear marker
      driveMotor(true, baseSpeed); 
      driveMotor(false, baseSpeed);
      delay(100);

      // Nudge hard right onto the right branch of the circle
      driveMotor(true, 170);   
      driveMotor(false, 40);   
      delay(280);

    } else if (blackoutEventCount == 9) {
      driveMotor(true, 0);
      driveMotor(false, 0);
      Serial.println("Finished.");
      while (true) { delay(1000); }  // stop here permanently
    }
  }
}


void calibrateSensors() {
  const int SAMPLES = 200;
  long blackSum[8] = {0};
  long whiteSum[8] = {0};

  waitForButtonPress();

  Serial.println("Sampling BLACK surface...");
  for (int s = 0; s < SAMPLES; s++) {
    for (uint8_t i = 0; i < 8; i++) {
      blackSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }

  waitForButtonPress();

  Serial.println("Sampling WHITE surface...");
  for (int s = 0; s < SAMPLES; s++) {
    for (uint8_t i = 0; i < 8; i++) {
      whiteSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }

  for (uint8_t i = 0; i < 8; i++) {
    int avgBlack = blackSum[i] / SAMPLES;
    int avgWhite = whiteSum[i] / SAMPLES;

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

