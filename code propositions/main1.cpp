// CONFIG
// Sensor pins, LEFT to RIGHT physically on the robot 
#include <Arduino.h>
const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23; //pullup pin, pressed = LOW

const uint8_t PWMA = 21, AIN1 = 16, AIN2 = 17;  // LEFT motor
const uint8_t PWMB = 22,  BIN1 = 19, BIN2 = 18;  // RIGHT motor

// PID tuning
float Kp = 0.045;
float Ki = 0.0;
float Kd = 0.25;

// Speed settings
// baseSpeed is how fast it drives straight (0-255).
int baseSpeed    = 150;
int maxSpeed     = 255;
int minTurnSpeed = 60;   



// INTERNAL VARIABLES

int sensorThreashold[8];
int sensorValue[8];
unsigned long blackMillis = 0; 
unsigned long whiteMillis = 0;  

int line_position;
int sensor_sum;

float lastError = 0;
float integral  = 0;
float   lastKnownPos = 0;

int blackoutEventCount = 0;
bool inBlackout = false;
bool invertedLine = false; // set to true if the line is white on black instead of black on white


// Function Declarations
void waitForButtonPress();
void calibrateSensors();
void followLine();
void checkBlackoutEvents();
void detectInvertedLine();
void driveMotor(bool isLeft, int speed);
float readSensor();

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  pinMode(startButtonPin, INPUT_PULLUP);
  Serial.println("Press the start button to begin calibration.");
  calibrateSensors();

  Serial.println("Calibration done.");
  Serial.println("Press the start button to begin driving.");

  waitForButtonPress();

  Serial.println("GO!");
  delay(300);  
}
void loop() {
  detectInvertedLine();
  int position = readLinePosition(); 
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
float readSensor() {
  sensor_sum=0;
  line_position=0;

  const int weight[8] = {1, 2, 3, 4, 5 ,6 ,7, 8};

  for (uint8_t i = 0; i < 8; i++) {
    sensorValue[i] = analogRead(sensorPins[i]) > sensorThreashold[i]; // black:1 , white:0

    if (invertedLine) sensorValue[i]= !sensorValue[i];

    line_position += sensorValue[i] * weight[i];
    sensor_sum += sensorValue[i];
  }

  if (sensor_sum==0) {
    return lastKnownPos;
  }

  float pos = line_position / sensor_sum;
  lastKnownPos = pos;
  return pos;
}

void followLine(){
  
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

    if (blackoutEventCount <9) {
      return;  // first two markers are ignored, just drive forward through them

    }/* else if (blackoutEventCount == 3) {
      // Drive forward slightly to clear marker
      driveMotor(true, baseSpeed); 
      driveMotor(false, baseSpeed);
      delay(100);

      // Nudge hard right onto the right branch of the circle
      driveMotor(true, 170);   
      driveMotor(false, 40);   
      delay(280);

    }*/ else if (blackoutEventCount == 9) {
      driveMotor(true, 0);
      driveMotor(false, 0);
      Serial.println("Finished.");
      while (true) { delay(1000); }  // stop here permanently
    }
  }
}


void calibrateSensors() {
  int BSAMPLES = 0;
  int WSAMPLES = 0;
  long blackSum[8] = {0};
  long whiteSum[8] = {0};

  waitForButtonPress();

  Serial.println("Sampling BLACK surface...");
  blackMillis = millis(); 
  while(millis()-blackMillis < 2000) {
    BSAMPLES++;
    for (uint8_t i = 0; i < 8; i++) {
      blackSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }
Serial.println("Black sampling done.");
  waitForButtonPress();

  Serial.println("Sampling WHITE surface...");
  whiteMillis = millis();
  while(millis()-whiteMillis < 2000) {
    WSAMPLES++;
    for (uint8_t i = 0; i < 8; i++) {
      whiteSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }
  
  Serial.println("White sampling done.");

  for (uint8_t i = 0; i < 8; i++) {
    int avgBlack = blackSum[i] / BSAMPLES;
    int avgWhite = whiteSum[i] / WSAMPLES;

    sensorThreashold[i] = (avgBlack + avgWhite) / 2;

    Serial.printf("Sensor %d | Black: %4d | White: %4d | Threshold: %4d\n", 
                  i, avgBlack, avgWhite, sensorThreashold[i]);
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
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(BIN1, forward ? HIGH : LOW);
    digitalWrite(BIN2, forward ? LOW  : HIGH);
    analogWrite(PWMB, pwm);
  }
}


