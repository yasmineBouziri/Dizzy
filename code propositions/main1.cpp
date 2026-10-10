#include <Arduino.h>

// CONFIG

// Sensor pins, LEFT to RIGHT physically on the robot 
const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23; //pullup pin, pressed = LOW

const uint8_t led = 2;

const uint8_t PWMA = 21, AIN1 = 16, AIN2 = 17;  // LEFT motor
const uint8_t PWMB = 22,  BIN1 = 19, BIN2 = 18;  // RIGHT motor
//const uint8_t PWMA = 22, AIN1 = 19, AIN2 = 18;  // LEFT motor
//const uint8_t PWMB = 21,  BIN1 = 16, BIN2 = 17;  // RIGHT motor

// PID tuning

int turn_value=0;
float Kp =17;
float PID;
float Kd = 0;//1;

// Speed settings
// baseSpeed is how fast it drives straight (0-255).
int baseSpeed    = 100;
int maxSpeed     = 255;
int minTurnSpeed = 60;   

unsigned long lastLineSeen = millis();

// INTERNAL VARIABLES

int sensorThreashold[8];
int sensorValue[8];
unsigned long blackMillis; 
unsigned long whiteMillis;  

int line_position;
int sensor_sum;

float lastError = 0;
float lastKnownPos = 0;// default to center if the line is ever fully lost
float pos=0; 

int blackoutEventCount = 0;
bool inBlackout = false;
bool invertedLine = false; // set to true if the line is white on black instead of black on white
bool justStarted;
bool Ti1 = false;
bool CIRCLE = false;
bool Ti2 = false;
bool TEAR = false;
bool C1 = false;
bool C2 = false;

// Function Declarations
void waitForButtonPress();
void calibrateSensors();
void followLine();
void checkBlackoutEvents();
void detectInvertedLine();
void driveMotors(int leftSpeed, int rightSpeed);
void readSensor();
void drivePID();

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  pinMode(led, OUTPUT);

  pinMode(startButtonPin, INPUT_PULLUP);

  digitalWrite(led, HIGH);


  calibrateSensors();

  waitForButtonPress();

  delay(300);  
  justStarted = true;
}


void loop() {
  //detectInvertedLine();
  followLine();
}


// HELPER FUNCTIONS

void waitForButtonPress() {
  while (digitalRead(startButtonPin) == HIGH) { delay(10); } 
  delay(50);                                                  
  while (digitalRead(startButtonPin) == LOW)  { delay(10); } 
  delay(200);                                                 
}

// Reads the line position from the sensors.
void readSensor() {
  sensor_sum = 0;
  int weighted = 0;
  const int weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

  for (uint8_t i = 0; i < 8; i++) {
    sensorValue[i] = analogRead(sensorPins[i]) > sensorThreashold[i];
    weighted   += sensorValue[i] * weight[i];
    sensor_sum += sensorValue[i];
  }

  if (sensor_sum > 0) {
    pos = weighted;// sensor_sum;
    lastKnownPos = pos;
  } else {
    pos = lastKnownPos;   // line lost, keep last known position
  }
}

void followLine(){

  readSensor();
  if(justStarted){
    driveMotors(baseSpeed,baseSpeed);
    delay(400);
    justStarted = false;
  }else{
    if(sensor_sum>4 && !Ti1){ // detecting the first T
      driveMotors(baseSpeed,baseSpeed);
      digitalWrite(led, LOW);
      delay(50);
      Ti1 = true;
    }else if(sensor_sum>4 && Ti1 && !CIRCLE){ // detecting the circle
      driveMotors(-minTurnSpeed,minTurnSpeed);
      digitalWrite(led, HIGH);
      delay(400);
      driveMotors(minTurnSpeed,minTurnSpeed);
      delay(200);
      CIRCLE = true;
      driveMotors(0,0);
      delay(1000);
      CIRCLE = true;
    }else{
      drivePID();
    } 
  }
}

void drivePID(){
  float error = pos;
  float corr = Kp * error + Kd * (error - lastError);
  lastError = error;
  driveMotors(baseSpeed + corr, baseSpeed - corr);
}

void detectInvertedLine() {
  if (sensorValue[0] && sensorValue[1] && sensorValue[2] && !sensorValue[3] && !sensorValue[4] && sensorValue[5] && sensorValue[6] && sensorValue[7]) invertedLine = !invertedLine; 
}

// Detects entering/leaving a "wide black zone" (marker) and reacts based on which one this is in sequence.
void checkBlackoutEvents() {
  bool isBlackout = (sensor_sum == 8);  // 8 sensors see black = wide black zone

  if (isBlackout && !inBlackout) {
    inBlackout = true;  // just entered a wide black zone
  }

  if (!isBlackout && inBlackout) {
    inBlackout = false;  // just exited, count this as one event
    blackoutEventCount++;

    if (blackoutEventCount <9) {
      return;  

    } else if (blackoutEventCount == 9) {
      driveMotors(0, 0);
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

  whiteMillis = millis();
  while(millis()-whiteMillis < 2000) {
    WSAMPLES++;
    digitalWrite(led, LOW);
    delay(100);
    digitalWrite(led, HIGH);
    delay(100);
    for (uint8_t i = 0; i < 8; i++) {
      whiteSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }

   waitForButtonPress();

  blackMillis = millis(); 
  while(millis()-blackMillis < 2000) {
    BSAMPLES++;
    digitalWrite(led, LOW);
    delay(100);
    digitalWrite(led, HIGH);
    delay(100);
    for (uint8_t i = 0; i < 8; i++) {
      blackSum[i] += analogRead(sensorPins[i]);
    }
    delay(2);
  }
  

  for (uint8_t i = 0; i < 8; i++) {
    int avgBlack = blackSum[i] / BSAMPLES;
    int avgWhite = whiteSum[i] / WSAMPLES;

    sensorThreashold[i] = (avgBlack + avgWhite) / 2;
  }
}

// Sends a speed command to motors.
void driveMotors(int speedLeft, int speedRight) {
  bool forwardLeft = speedLeft >= 0;
  bool forwardRight = speedRight >= 0;
  int pwmLeft = constrain(abs(speedLeft), 0, 255);
  int pwmRight = constrain(abs(speedRight), 0, 255);

  if (pwmLeft > 0 && pwmLeft < minTurnSpeed) pwmLeft = minTurnSpeed;
  if (pwmRight > 0 && pwmRight < minTurnSpeed) pwmRight = minTurnSpeed;

  digitalWrite(AIN1, forwardLeft ? HIGH : LOW);
  digitalWrite(AIN2, forwardLeft ? LOW : HIGH);
  analogWrite(PWMA, pwmLeft);

  digitalWrite(BIN1, forwardRight ? HIGH : LOW);
  digitalWrite(BIN2, forwardRight ? LOW : HIGH);
  analogWrite(PWMB, pwmRight);
}


