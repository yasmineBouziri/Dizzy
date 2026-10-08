#include <Arduino.h>

// CONFIG

// Sensor pins, LEFT to RIGHT physically on the robot 
const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23; //pullup pin, pressed = LOW

const uint8_t led = 2;

const uint8_t PWMA = 21, AIN1 = 16, AIN2 = 17;  // LEFT motor
const uint8_t PWMB = 22,  BIN1 = 19, BIN2 = 18;  // RIGHT motor

// PID tuning

int turn_value=0;
float Kp = 30;
float PID;
float Kd = 0;

// Speed settings
// baseSpeed is how fast it drives straight (0-255).
int baseSpeed    = 150;
int maxSpeed     = 255;
int minTurnSpeed = 60;   



// INTERNAL VARIABLES

int sensorThreashold[8];
int sensorValue[8];
unsigned long blackMillis; 
unsigned long whiteMillis;  

int line_position;
int sensor_sum;

float lastError = 0;
float lastKnownPos = 4.5;// default to center if the line is ever fully lost
float pos= 4.5; 

int blackoutEventCount = 0;
bool inBlackout = false;
bool invertedLine = false; // set to true if the line is white on black instead of black on white


// Function Declarations
void waitForButtonPress();
void calibrateSensors();
void followLine();
void checkBlackoutEvents();
void detectInvertedLine();
void driveMotors(int leftSpeed, int rightSpeed);
void readSensor();

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  pinMode(led, OUTPUT);

  pinMode(startButtonPin, INPUT_PULLUP);

  Serial.println("Press the start button to begin calibration.");
  digitalWrite(led, HIGH);

  calibrateSensors();

  Serial.println("Calibration done.");
  Serial.println("Press the start button to begin driving.");

  waitForButtonPress();

  Serial.println("GO!");
  delay(300);  
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
  sensor_sum=0;
  line_position=0;

  const int weight[8] = {1, 2, 3, 4, 5 ,6 ,7, 8};

  for (uint8_t i = 0; i < 8; i++) {
    sensorValue[i] = analogRead(sensorPins[i]) > sensorThreashold[i]; // black:1 , white:0

    if (invertedLine) sensorValue[i]= !sensorValue[i];

    line_position += sensorValue[i] * weight[i];
    sensor_sum += sensorValue[i];
  }
  if (sensor_sum){
    pos = (float) line_position / sensor_sum;
    lastKnownPos = pos;
  }
}

void followLine(){
    readSensor();
    
    int leftCount  = sensorValue[0] + sensorValue[1] + sensorValue[2];
    int rightCount = sensorValue[5] + sensorValue[6] + sensorValue[7];

    float error = 4.5 - pos; // center is 4.5
    PID = Kp * error + Kd * (error - lastError);
    lastError = error;

    Serial.printf("Pos: %.2f | Error: %.2f | PID: %.2f | SensorSum: %d\n", pos, error, PID, sensor_sum);  

    int leftSpeed  = round(constrain(baseSpeed - PID, -maxSpeed, maxSpeed));
    int rightSpeed = round(constrain(baseSpeed + PID, -maxSpeed, maxSpeed));
    driveMotors(leftSpeed, rightSpeed);

    //left turn detec
    if (sensorValue[0]==1 && sensorValue[7]==0) turn_value=1;
    //right turn detec
    if (sensorValue[0]==0 && sensorValue[7]==1) turn_value=2;

    //actually turning left
    if (turn_value==1){
      delay(100);
      driveMotors(-minTurnSpeed, minTurnSpeed);
      while (sensorValue[3]==0 && sensorValue[4]==0) readSensor();
      turn_value=0;
    }

    //actually turnnig right
    else if (turn_value==2){
      delay(100);
      driveMotors(minTurnSpeed, -minTurnSpeed);
      while (sensorValue[3]==0 && sensorValue[4]==0) readSensor();
      turn_value=0;
    }

    //u turn go back to prev pos
    else if (sensor_sum==0 && turn_value==0){
      delay(100);
      driveMotors(-minTurnSpeed, minTurnSpeed);
      while (sensorValue[3]==0 && sensorValue[4]==0) readSensor();
      turn_value=0;
    }

    if (sensor_sum==8){
      delay(500); // inertia can keep you going it's a temporary blackout
      readSensor();
      if (sensor_sum==8){
        driveMotors(0,0);
        while (sensor_sum==8) readSensor();
      }
      // sth to add if necessary
    }
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
    Serial.printf("Blackout event #%d\n", blackoutEventCount);

    if (blackoutEventCount <9) {
      return;  

    } else if (blackoutEventCount == 9) {
      driveMotors(0, 0);
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
    digitalWrite(led, LOW);
    delay(100);
    digitalWrite(led, HIGH);
    delay(100);
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
    digitalWrite(led, LOW);
    delay(100);
    digitalWrite(led, HIGH);
    delay(100);
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


