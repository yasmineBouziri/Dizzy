#include <Arduino.h>
const uint8_t sensorPins[8] = {13, 14, 26, 27, 25, 32, 33, 4};

const uint8_t startButtonPin = 23; //pullup pin, pressed = LOW

const uint8_t PWMA = 21, AIN1 = 16, AIN2 = 17;  // LEFT motor
const uint8_t PWMB = 22,  BIN1 = 18, BIN2 = 19;  // RIGHT motor

// Speed settings
// baseSpeed is how fast it drives straight (0-255).
int baseSpeed    = 150;
int maxSpeed     = 255;
int minTurnSpeed = 60;   
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


  Serial.println("GO!");
  delay(300);  
}

void loop() {
    driveMotor(true, 100);  // Drive left motor forward at speed 100
    driveMotor(false, -150); // Drive right motor backward at speed 150
    delay(2000);             // Run motors for 2 seconds

    driveMotor(true, -100);  // Drive left motor backward at speed 100
    driveMotor(false, 150);   // Drive right motor forward at speed 150
    delay(2000);              // Run motors for 2 seconds

    driveMotor(true, 0);      // Stop left motor
    driveMotor(false, 0);     // Stop right motor
    delay(2000);              // Wait for 2 seconds before repeating
}
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

