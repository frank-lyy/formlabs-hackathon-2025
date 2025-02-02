#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
AccelStepper stepperA(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperB(AccelStepper::DRIVER, 3, 6);
MultiStepper steppers;

// Command length
#define CMD_LEN 11
#define ACTION_LEN 3

// Servo settings
#define SERVO_MIN 125
#define SERVO_MAX 625
#define SERVO_FREQ 60
#define SERVO_A_A_ID 0
#define SERVO_A_B_ID 1
#define SERVO_A_C_ID 2
#define SERVO_A_D_ID 3
#define SERVO_A_WRIST_ID 4
#define SERVO_B_A_ID 5
#define SERVO_B_B_ID 6
#define SERVO_B_C_ID 7
#define SERVO_B_D_ID 8
#define SERVO_B_WRIST_ID 9

// Stepper motor settings
#define EN_PIN     8
#define STEPS_PER_REV       200
#define STEPPER_PULSE_WIDTH 100
#define STEPPER_PERIOD      5000
#define STEPPER_A_RATIO     1.0
#define STEPPER_B_RATIO     1.0

void moveArm(uint8_t* data) {
  int signA = data[0];
  int angleA = data[1];
  int signB = data[2];
  int angleB = data[3];

  if (signA == 1) {
    angleA *= -1;
  }
  if (signB == 1) {
    angleB *= -1;
  }

  long pos[2];
  pos[0] = (int)(angleA * STEPPER_A_RATIO * STEPS_PER_REV / 360.0);
  pos[1] = (int)(angleB * STEPPER_B_RATIO * STEPS_PER_REV / 360.0);
  steppers.moveTo(pos);
  steppers.runSpeedToPosition();
  delay(100);

  uint8_t response[] = {0xFF, 0x00};
  Serial.write(response, sizeof(response));
}

void moveEndoWrist(uint8_t* data) {
  int servoIdx = data[0];
  int potA = (data[1] << 8) + data[2];
  int potB = (data[3] << 8) + data[4];
  int potC = (data[5] << 8) + data[6];
  int potD = (data[7] << 8) + data[8];

  int pWideA, pWideB, pWideC, pWideD;     // Pulse_Wide  
  int pWidthA, pWidthB, pWidthC, pWidthD; // Pulse_Width
     
  int halfFlexion, remappedPotC;
  int F1, F2; // Flexion Variables
  
  // A -> ABDUCTION
  pWideA = map(potA, 0, 1023, SERVO_MIN, SERVO_MAX);
  pWidthA = int(float(pWideA) / 1000000 * SERVO_FREQ * 4096);

  // JAWS
  if (potC > 511) { // jaw open
    remappedPotC = constrain(potC, 511, 750);
    remappedPotC = map(remappedPotC, 511, 750, 0, 170);
    halfFlexion = (remappedPotC / 2); }
  else { // jaw closed
    remappedPotC = 0;
    halfFlexion = 0;
  }

  // F1 B -> FLEXION_1
  F1 = potB + ((potA-511)/2) + remappedPotC; // Jaw and abduction compensation
  F1 = constrain(F1, 0, 1023);
  pWideB = map(F1, 0, 1023, SERVO_MIN, SERVO_MAX);
  pWidthB = int(float(pWideB) / 1000000 * SERVO_FREQ * 4096);
  
  // F2 C -> FLEXION_2
  F2 = potB + ((potA-511)/2) - remappedPotC; // Jaw and abduction compensation
  F2 = constrain(F2, 0, 1023);
  pWideC = map(F2, 0, 1023, SERVO_MIN, SERVO_MAX);
  pWidthC = int(float(pWideC) / 1000000 * SERVO_FREQ * 4096);  

  // D -> SHAFT ROTATION
  pWideD = map(potD, 0, 1023, SERVO_MIN, SERVO_MAX);
  pWidthD = int(float(pWideD) / 1000000 * SERVO_FREQ * 4096);
 
  if (servoIdx == 0) {
    servoDriver.setPWM(SERVO_A_A_ID, 0, pWidthA);
    servoDriver.setPWM(SERVO_A_B_ID, 0, pWidthB);
    servoDriver.setPWM(SERVO_A_C_ID, 0, pWidthC);
    servoDriver.setPWM(SERVO_A_D_ID, 0, pWidthD);
  } else {
    servoDriver.setPWM(SERVO_B_A_ID, 0, pWidthA);
    servoDriver.setPWM(SERVO_B_B_ID, 0, pWidthB);
    servoDriver.setPWM(SERVO_B_C_ID, 0, pWidthC);
    servoDriver.setPWM(SERVO_B_D_ID, 0, pWidthD);
  }
  delay(500);

  uint8_t response[] = {0xFF, 0x01};
  Serial.write(response, sizeof(response));
}

void moveWrist(uint8_t* data) {
  int servoIdx = data[0];
  int angle = data[1];
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  if (servoIdx == 0) {
    servoDriver.setPWM(SERVO_A_WRIST_ID, 0, pulse);
  } else {
    servoDriver.setPWM(SERVO_B_WRIST_ID, 0, pulse);
  }
  delay(500);

  uint8_t response[] = {0xFF, 0x02};
  Serial.write(response, sizeof(response));
}

// This defines the command indices
void (*actions[])(uint8_t*) = {
  moveArm,
  moveEndoWrist,
  moveWrist
};

void setup() {
  Serial.begin(9600);

  // Servo motor setup
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);
 
  // Set enable pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); 

  // Stepper motor setup
  stepperA.setMaxSpeed(100);
  stepperB.setMaxSpeed(100);
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
}

uint8_t inBytes[CMD_LEN];
int numBytes = 0;
void loop() {
  if (Serial.available() > 0) {
    // Push new data to the bytes queue
    for (int i = 0; i < CMD_LEN - 1; i++) {
      inBytes[i] = inBytes[i + 1];
    }
    inBytes[CMD_LEN - 1] = Serial.read();
    numBytes++;

    // New full command has been received
    if (inBytes[0] == 0xFF && numBytes >= CMD_LEN) {
      numBytes = 0;
      uint8_t actionIndex = inBytes[1];
      if (actionIndex >= 0 && actionIndex < ACTION_LEN) {
        actions[actionIndex](&inBytes[2]); // Pass the remaining command bytes
      }
    }
  }
}
