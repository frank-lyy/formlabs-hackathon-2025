#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
AccelStepper stepperA(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperB(AccelStepper::DRIVER, 3, 6);
MultiStepper steppers;

// Command length
#define CMD_LEN 10
#define ACTION_LEN 2

// Servo settings
#define SERVO_MIN_PULSE_WIDTH 650
#define SERVO_MAX_PULSE_WIDTH 2350
#define SERVO_FREQ 60
#define SERVO_A_ID 0
#define SERVO_B_ID 1
#define SERVO_C_ID 2
#define SERVO_D_ID 3

// Stepper motor settings
#define EN_PIN     8
#define STEPS_PER_REV       200
#define STEPPER_PULSE_WIDTH 100
#define STEPPER_PERIOD      5000
#define STEPPER_A_RATIO     1.0
#define STEPPER_B_RATIO     1.0

void moveStepper(uint8_t* data) {
  int angleA = data[0];
  int angleB = data[1];

  long pos[2];
  pos[0] = (int)(angleA * STEPPER_A_RATIO * STEPS_PER_REV / 360.0);
  pos[1] = (int)(angleB * STEPPER_B_RATIO * STEPS_PER_REV / 360.0);
  steppers.moveTo(pos);
  steppers.runSpeedToPosition();

  uint8_t response[] = {0xFF, 0x00};
  Serial.write(response, sizeof(response));
}

void moveWrist(uint8_t* data) {
  int potA = (data[0] << 8) + data[1];
  int potB = (data[2] << 8) + data[3];
  int potC = (data[4] << 8) + data[5];
  int potD = (data[6] << 8) + data[7];

  int pWideA, pWideB, pWideC, pWideD;     // Pulse_Wide  
  int pWidthA, pWidthB, pWidthC, pWidthD; // Pulse_Width
     
  int halfFlexion, remappedPotC;
  int F1, F2; // Flexion Variables
  
  // A -> ABDUCTION
  pWideA = map(potA, 0, 1023, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
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
  pWideB = map(F1, 0, 1023, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  pWidthB = int(float(pWideB) / 1000000 * SERVO_FREQ * 4096);
  
  // F2 C -> FLEXION_2
  F2 = potB + ((potA-511)/2) - remappedPotC; // Jaw and abduction compensation
  F2 = constrain(F2, 0, 1023);
  pWideC = map(F2, 0, 1023, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  pWidthC = int(float(pWideC) / 1000000 * SERVO_FREQ * 4096);  

  // D -> SHAFT ROTATION
  pWideD = map(potD, 0, 1023, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  pWidthD = int(float(pWideD) / 1000000 * SERVO_FREQ * 4096);
 
  servoDriver.setPWM(SERVO_A_ID, 0, pWidthA);
  servoDriver.setPWM(SERVO_B_ID, 0, pWidthB);
  servoDriver.setPWM(SERVO_C_ID, 0, pWidthC);
  servoDriver.setPWM(SERVO_D_ID, 0, pWidthD);

  delay(100);
  uint8_t response[] = {0xFF, 0x01};
  Serial.write(response, sizeof(response));
}

// This defines the command indices
void (*actions[])(uint8_t*) = {
  moveStepper,
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
void loop() {
  if (Serial.available() > 0) {
    // Push new data to the bytes queue
    for (int i = 0; i < CMD_LEN - 1; i++) {
      inBytes[i] = inBytes[i + 1];
    }
    inBytes[CMD_LEN - 1] = Serial.read();

    // New full command has been received
    if (inBytes[0] == 0xFF) {
      uint8_t actionIndex = inBytes[1];
      if (actionIndex >= 0 && actionIndex < ACTION_LEN) {
        actions[actionIndex](&inBytes[2]); // Pass the remaining command bytes
      }
    }
  }
}
