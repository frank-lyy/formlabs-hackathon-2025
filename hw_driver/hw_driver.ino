#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);

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
#define STEP_X_PIN 2
#define STEP_Y_PIN 3
#define DIR_X_PIN  5
#define DIR_Y_PIN  6

// Stepper motor settings
#define STEPS_PER_REV       200
#define STEPPER_PULSE_WIDTH 100
#define STEPPER_PERIOD      5000
#define STEPPER_A_RATIO     1.0
#define STEPPER_B_RATIO     1.0
double STEPS_PER_DEGREE_A = STEPS_PER_REV * STEPPER_A_RATIO / 360.0;
double STEPS_PER_DEGREE_B = STEPS_PER_REV * STEPPER_B_RATIO / 360.0;

int currAngleA = 0;
int currAngleB = 0;
void moveStepper(uint8_t* data) {
  int angleA = data[0];
  int angleB = data[1];

  int requiredStepsA = (int)((angleA - currAngleA) * STEPS_PER_DEGREE_A);
  int requiredStepsB = (int)((angleB - currAngleB) * STEPS_PER_DEGREE_B);
  int totalStepsRequired = max(abs(requiredStepsA), abs(requiredStepsB));
  
  // Set rotation direction
  digitalWrite(DIR_X_PIN, requiredStepsA < 0);
  digitalWrite(DIR_Y_PIN, requiredStepsB < 0);

  for (int i = 0; i < totalStepsRequired; i++) {
    // Send pulse of HIGH command
    if (i < abs(requiredStepsA)) {
      digitalWrite(STEP_X_PIN, HIGH);
    }
    if (i < abs(requiredStepsB)) {
      digitalWrite(STEP_Y_PIN, HIGH);
    }
    delayMicroseconds(STEPPER_PULSE_WIDTH);

    // Send LOW command
    if (i < abs(requiredStepsA)) {
      digitalWrite(STEP_X_PIN, LOW);
    }
    if (i < abs(requiredStepsB)) {
      digitalWrite(STEP_Y_PIN, LOW);
    }
    delayMicroseconds(STEPPER_PERIOD - STEPPER_PULSE_WIDTH);
  }

  currAngleA = angleA;
  currAngleB = angleB;
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
}

// This defines the command indices
void (*actions[])(uint8_t*) = {
  moveStepper,
  moveWrist
};

void setup() {
  Serial.begin(9600);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  pinMode(STEP_X_PIN, OUTPUT);
  pinMode(STEP_Y_PIN, OUTPUT);
  pinMode(DIR_X_PIN, OUTPUT);
  pinMode(DIR_Y_PIN, OUTPUT);
  servoDriver.begin();
  servoDriver.setPWMFreq(60);
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
