#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
AccelStepper stepperA(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperB(AccelStepper::DRIVER, 3, 6);

// Constants defining command and action lengths
#define CMD_LEN 11      // The total number of bytes in each full command
#define ACTION_LEN 3    // Number of possible actions (moveArm, moveEndoWrist, moveWrist)

// Servo limits and configuration
#define SERVO_FREQ 60           // Servo PWM frequency (usually 50-60Hz)
#define SERVO_MIN 125           // Minimum pulse length count for typical servo range
#define SERVO_MAX 625           // Maximum pulse length count for typical servo range
#define ENDO_WRIST_MIN 650      // Minimum pulse for the endo-wrist mechanism (in microseconds)
#define ENDO_WRIST_MAX 2350     // Maximum pulse for the endo-wrist mechanism (in microseconds)

// Servo channel IDs on the PCA9685 driver board
// SERVO WIRING DIAGRAM
#define SERVO_A_A_ID 0
#define SERVO_A_B_ID 1
#define SERVO_A_C_ID 2
#define SERVO_A_D_ID 3
#define SERVO_B_A_ID 5
#define SERVO_B_B_ID 6
#define SERVO_B_C_ID 7
#define SERVO_B_D_ID 8

#define SERVO_LEFT_WRIST_ZERO 121.5  // deg
#define SERVO_RIGHT_WRIST_ZERO 50  // deg

// Stepper motor settings
#define EN_PIN 8                 // Enable pin for stepper drivers
#define STEPS_PER_REV 200        // Steps per revolution (depends on your stepper motor)
#define STEPPER_A_RATIO 1.0      // Gear ratio or offset for stepper A
#define STEPPER_B_RATIO 1.0      // Gear ratio or offset for stepper B
#define STEPPER_MAX_SPEED   200  // steps/s
#define STEPPER_ACCEL       100  // steps/s^2

/**
 * moveArm()
 * ----------
 * This function receives four bytes in `data`:
 *   data[0] -> signA (0 or 1)
 *   data[1] -> angleA (0-255, interpreted in degrees)
 *   data[2] -> signB (0 or 1)
 *   data[3] -> angleB (0-255, interpreted in degrees)
 * 
 * signA/signB indicate if the angle is positive or negative.
 * 
 * 1) Convert sign and raw angle into a signed angle for each stepper.
 * 2) Calculate the target step position based on STEPS_PER_REV and angle.
 * 3) Move both steppers simultaneously to the target position.
 * 4) Send back a response (0xFF, 0x00).
 */
void moveArm(uint8_t* data) {
  int signA = data[0];
  int angleA = data[1];
  int signB = data[2];
  int angleB = data[3];

  // If sign is 1, invert the angle (negative)
  if (signA == 1) {
    angleA *= -1;
  }
  if (signB == 1) {
    angleB *= -1;
  }

  // Convert angles to steps
  long stepsA = (long)(angleA * STEPPER_A_RATIO * STEPS_PER_REV / 360.0);
  long stepsB = (long)(angleB * STEPPER_B_RATIO * STEPS_PER_REV / 360.0);

  // Instruct each stepper to move to its target (absolute position)
  stepperA.moveTo(stepsA);
  stepperB.moveTo(stepsB);

  // Run both steppers until they reach their targets
  while ((stepperA.distanceToGo() != 0) || (stepperB.distanceToGo() != 0)) {
    stepperA.run();
    stepperB.run();
  }

  uint8_t response[] = {0xFF, 0x00};
  Serial.write(response, sizeof(response));
}

/**
 * moveEndoWrist()
 * ----------------
 * This function receives 9 bytes in `data`:
 *   data[0]  -> armIdx (which arm: 0 for arm A, 1 for arm B)
 *   data[1]  -> high byte of potA
 *   data[2]  -> low byte of potA
 *   data[3]  -> high byte of potB
 *   data[4]  -> low byte of potB
 *   data[5]  -> high byte of potC
 *   data[6]  -> low byte of potC
 *   data[7]  -> high byte of potD
 *   data[8]  -> low byte of potD
 *
 * Each potX is a 10-bit reading (0-1023) that corresponds to a desired
 * position for the endo-wrist servos:
 *   potA -> abduction servo
 *   potB, potC -> used to compute flexion for two servos plus a jaw mechanism
 *   potD -> shaft rotation servo
 *
 * The code then maps these pot values to the correct servo pulse widths (in microseconds),
 * and further translates them into the correct PCA9685 driver counts (0-4095).
 * Finally, it sets the servos on either arm A or arm B, depending on armIdx,
 * and sends back a 2-byte response (0xFF, 0x01).
 */
void moveEndoWrist(uint8_t* data) {
  int armIdx = data[0];
  int potA = (data[1] << 8) + data[2];
  int potB = (data[3] << 8) + data[4];
  int potC = (data[5] << 8) + data[6];
  int potD = (data[7] << 8) + data[8];

  // Variables to store converted pulse widths
  int pWideA, pWideB, pWideC, pWideD;     // Pulse_Wide  
  int pWidthA, pWidthB, pWidthC, pWidthD; // Pulse_Width
     
  int halfFlexion, remappedPotC;
  int F1, F2; // Flexion Variables
  
  // A -> ABDUCTION
  // Map potA from [0..1023] into endo-wrist microseconds [ENDO_WRIST_MIN..ENDO_WRIST_MAX],
  // then convert that into the PCA9685 counts [0..4095].
  pWideA = map(potA, 0, 1023, ENDO_WRIST_MIN, ENDO_WRIST_MAX);
  pWidthA = int(float(pWideA) / 1000000 * SERVO_FREQ * 4096);

  // JAWS
  // If potC > 511, that means "open jaw," so we remap part of its range.
  // Otherwise, we default to closed or no movement.
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
  pWideB = map(F1, 0, 1023, ENDO_WRIST_MIN, ENDO_WRIST_MAX);
  pWidthB = int(float(pWideB) / 1000000 * SERVO_FREQ * 4096);
  
  // F2 C -> FLEXION_2
  F2 = potB + ((potA-511)/2) - remappedPotC; // Jaw and abduction compensation
  F2 = constrain(F2, 0, 1023);
  pWideC = map(F2, 0, 1023, ENDO_WRIST_MIN, ENDO_WRIST_MAX);
  pWidthC = int(float(pWideC) / 1000000 * SERVO_FREQ * 4096);  

  // D -> SHAFT ROTATION
  pWideD = map(potD, 0, 1023, ENDO_WRIST_MIN, ENDO_WRIST_MAX);
  pWidthD = int(float(pWideD) / 1000000 * SERVO_FREQ * 4096);

  // Write the computed PWM values to the correct set of servo channels,
  // depending on which arm (A or B) we're controlling.
  if (armIdx == 0) {
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

  // Pause to allow servos to move
  delay(500);

  uint8_t response[] = {0xFF, 0x01};
  Serial.write(response, sizeof(response));
}

/**
 * moveWrist()
 * ------------
 * This function receives 2 bytes in `data`:
 *   data[0] -> servoIdx (which servo channel to move; should be 4 or 9)
 *   data[1] -> angle (0..180 degrees)
 *
 * 1) Maps the angle into the PCA9685 servo driver counts.
 * 2) Writes the PWM signal to the servo at channel 'servoIdx'.
 * 3) Sends back a response (0xFF, 0x02).
 */
void moveWrist(uint8_t* data) {
  int servoIdx = data[0];
  int angle = data[1];

  if (servo_idx == 4) {
    int angle_offset = SERVO_LEFT_WRIST_ZERO;
  } else if (servo_idx == 9) {
    int angle_offset = SERVO_RIGHT_WRIST_ZERO;
  }

  // Map the angle to the servo pulse width (in PCA9685 counts)
  int pulse = map(angle + angle_offset, 0, 180, SERVO_MIN, SERVO_MAX);
  servoDriver.setPWM(servoIdx, 0, pulse);
  delay(500);

  uint8_t response[] = {0xFF, 0x02};
  Serial.write(response, sizeof(response));
}

// An array of function pointers matching the "actions" that can be requested:
//   0 -> moveArm
//   1 -> moveEndoWrist
//   2 -> moveWrist
void (*actions[])(uint8_t*) = {
  moveArm,
  moveEndoWrist,
  moveWrist
};

// Helper function to move all endowrist servos back and forth to fully seat the couplers
void engage_couplers() {
  int NUM_CYCLES = 12;  // Reduce cycles for smoother motion
  int CYCLE_MIN = 60;
  int CYCLE_MAX = 120;
  int STEP_DELAY = 50;  // Delay between steps for slower motion
  int STEP_SIZE = 2;    // Smaller step size for smoother motion

  for (int i = 0; i < NUM_CYCLES; i++) {
    // Move from CYCLE_MIN to CYCLE_MAX in steps
    for (int angle = CYCLE_MIN; angle <= CYCLE_MAX; angle += STEP_SIZE) {
      int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
      servoDriver.setPWM(SERVO_A_A_ID, 0, pulse);
      servoDriver.setPWM(SERVO_A_B_ID, 0, pulse);
      servoDriver.setPWM(SERVO_A_C_ID, 0, pulse);
      servoDriver.setPWM(SERVO_A_D_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_A_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_B_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_C_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_D_ID, 0, pulse);
      delay(STEP_DELAY);
    }

    // Move from CYCLE_MAX to CYCLE_MIN in steps
    for (int angle = CYCLE_MAX; angle >= CYCLE_MIN; angle -= STEP_SIZE) {
      int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
      servoDriver.setPWM(SERVO_A_A_ID, 0, pulse);
      servoDriver.setPWM(SERVO_A_B_ID, 0, pulse);
      servoDriver.setPWM(SERVO_A_C_ID, 0, pulse);
      servoDriver.setPWM(SERVO_A_D_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_A_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_B_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_C_ID, 0, pulse);
      servoDriver.setPWM(SERVO_B_D_ID, 0, pulse);
      delay(STEP_DELAY);
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Servo motor setup
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);

  engage_couplers();
 
  // Set enable pins for steppers
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); 

  // Stepper enable pin
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // LOW to enable stepper drivers

  // Configure stepper speeds and accelerations
  stepperA.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperA.setAcceleration(STEPPER_ACCEL);

  stepperB.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperB.setAcceleration(STEPPER_ACCEL);
}

uint8_t inBytes[CMD_LEN];  // Buffer for incoming command bytes
int numBytes = 0;  // counter for number of bytes received
void loop() {
  int pulse = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
  servoDriver.setPWM(SERVO_A_A_ID, 0, pulse);
  servoDriver.setPWM(SERVO_A_B_ID, 0, pulse);
  servoDriver.setPWM(SERVO_A_C_ID, 0, pulse);
  servoDriver.setPWM(SERVO_A_D_ID, 0, pulse);
  servoDriver.setPWM(SERVO_B_A_ID, 0, pulse);
  servoDriver.setPWM(SERVO_B_B_ID, 0, pulse);
  servoDriver.setPWM(SERVO_B_C_ID, 0, pulse);
  servoDriver.setPWM(SERVO_B_D_ID, 0, pulse);
  delay(1000);

  Serial.println("looping...");
  delay(500);
  if (Serial.available() > 0) {
    // Shift the existing buffer contents over by 1
    // (This effectively keeps the newest byte in inBytes[CMD_LEN - 1])
    for (int i = 0; i < CMD_LEN - 1; i++) {
      inBytes[i] = inBytes[i + 1];
    }
    // Read the newest byte from the serial buffer and store in end of inBytes
    inBytes[CMD_LEN - 1] = Serial.read();
    numBytes++;

    // If the oldest byte in the buffer is 0xFF and we have at least CMD_LEN bytes,
    // we interpret that as a complete command.
    if (inBytes[0] == 0xFF && numBytes >= CMD_LEN) {
      numBytes = 0;  // Reset byte count

      // The second byte is action index: 0, 1, or 2
      uint8_t actionIndex = inBytes[1];

      // If actionIndex is valid, call the corresponding function
      if (actionIndex >= 0 && actionIndex < ACTION_LEN) {

        // Pass a pointer to the rest of the command (the bytes after actionIndex)
        actions[actionIndex](&inBytes[2]); // Pass the remaining command bytes
      }
    }
  }
}
