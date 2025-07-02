#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);
AccelStepper stepperA(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperB(AccelStepper::DRIVER, 3, 6);

#define LEFT true

// Number of times of wrist movement
#define NUM_CYCLES 2

// Constants defining command and action lengths
#define CMD_LEN 11      // The total number of bytes in each full command
#define ACTION_LEN 3    // Number of possible actions (moveArm, moveEndoWrist, moveWrist)

// Servo limits and configuration
#define SERVO_FREQ 60           // Servo PWM frequency (usually 50-60Hz)
#define SERVO_MIN 125           // Minimum pulse length count for typical servo range
#define SERVO_MAX 625           // Maximum pulse length count for typical servo range

// Servo channel IDs on the PCA9685 driver board
#define SERVO_A_A_ID 0
#define SERVO_A_B_ID 1
#define SERVO_A_C_ID 2
#define SERVO_A_D_ID 3
#define SERVO_B_A_ID 5
#define SERVO_B_B_ID 6
#define SERVO_B_C_ID 7
#define SERVO_B_D_ID 8

#define SERVO_LEFT_WRIST 4
#define SERVO_RIGHT_WRIST 9

#define SERVO_LEFT_WRIST_ZERO 121.5  // deg
#define SERVO_RIGHT_WRIST_ZERO 50  // deg

// Stepper motor settings
#define EN_PIN 8                 // Enable pin for stepper drivers
#define STEPS_PER_REV 200        // Steps per revolution (depends on your stepper motor)
#define STEPPER_A_RATIO 50.0      // Gear ratio or offset for stepper A
#define STEPPER_B_RATIO 4.0      // Gear ratio or offset for stepper B
#define STEPPER_A_MAX_SPEED   800  // steps/s
#define STEPPER_A_ACCEL       8000  // steps/s^2
#define STEPPER_B_MAX_SPEED   200  // steps/s
#define STEPPER_B_ACCEL       6000  // steps/s^2

/**
 * moveArm()
 * ----------
 * This function receives four bytes in `data`:
 *   data[0] -> signA (0 or 1)
 *   data[1,2] -> angleA (interpreted in 0.01 degrees)
 *   data[3] -> signB (0 or 1)
 *   data[4,5] -> angleB (interpreted in 0.01 degrees)
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
  int angleA = (data[1] << 8) + data[2];  // 0.01 degrees
  int signB = data[3];
  int angleB = (data[4] << 8) + data[5];  // 0.01 degrees

  // If sign is 1, invert the angle (negative)
  if (signA == 1) {
    angleA *= -1;
  }
  if (signB == 1) {
    angleB *= -1;
  }

  // Left motors move in opposite direction
  if (LEFT) {
    angleA *= -1;
    angleB *= -1;
  }

  // Convert angles to steps
  long stepsA = (long)(angleA*0.01 * STEPPER_A_RATIO * STEPS_PER_REV / 360.0);
  long stepsB = (long)(angleB*0.01 * STEPPER_B_RATIO * STEPS_PER_REV / 360.0);

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
 * 
 * The input data are values from 0 to 360
 *
 * The code then maps these pot values to the correct servo pulse widths (in microseconds),
 * and further translates them into the correct PCA9685 driver counts (0-4095).
 * Finally, it sets the servos on either arm A or arm B, depending on wristIdx,
 * and sends back a 2-byte response (0xFF, 0x01).
 */
void moveEndoWrist(uint8_t* data) {
  int wristIdx = data[0];
  int phi = (data[1] << 8) + data[2];
  int theta = (data[3] << 8) + data[4];
  int angleA = (data[5] << 8) + data[6];
  const float PHI_COEFF = 0.66667;
  int angleB = (data[7] << 8) + data[8];

  int pulseA = angleA + PHI_COEFF * (float)(phi - 180);
  pulseA = map(constrain(pulseA, 100, 260), 100, 260, SERVO_MIN, SERVO_MAX);
  int pulseB = angleB + PHI_COEFF * (float)(phi - 180);
  pulseB = map(constrain(pulseB, 100, 260), 100, 260, SERVO_MIN, SERVO_MAX);
  int pulseTheta = map(theta, 18, 342, SERVO_MIN, SERVO_MAX);
  int pulsePhi = map(constrain(phi, 90, 270), 90, 270, SERVO_MIN, SERVO_MAX);

  if (wristIdx == 0) {
    servoDriver.setPWM(SERVO_A_A_ID, 0, pulseTheta); 
    servoDriver.setPWM(SERVO_A_B_ID, 0, pulseB); 
    servoDriver.setPWM(SERVO_A_C_ID, 0, pulseA); 
    servoDriver.setPWM(SERVO_A_D_ID, 0, pulsePhi);
  } else {
    servoDriver.setPWM(SERVO_B_A_ID, 0, pulseTheta); 
    servoDriver.setPWM(SERVO_B_B_ID, 0, pulseB); 
    servoDriver.setPWM(SERVO_B_C_ID, 0, pulseA); 
    servoDriver.setPWM(SERVO_B_D_ID, 0, pulsePhi);
  }

  uint8_t response[] = {0xFF, 0x01};
  Serial.write(response, sizeof(response));
}


/**
 * moveWrist()
 * ------------
 * This function receives 2 bytes in `data`:
 *   data[0] -> servoIdx (which servo channel to move; should be 4 or 9)
 *   data[1] -> angle (0..180 degrees)
 *`
 * 1) Maps the angle into the PCA9685 servo driver counts.
 * 2) Writes the PWM signal to the servo at channel 'servoIdx'.
 * 3) Sends back a response (0xFF, 0x02).
 */
void moveWrist(uint8_t* data) {
  int l_r = data[0];
  int sign = data[1]; 
  int angle = (data[2] << 8) + data[3];  // 0.01 degrees

  // If sign is 1, invert the angle (negative)
  if (sign == 1) {
    angle *= -1;
  }

  int angle_offset; 
  if (l_r == SERVO_LEFT_WRIST) {
    angle_offset = SERVO_LEFT_WRIST_ZERO;
  } else if (l_r == SERVO_RIGHT_WRIST) {
    angle_offset = SERVO_RIGHT_WRIST_ZERO;
  }

  // Map the angle to the servo pulse width (in PCA9685 counts)
  int pulse = map(angle*0.01 + angle_offset, 0, 180, SERVO_MIN, SERVO_MAX);
  servoDriver.setPWM(l_r, 0, pulse);

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

// For testing purposes -- move wrists back and forth
void test_wrists() {
  for (int i=0; i<5; i++) {
    int l_pulse = map(-15+SERVO_LEFT_WRIST_ZERO, 0, 180, SERVO_MIN, SERVO_MAX);
    int r_pulse = map(-15+SERVO_RIGHT_WRIST_ZERO, 0, 180, SERVO_MIN, SERVO_MAX);
    servoDriver.setPWM(SERVO_LEFT_WRIST, 0, l_pulse);
    servoDriver.setPWM(SERVO_RIGHT_WRIST, 0, r_pulse);
    delay(1000);
    l_pulse = map(SERVO_LEFT_WRIST_ZERO, 0, 180, SERVO_MIN, SERVO_MAX);
    r_pulse = map(SERVO_RIGHT_WRIST_ZERO, 0, 180, SERVO_MIN, SERVO_MAX);
    servoDriver.setPWM(SERVO_LEFT_WRIST, 0, l_pulse);
    servoDriver.setPWM(SERVO_RIGHT_WRIST, 0, r_pulse);
    delay(1000);
  }
  
}

void setup() {
  Serial.begin(115200);
  // Servo motor setup
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);

//  engage_couplers();
//  test_wrists();

  // Stepper enable pin
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // LOW to enable stepper drivers

  // Configure stepper speeds and accelerations
  stepperA.setMaxSpeed(STEPPER_A_MAX_SPEED);
  stepperA.setAcceleration(STEPPER_A_ACCEL);

  stepperB.setMaxSpeed(STEPPER_B_MAX_SPEED);
  stepperB.setAcceleration(STEPPER_B_ACCEL);
}

uint8_t inBytes[CMD_LEN];  // Buffer for incoming command bytes
int numBytes = 0;  // counter for number of bytes received
void loop() {
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