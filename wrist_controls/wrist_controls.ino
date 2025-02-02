#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Command length
#define CMD_LEN 9
 
// Servo settings
#define NUM_SERVOS      4
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define FREQUENCY       60
#define SERVOMIN        150
#define SERVOHALF       375
#define SERVOMAX        600

// Motor Outputs on PCA9685 board
#define SERVO_A_ID 4
#define SERVO_B_ID 5
#define SERVO_C_ID 6
#define SERVO_D_ID 7

const int analogInPin = A0;

void setup() {
  pwm.begin();
  Serial.begin(9600);
  pinMode(analogInPin, INPUT);
  pwm.setPWMFreq(FREQUENCY);
  //Serial.println("About to calibrate");
  //calibrate();
  //Serial.println("Calibrated!");
  delay(10);
}

void calibrate(){
  int timeDelay = 100;
  for (int i = 0; i < NUM_SERVOS; i++) { // goto 90
    pwm.setPWM(i, 0, SERVOHALF);
    delay(50);
  }
  for (int i = 0; i < NUM_SERVOS; i++) { // 45 sweep
    pwm.setPWM(i, 0, (SERVOHALF + 90)); delay(timeDelay); pwm.setPWM(i, 0, (SERVOHALF - 90)); delay(timeDelay);
    pwm.setPWM(i, 0, 262.5); delay(timeDelay); pwm.setPWM(i, 0, SERVOHALF); delay(timeDelay);
  }
  for (int i = 0; i < NUM_SERVOS; i++) { // 45 sweep
    pwm.setPWM(i, 0, 487.5); delay(timeDelay); pwm.setPWM(i, 0, SERVOHALF); delay(timeDelay);
    pwm.setPWM(i, 0, 262.5); delay(timeDelay); pwm.setPWM(i, 0, SERVOHALF); delay(timeDelay);
  }
  for (int i = 0; i < NUM_SERVOS; i++) { // goto 90
    pwm.setPWM(i, 0, SERVOHALF);
    delay(timeDelay);
  }
}

void moveWrist(int potA, int potB, int potC, int potD) {

  int pWideA, pWideB, pWideC, pWideD;     // Pulse_Wide  
  int pWidthA, pWidthB, pWidthC, pWidthD; // Pulse_Width
     
  int halfFlexion, remappedPotC;
  int F1, F2; // Flexion Variables
  
  // A -> ABDUCTION
  pWideA = map(potA, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthA = int(float(pWideA) / 1000000 * FREQUENCY * 4096);

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
  pWideB = map(F1, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthB = int(float(pWideB) / 1000000 * FREQUENCY * 4096);
  
  // F2 C -> FLEXION_2
  F2 = potB + ((potA-511)/2) - remappedPotC; // Jaw and abduction compensation
  F2 = constrain(F2, 0, 1023);
  pWideC = map(F2, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthC = int(float(pWideC) / 1000000 * FREQUENCY * 4096);  

  // D -> SHAFT ROTATION
  pWideD = map(potD, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pWidthD = int(float(pWideD) / 1000000 * FREQUENCY * 4096);
 
  pwm.setPWM(SERVO_A_ID, 0, pWidthA);
  pwm.setPWM(SERVO_B_ID, 0, pWidthB);
  pwm.setPWM(SERVO_C_ID, 0, pWidthC);
  pwm.setPWM(SERVO_D_ID, 0, pWidthD);
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
      int potA = (inBytes[1] << 8) + inBytes[2];
      int potB = (inBytes[3] << 8) + inBytes[4];
      int potC = (inBytes[5] << 8) + inBytes[6];
      int potD = (inBytes[7] << 8) + inBytes[8];

      moveWrist(potA, potB, potC, potD);
    }
  }
}
