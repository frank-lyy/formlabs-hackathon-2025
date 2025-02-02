#define EN_PIN 8      // Stepper enable
#define STEP_X_PIN 2  // X.STEP
#define STEP_Y_PIN 3  // Y.STEP
#define DIR_X_PIN 5   // X.DIR
#define DIR_Y_PIN 6   // Y.DIR

// Stepper motor settings
#define STEPS_PER_REV 200
#define PULSE_WIDTH_MICROS 100
#define PERIOD_MICROS 5000

void moveStepper(uint8_t target, uint8_t value) {
  digitalWrite(DIR_X_PIN, HIGH);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_X_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_MICROS);
    digitalWrite(STEP_X_PIN, LOW);
    delayMicroseconds(PERIOD_MICROS - PULSE_WIDTH_MICROS);
  }
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  pinMode(STEP_X_PIN, OUTPUT);
  pinMode(STEP_Y_PIN, OUTPUT);
  pinMode(DIR_X_PIN, OUTPUT);
  pinMode(DIR_Y_PIN, OUTPUT);
}

void loop() {
  Serial.println(F("Running clockwise"));
  digitalWrite(DIR_X_PIN, HIGH);
  digitalWrite(DIR_Y_PIN, HIGH);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_X_PIN, HIGH);
    digitalWrite(STEP_Y_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_MICROS);
    digitalWrite(STEP_X_PIN, LOW);
    digitalWrite(STEP_Y_PIN, LOW);
    delayMicroseconds(PERIOD_MICROS - PULSE_WIDTH_MICROS);
  }
  delay(1000);

  Serial.println(F("Running counter-clockwise"));
  digitalWrite(DIR_X_PIN, LOW);
  digitalWrite(DIR_Y_PIN, LOW);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_X_PIN, HIGH);
    digitalWrite(STEP_Y_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_MICROS);
    digitalWrite(STEP_X_PIN, LOW);
    digitalWrite(STEP_Y_PIN, LOW);
    delayMicroseconds(PERIOD_MICROS - PULSE_WIDTH_MICROS);
  }
  delay(1000);
}
