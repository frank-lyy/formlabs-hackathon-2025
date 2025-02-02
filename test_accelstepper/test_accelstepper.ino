#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper1(AccelStepper::DRIVER, 2, 5);
AccelStepper stepper2(AccelStepper::DRIVER, 3, 6);

MultiStepper steppers;

void setup() {
  Serial.begin(9600);

  // Configure each stepper
  stepper1.setMaxSpeed(100);
  stepper2.setMaxSpeed(100);

  // Set enable pins
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

void loop() {
  long positions[2]; // Array of desired stepper positions
  
  positions[0] = 200;
  positions[1] = 200;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
  
  // Move to a different coordinate
  positions[0] = 0;
  positions[1] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);
}
