#include <AccelStepper.h>
#include <TMCStepper.h>
//motor 1 ground 
#define EN_PIN1    26
#define DIR_PIN1   27
#define STEP_PIN1  14
// motor 2 right arm
#define EN_PIN2    25
#define DIR_PIN2   33
#define STEP_PIN2  32
//motor 3 left arm
#define EN_PIN3    15
#define DIR_PIN3   4
#define STEP_PIN3  2

#define SERIAL_PORT Serial2
#define DRIVER_ADDRESS1 0b00
#define DRIVER_ADDRESS2 0b01
#define DRIVER_ADDRESS3 0b10
#define R_SENSE 0.11f

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS3);

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(EN_PIN1, OUTPUT);
  pinMode(EN_PIN2, OUTPUT);
  pinMode(EN_PIN3, OUTPUT);
  digitalWrite(EN_PIN1, LOW);
  digitalWrite(EN_PIN2, LOW);
  digitalWrite(EN_PIN3, LOW);

  SERIAL_PORT.begin(115200);
  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(800);
  driver1.microsteps(16);

  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(800);
  driver2.microsteps(16);

  driver3.begin();
  driver3.toff(5);
  driver3.rms_current(800);
  driver3.microsteps(8);

  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(500);
}

void loop() {
  static int direction1 = 1;
  if (stepper1.distanceToGo() == 0) {
    stepper1.moveTo(direction1 * 4000);
    direction1 = -direction1;
  }
  stepper1.run();

  static int direction2 = 1;
  if (stepper2.distanceToGo() == 0) {
    stepper2.moveTo(direction2 * 4000);
    direction2 = -direction2;
  }
  stepper2.run();

  static int direction3 = 1;
  if (stepper3.distanceToGo() == 0) {
    stepper3.moveTo(direction3 * 4000);
    direction3 = -direction3;
  }
  stepper3.run();
}
