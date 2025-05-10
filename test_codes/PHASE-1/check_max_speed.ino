#include <AccelStepper.h>
#include <TMCStepper.h>

// Motor 1 left arm 
#define EN_PIN1    26
#define DIR_PIN1   27
#define STEP_PIN1  14
// Motor 2 right arm
#define EN_PIN2    25
#define DIR_PIN2   33
#define STEP_PIN2  32

#define SERIAL_PORT Serial1
#define DRIVER_ADDRESS1 0b00
#define DRIVER_ADDRESS2 0b01
#define R_SENSE 0.11f

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

// Variables to set RPM for each motor
float motor1RPM = 40; // Default RPM for Motor 1
float motor2RPM = 140; // Default RPM for Motor 2

// Steps per revolution for the stepper motor
const int stepsPerRevolution = 200; // Adjust as per your stepper motor

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(EN_PIN1, OUTPUT);
  pinMode(EN_PIN2, OUTPUT);
  digitalWrite(EN_PIN1, LOW);
  digitalWrite(EN_PIN2, LOW);

  SERIAL_PORT.begin(115200);
  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(800);
  driver1.microsteps(16);

  SERIAL_PORT.begin(115200);
  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(800);
  driver2.microsteps(16);

  updateMotorSpeeds();
  stepper1.setAcceleration(500);
  stepper2.setAcceleration(500);
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
}

// Function to calculate and update the motor speeds based on RPM
void updateMotorSpeeds() {
  float motor1MaxSpeed = (motor1RPM * stepsPerRevolution) / 60.0;
  float motor2MaxSpeed = (motor2RPM * stepsPerRevolution) / 60.0;

  stepper1.setMaxSpeed(motor1MaxSpeed);
  stepper2.setMaxSpeed(motor2MaxSpeed);
}

// Function to update RPM dynamically (Call this when you want to change RPM)
void setMotorRPM(int motor, float rpm) {
  switch (motor) {
    case 1: motor1RPM = rpm; break;
    case 2: motor2RPM = rpm; break;
    default: return; // Invalid motor
  }
  updateMotorSpeeds(); // Update speeds after changing RPM
}
