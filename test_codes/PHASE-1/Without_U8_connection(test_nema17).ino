#include <AccelStepper.h>
#include <TMCStepper.h>

// Pin Definitions
#define EN_PIN    26  // Enable pin
#define DIR_PIN   27  // Direction pin
#define STEP_PIN  14  // Step pin
#define SERIAL_PORT Serial2 // UART port for TMC2209
#define DRIVER_ADDRESS 0b00 // TMC2209 UART address

// TMC2209 Driver Configuration
#define R_SENSE 0.11f // Sense resistor value
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

// AccelStepper instance
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Serial for debugging
  Serial.begin(115200);
  while (!Serial);

  // Configure pins
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable the driver (active low)

  // Initialize UART for TMC2209
  SERIAL_PORT.begin(115200);
  driver.begin();
  driver.toff(5);              // Enable driver
  driver.rms_current(800);     // Set motor current (mA)
  driver.microsteps(16);       // Set microstepping to 1/16

  // Configure AccelStepper
  stepper.setMaxSpeed(4000);   // Maximum speed
  stepper.setAcceleration(500); // Acceleration
}

void loop() {
  // Example: Move motor 1000 steps forward and backward
  static int direction = 1;
  if (stepper.distanceToGo() == 0) {
    stepper.moveTo(direction * 4000); // Set new target
    direction = -direction;          // Change direction
  }
  stepper.run();
}