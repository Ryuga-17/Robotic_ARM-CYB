#include <TMCStepper.h>

// Pin definitions for Motor 1
#define EN_PIN1     26  
#define STEP_PIN1   14  
#define DIR_PIN1    27  

// Pin definitions for Motor 2
#define EN_PIN2     25  
#define STEP_PIN2   33  
#define DIR_PIN2    32  

// UART pin definitions
#define UART_TX     17  
#define UART_RX     16  

#define DRIVER_ADDRESS1 0b00 // TMC2209 driver address for Motor 1
#define DRIVER_ADDRESS2 0b01 // TMC2209 driver address for Motor 2
#define R_SENSE 0.11f       // Sense resistor value

// Create UART serial interface
HardwareSerial SERIAL_PORT(1); 

// TMC2209 driver instances
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Configure pins for Motor 1
  pinMode(EN_PIN1, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  digitalWrite(EN_PIN1, LOW); // Enable Motor 1

  // Configure pins for Motor 2
  pinMode(EN_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(EN_PIN2, LOW); // Enable Motor 2

  // Initialize UART
  SERIAL_PORT.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  // Initialize the drivers
  driver1.begin();
  driver1.toff(5);           
  driver1.rms_current(900);  // Set motor current (higher = more torque)
  driver1.microsteps(16);    // Use 1/16 microstepping
  driver1.en_spreadCycle(false); 
  driver1.intpol(true);      // Enable interpolation for smoother motion

  driver2.begin();
  driver2.toff(5);           
  driver2.rms_current(900);  
  driver2.microsteps(16);
  driver2.en_spreadCycle(false);
  driver2.intpol(true);      

  // Set motor direction once (No direction changes)
  digitalWrite(DIR_PIN1, HIGH); // Clockwise
  digitalWrite(DIR_PIN2, HIGH); // Clockwise

  Serial.println("Motors running at full speed...");
}

void loop() {
  // Continuous rotation at full speed
  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  delayMicroseconds(2); // Minimum pulse width
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  delayMicroseconds(500); // Adjust for speed
}
