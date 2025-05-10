#include <TMCStepper.h>
#include <math.h>

#define EN_PIN1     26  
#define STEP_PIN1   14  
#define DIR_PIN1    27  

#define EN_PIN2     25  
#define STEP_PIN2   33  
#define DIR_PIN2    32  

#define UART_TX     17  
#define UART_RX     16  

#define DRIVER_ADDRESS1 0b00 
#define DRIVER_ADDRESS2 0b01 
#define R_SENSE 0.11f       

#define MIN_SPEED  800  
#define MAX_SPEED  100  

HardwareSerial SERIAL_PORT(1); 

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);

float link1 = 0.2592;  // Link 1 length (m)
float link2 = 0.25347; // Link 2 length (m)

#define MICROSTEPS 64
#define STEPS_PER_REV 200
#define DEG_TO_STEP (STEPS_PER_REV * MICROSTEPS / 360.0)

void setup() {
    Serial.begin(9600);
    while (!Serial);

    pinMode(EN_PIN1, OUTPUT);
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    digitalWrite(EN_PIN1, LOW); // Enable Driver 1

    pinMode(EN_PIN2, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);
    digitalWrite(EN_PIN2, LOW); // Enable Driver 2

    SERIAL_PORT.begin(9600, SERIAL_8N1, UART_RX, UART_TX);

    driver1.begin();
    driver1.toff(4); 
    driver1.rms_current(1800);
    driver1.microsteps(MICROSTEPS);
    driver1.en_spreadCycle(true);
    driver1.intpol(true);

    driver2.begin();
    driver2.toff(4);
    driver2.rms_current(1800);
    driver2.microsteps(MICROSTEPS);
    driver2.en_spreadCycle(true);
    driver2.intpol(true);

    Serial.println("✅ System Ready! Moving Arm in X-axis...");
}

void moveStepper(int steps, int dirPin, int stepPin) {
    digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
    steps = abs(steps);

    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(200); 
    }
}

void moveToX(float targetX) {
    float c2 = (targetX * targetX - (link1 * link1 + link2 * link2)) / (2 * link1 * link2);
    float s2 = sqrt(1 - c2 * c2);
    float theta2 = atan2(s2, c2);  

    float k1 = link1 + link2 * cos(theta2);
    float k2 = link2 * sin(theta2);
    float theta1 = atan2(0.2, targetX) - atan2(k2, k1);

    int steps1 = round(theta1 * DEG_TO_STEP);
    int steps2 = round(theta2 * DEG_TO_STEP);

    Serial.print("📍 Target X: "); Serial.print(targetX);
    Serial.print(" | θ1: "); Serial.print(theta1 * 180 / M_PI);
    Serial.print(" | θ2: "); Serial.print(theta2 * 180 / M_PI);
    Serial.print(" | Steps1: "); Serial.print(steps1);
    Serial.print(" | Steps2: "); Serial.println(steps2);

    moveStepper(steps1, DIR_PIN1, STEP_PIN1);
    moveStepper(steps2, DIR_PIN2, STEP_PIN2);
}

void loop() {
    for (float x = 0.25; x <= 0.35; x += 0.01) {
        moveToX(x);
        delay(100);  // Wait between moves
    }

    Serial.println("🏁 Motion sequence complete. Restarting...");
    delay(2000);  // Pause before restarting
}
