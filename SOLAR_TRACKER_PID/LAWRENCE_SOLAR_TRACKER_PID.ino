#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "data.hpp"

const int   LDR_LEFT   = A2;      // Left LDR pin
const int   LDR_RIGHT  = A0;      // Right LDR pin
const int   SERVO_PIN  = 9;       // Servo PWM pin
const int   DEAD_BAND  = 2;       // Error threshold for servo adjustment (scaled)
const float DIVIDER_K  = 3.50;    // Voltage divider constant for A7

float dt = 0;                     // Time step for PID (seconds)
float control_variable;            // PID output
int   servoPos = 90;              // Initial servo position (centered, 0-180°)
unsigned long prevTime = 0;       // Previous loop time (microseconds)
unsigned long currTime = 0;       // Current loop time (microseconds)
float error;                      // Scaled error for PID

PIDController Lorenzo(0.1, 2.0, 0.3, -90, 90); // PID: Kp=0.05, Ki=0, Kd=0, output limits ±50
Servo sg90;                       // Servo object
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD object
void setup() {
  Serial.begin(9600);

  sg90.attach(SERVO_PIN);
  sg90.write(servoPos); // Start at 90° (centered)

  pinMode(LDR_LEFT, INPUT);
  pinMode(LDR_RIGHT, INPUT);
  pinMode(A7, INPUT);

  lcd.init();
  lcd.backlight();

  delay(2000); // Let hardware stabilize
  prevTime = micros();
}

void loop() {
  currTime = micros();
  dt = (currTime - prevTime) / 1.0e6; // Calculate dt in seconds

  // Read LDRs
  int left = readLDR(LDR_LEFT);  // 0–1023
  int right = readLDR(LDR_RIGHT); // 0–1023
  Serial.print("Left Raw: ");
  Serial.print(left);
  Serial.print(" Right Raw: ");
  Serial.println(right);  
  // Scale error to -100 to 100 for PID tuning
  error = ((right - left) / 1023.0) * 100.0; // Signed difference, scaled

  // Debug output
  // Serial.print("Left: ");
  // Serial.print(left);
  // Serial.print(" Right: ");
  // Serial.print(right);
  // Serial.print(" Scaled Error: ");
  // Serial.println(error);

  // Check if both LDRs are in low light (<50)
  if (left < 50 && right < 50) {
    servoPos = 90; // Center servo
    sg90.write(servoPos);
    delay(100);
  } else if (abs(error) > DEAD_BAND) { // Only adjust if error exceeds deadband
    control_variable = Lorenzo.compute(error, dt); // Compute PID output
    servoPos = 90 + control_variable; // Adjust from center
    servoPos = constrain(servoPos, 30, 180); // Constrain to servo range
    sg90.write(servoPos);
    delay(100);
    // Debug servo position
    // Serial.print("Servo Position: ");
    // Serial.print(servoPos);
    // Serial.print(" Control Variable: ");
    Serial.println("Servo Position: " + String(servoPos) + "Error :"  + String(error) + " Control Variable: " + String(control_variable));
  }else{
    Serial.println("Servo stable");
  }

  // Read and display panel voltage
  int rawA7 = analogRead(A7);
  float pinVoltage = rawA7 * (5.0 / 1023.0); // Convert to volts
  float panelVoltage = pinVoltage * DIVIDER_K; // Apply voltage divider

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("VOUT:");
  lcd.print(panelVoltage, 2);
  lcd.print("V");

  // Update timing
  prevTime = currTime;
  delay(500); // Consolidated delay for loop refresh
}