#include <PIDSimple.h>

// Define Pins
const int motorSpeedPinA_Wheels = 8; // PWM for wheels motor A (direction)
const int motorSpeedPinB_Wheels = 7; // PWM for wheels motor B (direction)
const int motorSpeedPinA_Steering = 6; // PWM for steering motor A (direction)
const int motorSpeedPinB_Steering = 5; // PWM for steering motor B (direction)
const int steeringPotPin = A0; // Potentiometer pin

// PID Parameters - Wheels
PIDSimple wheelsSpeedPID_Left(1, 0.1, 0.05); // Adjust Kp, Ki, Kd for left wheel
PIDSimple wheelsSpeedPID_Right(1, 0.1, 0.05); // Adjust Kp, Ki, Kd for right wheel

// PID Parameters - Steering Angle
PIDSimple steeringAnglePID(2, 0.05, 0.01); // Adjust Kp, Ki, Kd for steering

// Target Values
float targetSpeed = 50; // Adjust desired speed
float targetAngle = 0; // Center position for steering

// Variables
float currentAngle = 0; // Potentiometer reading converted to angle
float leftWheelSpeed = 0; // Wheel speed sensor reading (left)
float rightWheelSpeed = 0; // Wheel speed sensor reading (right)
int motorPowerA_Wheels, motorPowerB_Wheels; // PWM duty cycle for wheels motor
int motorPowerA_Steering, motorPowerB_Steering; // PWM duty cycle for steering motor

void setup() {
  Serial.begin(9600); // For debugging

  // Set motor driver pins as outputs
  pinMode(motorSpeedPinA_Wheels, OUTPUT);
  pinMode(motorSpeedPinB_Wheels, OUTPUT);
  pinMode(motorSpeedPinA_Steering, OUTPUT);
  pinMode(motorSpeedPinB_Steering, OUTPUT);

  // Set potentiometer pin as input
  pinMode(steeringPotPin, INPUT);

  // Initialize PID controllers
  wheelsSpeedPID_Left.setSetPoint(targetSpeed);
  wheelsSpeedPID_Right.setSetPoint(targetSpeed);
  steeringAnglePID.setSetPoint(targetAngle);
}

void loop() {
  // Read potentiometer value and convert to angle
  currentAngle = map(analogRead(steeringPotPin), 0, 1023, -87.5, 87.5); // Adjust range based on your potentiometer

  // Get wheel speed readings (optional)
  leftWheelSpeed = ...; // Implement your left wheel speed sensor calculation
  rightWheelSpeed = ...; // Implement your right wheel speed sensor calculation

  // Update PID controllers
  motorPowerA_Wheels = abs(wheelsSpeedPID_Left.compute(targetSpeed, leftWheelSpeed)) * 255;
  motorPowerB_Wheels = 255 - motorPowerA_Wheels;
  motorPowerA_Steering = abs(steeringAnglePID.compute(targetAngle, currentAngle)) * 255;
  motorPowerB_Steering = 255 - motorPowerA_Steering;

  // Apply motor power with PWM and direction control
  if (steeringCorrection > 0) {
    analogWrite(motorSpeedPinA_Wheels, motorPowerA_Wheels);
    analogWrite(motorSpeedPinB_Wheels, motorPowerB_Wheels);
    analogWrite(motorSpeedPinA_Steering, motorPowerA_Steering);
    analogWrite(motorSpeedPinB_Steering, 0); // Left motor forward, right motor backward
  } else if (steeringCorrection < 0) {
    analogWrite(motorSpeedPinA_Wheels, motorPowerA_Wheels);
    analogWrite(motorSpeedPinB_Wheels, motorPowerB_Wheels);
    analogWrite(motorSpeedPinA_Steering, 0);
    analogWrite(motorSpeedPinB_Steering, motorPowerB_Steering); // Right motor forward, left motor backward
  } else { // Center position
  analogWrite(motorSpeedPinA_Wheels, motorPowerA_Wheels);
    analogWrite(motorSpeedPinB_Wheels, motorPowerB_Wheels);
    analogWrite(motorSpeedPinA_Steering, 0);
    analogWrite(motorSpeedPinB_Steering, 0); // Both steering motors off
  }
}
