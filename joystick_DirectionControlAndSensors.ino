#include <Arduino.h>

// Motor A
int motorA1 = 8;
int motorA2 = 9;
int enableA = 5;

// Motor B
int motorB1 = 10;
int motorB2 = 11;
int enableB = 6;

// Joystick
int joyX = A0;
int joyY = A1;
int joyButton = 2;

// Ultrasonic Sensor 1
int trigPin1 = 3;
int echoPin1 = 4;

// Ultrasonic Sensor 2
int trigPin2 = 7;
int echoPin2 = 12;

bool motorState = false;

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(enableB, OUTPUT);

  pinMode(joyButton, INPUT_PULLUP);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void loop() {
  long distance1 = readUltrasonicDistance(trigPin1, echoPin1);
  long distance2 = readUltrasonicDistance(trigPin2, echoPin2);

  if (digitalRead(joyButton) == LOW) {
    motorState = !motorState;
    delay(200);  // Debounce delay
  }

  if (motorState) {
    if (distance1 <= 20 && distance2 <= 20) {
      // Stop if both sensors detect an object
      digitalWrite(motorA1, LOW);
      digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW);
      digitalWrite(motorB2, LOW);
    } else if (distance1 <= 20) {
      // Turn right if left sensor detects an object
      digitalWrite(motorA1, LOW);
      digitalWrite(motorA2, HIGH);
      digitalWrite(motorB1, HIGH);
      digitalWrite(motorB2, LOW);
      analogWrite(enableA, 200); // Control the speed
      analogWrite(enableB, 200);
      delay(1000); // Turn for 1 second
    } else if (distance2 <= 20) {
      // Turn left if right sensor detects an object
      digitalWrite(motorA1, HIGH);
      digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW);
      digitalWrite(motorB2, HIGH);
      analogWrite(enableA, 200);
      analogWrite(enableB, 200);
      delay(1000); // Turn for 1 second
    }
    // Resume normal operation by reading joystick
    int xValue = analogRead(joyX);
    int yValue = analogRead(joyY);
    int xMapped = map(xValue, 0, 1023, -255, 255);
    int yMapped = map(yValue, 0, 1023, -255, 255);

    if (abs(yMapped) > 50 || abs(xMapped) > 50) {
      digitalWrite(motorA1, yMapped > 0 ? HIGH : LOW);
      digitalWrite(motorA2, yMapped > 0 ? LOW : HIGH);
      digitalWrite(motorB1, xMapped > 0 ? LOW : HIGH);
      digitalWrite(motorB2, xMapped > 0 ? HIGH : LOW);
      analogWrite(enableA, abs(yMapped));
      analogWrite(enableB, abs(xMapped));
    } else {
      digitalWrite(motorA1, LOW);
      digitalWrite(motorA2, LOW);
      digitalWrite(motorB1, LOW);
      digitalWrite(motorB2, LOW);
    }
  } else {
    // Ensure motors are off when motorState is false
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
}
