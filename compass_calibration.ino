#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define LED_PIN 13 // LED pin for status indication

// Create a compass object
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Calibration variables
float minX = 0, maxX = 0;
float minY = 0, maxY = 0;
float minZ = 0, maxZ = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
    while (1);
  }
  
  Serial.println("HMC5883 Magnetometer Calibration");
  Serial.println("Move the compass in all directions to calibrate...");
  delay(2000); // Give some time to start the serial monitor
}

void loop() {
  sensors_event_t event;
  mag.getEvent(&event);

  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;

  // Update min and max values for each axis
  minX = min(minX, x);
  maxX = max(maxX, x);
  minY = min(minY, y);
  maxY = max(maxY, y);
  minZ = min(minZ, z);
  maxZ = max(maxZ, z);

  // Indicate calibration in progress
  digitalWrite(LED_PIN, HIGH);

  // Print raw magnetometer data
  Serial.print("Raw Data (X, Y, Z): ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);
  
  delay(100); // Small delay to stabilize readings

  // Indicate end of calibration
  digitalWrite(LED_PIN, LOW);
}

void showCalibrationValues() {
  Serial.println("Calibration Complete!");
  Serial.println("Minimum Values:");
  Serial.print("X: "); Serial.println(minX);
  Serial.print("Y: "); Serial.println(minY);
  Serial.print("Z: "); Serial.println(minZ);
  Serial.println("Maximum Values:");
  Serial.print("X: "); Serial.println(maxX);
  Serial.print("Y: "); Serial.println(maxY);
  Serial.print("Z: "); Serial.println(maxZ);
}
