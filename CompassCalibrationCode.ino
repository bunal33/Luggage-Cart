#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Variables to store maximum and minimum readings
float minX = 10000, maxX = -10000;
float minY = 10000, maxY = -10000;
float minZ = 10000, maxZ = -10000;

void setup() {
  Serial.begin(9600);
  if (!mag.begin()) {
    Serial.println("No HMC5883 detected, check your connections.");
    while (1);
  }
  Serial.println("HMC5883 detected!");
}

void loop() {
  sensors_event_t event; 
  mag.getEvent(&event);

  // Update max and min for X
  if (event.magnetic.x > maxX) maxX = event.magnetic.x;
  if (event.magnetic.x < minX) minX = event.magnetic.x;

  // Update max and min for Y
  if (event.magnetic.y > maxY) maxY = event.magnetic.y;
  if (event.magnetic.y < minY) minY = event.magnetic.y;

  // Update max and min for Z
  if (event.magnetic.z > maxZ) maxZ = event.magnetic.z;
  if (event.magnetic.z < minZ) minZ = event.magnetic.z;

  // Print the current readings
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.println(" uT");

  // Print the max/min values
  Serial.print("Max X: "); Serial.print(maxX); Serial.print(" Min X: "); Serial.println(minX);
  Serial.print("Max Y: "); Serial.print(maxY); Serial.print(" Min Y: "); Serial.println(minY);
  Serial.print("Max Z: "); Serial.print(maxZ); Serial.print(" Min Z: "); Serial.println(minZ);

  delay(500);
}
