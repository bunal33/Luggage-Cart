// cart
#include <WiFi.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// WiFi credentials
const char* ssid = "Begum";
const char* password = "Bu-300501";

// WiFi client
WiFiClient client;

// Server address and port
const char* serverAddress = "172.20.10.9"; // Use the IP address of ArduinoB
const int serverPort = 80;                   // TCP port

// Calibration Data for Compass
const float offsetX = (-31.09 + 25.27) / 2;
const float offsetY = (-49.91 + 11.36) / 2;
const float offsetZ = (-26.02 + 48.37) / 2;

// Scaling factors
const float avgRadius = ((25.27 - offsetX) + (11.36 - offsetY) + (48.37 - offsetZ)) / 3;
const float scaleX = avgRadius / (25.27 - offsetX);
const float scaleY = avgRadius / (11.36 - offsetY);
const float scaleZ = avgRadius / (48.37 - offsetZ);

// GPS and compass objects
TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Motor control pins for L298N H-bridge
const int ENA = 3;   // Enable pin for Motor A
const int IN1 = 5;   // Input 1 pin for Motor A
const int IN2 = 6;   // Input 2 pin for Motor A
const int ENB = 9;   // Enable pin for Motor B
const int IN3 = 10;  // Input 1 pin for Motor B
const int IN4 = 11;  // Input 2 pin for Motor B

// LED pin for WiFi connection status
const int wifiLED = 13; // Built-in LED pin on most Arduino boards (red)

const int gpsLED = 12; // GPS LED to indicate whether the GPS data received is valid (green)

// Define pins for ultrasonic sensors
const int trigPin1 = 2;
const int echoPin1 = 4;
const int trigPin2 = 7;
const int echoPin2 = 8;

// Constants for motor control
const int minSpeed = 100;      // Minimum speed for motors
const int maxSpeed = 255;      // Maximum speed for motors
const double maxDistance = 10; // Maximum distance to operate motors at max speed (in meters)

// Constants for GPS coordinates
const double EARTH_RADIUS = 6371000; // Earth's radius in meters

bool hasValidGPS = false; // Flag to indicate valid GPS data

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // Assuming Serial1 is for GPS module communication

  pinMode(wifiLED, OUTPUT); // Initialize the WiFi status LED pin as an output
  digitalWrite(wifiLED, LOW); // Ensure the LED is off initially

  pinMode(gpsLED, OUTPUT); // Initialize the WiFi status LED pin as an output
  digitalWrite(gpsLED, LOW); // Ensure the LED is off initially

  // Initialize ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  connectWiFi();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Explicitly stop the motors
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  if (!mag.begin()) {
    Serial.println("Failed to initialize compass!");
    while (1);
  }
}


void loop() {
    // Process GPS data
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        if (gps.location.isValid()) {
          hasValidGPS = true;
          digitalWrite(gpsLED, HIGH);
          Serial.println("Valid GPS Data Received:");
          Serial.print("Latitude of ArduinoA: ");
          Serial.println(gps.location.lat(), 6);
          Serial.print("Longitude of ArduinoA: ");
          Serial.println(gps.location.lng(), 6);
        } else {
          Serial.println("Invalid GPS Data Received");
          digitalWrite(gpsLED, LOW);
          hasValidGPS = false;
        }
      }
    }

    // Ensure this is outside the GPS data processing loop to not delay obstacle checks
    if (hasValidGPS) {
      // Obstacle detection
      long distance1 = readDistance(trigPin1, echoPin1);
      long distance2 = readDistance(trigPin2, echoPin2);

      Serial.print("Distance1: ");
      Serial.print(distance1);
      Serial.println(" cm");
      Serial.print("Distance2: ");
      Serial.print(distance2);
      Serial.println(" cm");

      if (distance1 < 30 || distance2 < 30) {
        // Obstacle detected, stop motors
        Serial.println("Obstacle detected, stopping motors");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
      } else {
        // No obstacle, proceed with following logic
        double targetLat = getTargetLatitudeFromArduinoB();
        double targetLon = getTargetLongitudeFromArduinoB();
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();

        // Calibrated compass readings
        sensors_event_t event;
        mag.getEvent(&event);
        float calibratedX = (event.magnetic.x - offsetX) * scaleX;
        float calibratedY = (event.magnetic.y - offsetY) * scaleY;
        float calibratedZ = (event.magnetic.z - offsetZ) * scaleZ;

        // Debugging compass calibration values
        Serial.print("Raw X: ");
        Serial.print(event.magnetic.x);
        Serial.print(" Calibrated X: ");
        Serial.println((event.magnetic.x - offsetX) * scaleX);

        Serial.print("Raw Y: ");
        Serial.print(event.magnetic.y);
        Serial.print(" Calibrated Y: ");
        Serial.println((event.magnetic.y - offsetY) * scaleY);

        Serial.print("Raw Z: ");
        Serial.print(event.magnetic.z);
        Serial.print(" Calibrated Z: ");
        Serial.println((event.magnetic.z - offsetZ) * scaleZ);

        float heading = atan2(calibratedY, calibratedX) * 180 / PI;
        if (heading < 0) {
          heading += 360;  // Normalize to 0-360
        }

        float bearing = TinyGPSPlus::courseTo(currentLat, currentLon, targetLat, targetLon);
        adjustDirection(heading, bearing);
        moveTowards(targetLat, targetLon, currentLat, currentLon);
      }
    } else {
      // If no valid GPS, ensure motors are stopped to handle the "lost" scenario
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      Serial.println("No valid GPS. Motors stopped.");
    }
}



void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(wifiLED, HIGH); // Turn on the LED to indicate a successful WiFi connection

}

double getTargetLatitudeFromArduinoB() {
  double latitude = 0;
  if (client.connect(serverAddress, serverPort)) {
    client.println("GET LATITUDE");
    Serial.println("Requesting latitude from ArduinoB...");
    while (client.available() == 0) {
      delay(1); // Small delay to wait for data
    }
    String latitudeString = client.readStringUntil('\n');
    latitude = latitudeString.toDouble();
    client.stop();
  }
  Serial.print("Latitude received from ArduinoB: ");
  Serial.println(latitude, 6);
  return latitude;
}


double getTargetLongitudeFromArduinoB() {
  double longitude = 0;
  if (client.connect(serverAddress, serverPort)) {
    client.println("GET LONGITUDE");
    Serial.println("Requesting longitude from ArduinoB...");
    while (client.available() == 0) {
      delay(1); // Small delay to wait for data
    }
    String longitudeString = client.readStringUntil('\n');
    longitude = longitudeString.toDouble();
    client.stop();
  }
  Serial.print("Longitude received from ArduinoB: ");
  Serial.println(longitude, 6);
  return longitude;
}



void adjustDirection(float heading, float targetBearing) {
  // Magnetic declination adjustment
  heading += 6.13;
  if (heading > 360) heading -= 360; // Ensure the heading is within 0 to 360 degrees
  
  float angleDiff = targetBearing - heading;

  // Normalize the angle difference to between 0 and 360 degrees
  angleDiff = fmod(angleDiff + 360, 360); // More robust normalization

  Serial.print("Adjusted Heading: "); Serial.println(heading);
  Serial.print("Target Bearing: "); Serial.println(targetBearing);
  Serial.print("Angle Difference: "); Serial.println(angleDiff);

  // Define a tolerance for deciding when to move straight
  const float tolerance = 10; // Degrees within which we consider we're "close enough" to the correct bearing

  if (angleDiff > 180 + tolerance) {
    // Turn left
    Serial.println("Turning Left");
    analogWrite(ENB, minSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (angleDiff > tolerance && angleDiff <= 180 - tolerance) {
    // Turn right
    Serial.println("Turning Right");
    analogWrite(ENA,minSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Move straight
    Serial.println("Moving Straight");
    analogWrite(ENB,minSpeed);
    analogWrite(ENA, minSpeed);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}


void moveTowards(double targetLat, double targetLon, double currentLat, double currentLon) {
  double distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);

  Serial.print("Distance to Target: "); Serial.println(distance);

  if (distance > 10) { // Assuming 1 meter as the threshold to stop
    // Move forward
    Serial.println("Moving forward");
    analogWrite(ENA, minSpeed);
    analogWrite(ENB, minSpeed);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Stop if within 1 meter of the target
    Serial.println("Stopping");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }
}


double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS * c;
}

double toRadians(double degrees) {
  return degrees * PI / 180.0;
}

long readDistance(int trigPin, int echoPin) {
  // Clear the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and return)
  return distance;
}