// Blynk credentials
#define BLYNK_TEMPLATE_ID "TMPL2RacGwd52"
#define BLYNK_TEMPLATE_NAME "Follow Me Luggage CA"
#define BLYNK_AUTH_TOKEN "E_aT698EAtWa09a2H9xLjAaR7acfksNv"

#include <WiFi.h>
#include <WiFiClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Blynk.h>
#include <BlynkSimpleWifi.h>
#include <WifiS3.h>
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
const int wifiLED = 13; // Built-in LED pin on most Arduino boards

const int gpsLED = 12; // GPS LED to indicate whether the GPS data received is valid

// Ultrasonic sensor pins
const int trigPin = 7; // HC-SR04 Trig pin connected to Arduino pin 7
const int echoPin = 8; // HC-SR04 Echo pin connected to Arduino pin 8

// Buzzer for object avoidance
const int buzzerPin = 9; // Buzzer connected to digital pin 9


// Constants for motor control
const int minSpeed = 100;      // Minimum speed for motors
const int maxSpeed = 255;      // Maximum speed for motors
const double maxDistance = 10; // Maximum distance to operate motors at max speed (in meters)

// Constants for GPS coordinates
const double EARTH_RADIUS = 6371000; // Earth's radius in meters

bool hasValidGPS = false; // Flag to indicate valid GPS data

// Flag to indicate system power state
bool systemPower = false; // System is off by default

BLYNK_WRITE(V1) {
  systemPower = param.asInt(); // Assign the incoming value from Blynk app to systemPower
  if (!systemPower) {
    // If the system is turned off, stop the motors
    digitalWrite(ENA, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENB, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(gpsLED, LOW); // Optionally turn off the GPS LED
  }
}

void sendGPSData() {
  if (hasValidGPS) {
    // Send ArduinoA's GPS coordinates to Blynk
    Blynk.virtualWrite(V3, gps.location.lat(), gps.location.lng());

    // Assuming you get ArduinoB's coordinates through a function call
    double arduinoBLat = getTargetLatitudeFromArduinoB();
    double arduinoBLng = getTargetLongitudeFromArduinoB();
    // Send ArduinoB's GPS coordinates to Blynk
    Blynk.virtualWrite(V4, arduinoBLat, arduinoBLng);
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // Assuming Serial1 is for GPS module communication

  pinMode(wifiLED, OUTPUT); // Initialize the WiFi status LED pin as an output
  digitalWrite(wifiLED, LOW); // Ensure the LED is off initially

  pinMode(gpsLED, OUTPUT); // Initialize the WiFi status LED pin as an output
  digitalWrite(gpsLED, LOW); // Ensure the LED is off initially

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

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
  Blynk.run(); // Runs the Blynk process
  
  // Obstacle detection with HC-SR04
  long duration, distance;
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); // Clears the trigPin condition
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2; // Calculating the distance
  
  // Send the distance to Blynk app on Virtual Pin V2
  Blynk.virtualWrite(V2, distance);

  // Check distance for buzzer activation
  if (distance > 0 && distance < 40) {
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  } else {
    digitalWrite(buzzerPin, LOW); // Turn off buzzer
  }

  // Checking if distance is within obstacle avoidance threshold
  if (distance < 30 && distance > 0) {
    // Stop the motors if an obstacle is detected within 30 cm
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
  } else {
    // Resume operations if the obstacle is cleared, but still check for system power status
    if (systemPower) {
      // GPS data reading and handling
      while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
          if (gps.location.isValid()) {
            hasValidGPS = true;
            digitalWrite(gpsLED, HIGH); // Turn on GPS LED if data is valid
            sendGPSData(); // Send GPS data to Blynk
            
            Serial.println("Valid GPS Data Received:");
            Serial.print("Latitude: ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            
            double targetLat = getTargetLatitudeFromArduinoB();
            double targetLon = getTargetLongitudeFromArduinoB();
            double currentLat = gps.location.lat();
            double currentLon = gps.location.lng();
            
            sensors_event_t event;
            mag.getEvent(&event);
            float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
            
            float bearing = TinyGPSPlus::courseTo(currentLat, currentLon, targetLat, targetLon);
            
            adjustDirection(heading, bearing);
            moveTowards(targetLat, targetLon, currentLat, currentLon);
            
          } else {
            Serial.println("Invalid GPS Data Received");
            digitalWrite(gpsLED, LOW); // Turn off GPS LED if data is invalid
          }
        }
      }
    }
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
    while (client.available() == 0);
    if (client.available() > 0) {
      latitude = client.parseFloat();
      Serial.print("Longitude received from ArduinoB: ");
      Serial.println(latitude, 6);
    }
    client.stop();
  }
  return latitude;
}

double getTargetLongitudeFromArduinoB() {
  double longitude = 0;
  if (client.connect(serverAddress, serverPort)) {
    client.println("GET LONGITUDE");
    Serial.println("Requesting logitude from ArduinoB...");
    while (client.available() == 0);
    if (client.available() > 0) {
      longitude = client.parseFloat();
      Serial.print("Longitude received from ArduinoB: ");
      Serial.println(longitude, 6);
    }
    client.stop();
  }
  return longitude;
}

void adjustDirection(float heading, float targetBearing) {
  float angleDiff = targetBearing - heading;
  if (angleDiff < 0) {
    angleDiff += 360;
  }
  if (angleDiff > 180) {
    // Turn left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Turn right
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void moveTowards(double targetLat, double targetLon, double currentLat, double currentLon) {
  double distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
  if (distance > 1) { // More than 1 meter away
    // Move forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Stop if within 1 meter of the target
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
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