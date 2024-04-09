#include <WiFi.h>
#include <TinyGPS++.h>

// WiFi credentials
const char* ssid = "Begum";
const char* password = "Bu-300501";

// WiFi server
WiFiServer server(80);

TinyGPSPlus gps;

// GPS module serial connection
// Assuming the GPS module is connected to Serial1 (RX & TX pins)
// Adjust if using a different serial interface

// LED pin for WiFi connection status
const int wifiLED = 13; // Built-in LED pin on most Arduino boards (red)

const int gpsLED = 12; // GPS LED to indicate valid GPS data (green)


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // Initialize serial communication with the GPS module

  pinMode(wifiLED, OUTPUT); // Initialize the WiFi status LED pin as an output
  digitalWrite(wifiLED, LOW); // Ensure the LED is off initially

  pinMode(gpsLED, OUTPUT); // Initialize the GPS LED pin as an output
  digitalWrite(gpsLED, LOW); // Ensure the LED is off initially

  connectWiFi();
  server.begin();
}

void loop() {
  bool hasValidGPS = false; // Flag to track GPS data validity

  // Check for available GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid()) {
        hasValidGPS = true;
        // Add here any action you want to take with valid GPS data
        Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
      }
    }
  }

  // Update the GPS LED status based on data validity
  if (hasValidGPS) {
    digitalWrite(gpsLED, HIGH); // Turn on the LED if GPS data is valid
  } else {
    digitalWrite(gpsLED, LOW); // Turn off the LED if GPS data is not valid
  }

  // Check WiFi status and reconnect if necessary
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(wifiLED, LOW); // Turn off the WiFi LED if connection is lost
    connectWiFi(); // Attempt to reconnect to WiFi
  } else {
    digitalWrite(wifiLED, HIGH); // Ensure the WiFi LED is on while connected
  }

  // Handle incoming client connections and respond to requests for GPS data
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\n');
        if (request.indexOf("GET LATITUDE") != -1) {
          client.println(gps.location.isValid() ? gps.location.lat() : 0.0, 6);
        } else if (request.indexOf("GET LONGITUDE") != -1) {
          client.println(gps.location.isValid() ? gps.location.lng() : 0.0, 6);
        }
        break; // Exit the while-loop after handling the request
      }
    }
    delay(100); // Give the client time to receive the data
    client.stop(); // Close the connection
  }
  
}

void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(wifiLED, HIGH); // Turn on the LED to indicate a successful WiFi connection
}
