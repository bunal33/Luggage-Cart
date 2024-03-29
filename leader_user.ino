//LEADER, W/ THE USER 


#include "./TinyGPS.h"
#include "./ArduinoBdefinitions.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>

// WiFi

//#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <WiFi.h>


// GPS
TinyGPS gps;

// Access Point (AP) settings
//char ssid[] = "MyAP";          // Name of the access point
//char pass[] = "password";      // Password for the access point
WiFiServer server(80);         // Create a server on port 80

// SoftwareSerial for GPS module
SoftwareSerial gpsSerial(10, 11); // RX, TX

char ssid[] = "Aaron's iPhone";      //  your network SSID (name)
char pass[] = "12345678";   // your network password
//int status = WL_IDLE_STATUS;       // the Wifi radio's status

WiFiUDP Udp;
IPAddress destinationIP(192, 168, 1, 2);  // IP address of the follower Arduino
unsigned int localUdpPort = 4210;  // local port to listen on

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  // Connect to WiFi network (as Access Point)
  WiFi.beginAP(ssid, pass);
  while (WiFi.status() != WL_AP_LISTENING) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Access Point ready");

  // Start the server
  Udp.begin(localUdpPort);

  // Start the server
  //server.begin();
}

void loop() {

    if (feedGPS()) {
    sendGPSData();
    delay(1000); // Send GPS data every second
    }

    WiFiClient client = server.available();  // Check for a client's connection

  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();    // Read data from the client
        Serial.write(c);           // Send data to Serial monitor
      }
    }
    client.stop();                // Close the connection
    Serial.println("Client disconnected");
  }
}

bool feedGPS() {
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read()))
      return true;
  }
  return false;
}

void sendGPSData() {
  float latitude, longitude;
  unsigned long age;
  gps.f_get_position(&latitude, &longitude, &age);

  Serial.print("Latitude: "); 
  Serial.println(latitude, 6);
  Serial.print("Longitude: "); 
  Serial.println(longitude, 6);

  // Send GPS data to follower Arduino
  sendUDP(String(latitude, 6) + "," + String(longitude, 6));
}

void sendUDP(String message) {
  Udp.beginPacket(destinationIP, localUdpPort);
  Udp.print(message);
  Udp.endPacket();
}