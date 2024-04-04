// Blynk credentials
#define BLYNK_TEMPLATE_ID "TMPL2RacGwd52"
#define BLYNK_TEMPLATE_NAME "Follow Me Luggage CA"
#define BLYNK_AUTH_TOKEN "E_aT698EAtWa09a2H9xLjAaR7acfksNv"

// Blynk credentials
char auth[] = "E_aT698EAtWa09a2H9xLjAaR7acfksNv"; // Use the same Auth Token for ArduinoB

#include <WiFi.h>
#include <Blynk.h>
#include <BlynkSimpleWifi.h>
#include <WifiS3.h>

// WiFi credentials
const char* ssid = "Begum";
const char* password = "Bu-300501";

// WiFi server
WiFiServer server(80);

// Target GPS coordinates (simulate or update with actual GPS data)
double targetLatitude = 0.0;
double targetLongitude = 0.0;

// LED pin for WiFi connection status
const int wifiLED = 13; // Built-in LED pin on most Arduino boards

void setup() {
  Serial.begin(9600);

  pinMode(wifiLED, OUTPUT); // Initialize the WiFi status LED pin as an output
  digitalWrite(wifiLED, LOW); // Ensure the LED is off initially

  Blynk.begin(auth, ssid, password); // Initialize Blynk

  connectWiFi();
  server.begin();
}

void loop() {
  
  Blynk.run(); // This will run the Blynk magic

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(wifiLED, LOW); // Turn off the LED if connection is lost
    connectWiFi(); // Attempt to reconnect
  } else {
    digitalWrite(wifiLED, HIGH); // Ensure the LED is on while connected
    Blynk.virtualWrite(V5, targetLatitude, targetLongitude); // Update Blynk with current location
  }

  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\n');
        if (request.indexOf("GET LATITUDE") != -1) {
          client.println(targetLatitude);
        } else if (request.indexOf("GET LONGITUDE") != -1) {
          client.println(targetLongitude);
        }
        break;
      }
    }
    delay(1);
    client.stop();
  }
}

void connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
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
}