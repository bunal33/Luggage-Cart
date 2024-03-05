// Follow me and Obstacle Avoidance Code

// Version 1
// 3.4.2024

#define BLYNK_USE_DIRECT_CONNECT

// Define Relays
#define Relay1 12
#define Relay2 11


#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters).

// Define LED and Buzzer
#define ledPin 13
#define buzzerPin 10

// Empty Digital Pins: 1,8,9

// Library Imports
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <NewPing.h> // For Ultrasonic Sensors


// Use local version of this library
#include "./TinyGPS.h"                 
#include "./CoolerDefinitions.h"

// NewPing setup of Ultrasonic Sensor Pins and maximum Distance
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);

// GPS
TinyGPS gps;

// Lid
//Servo lidServo;
//CoolerLid lidState = CLOSED;

// Master Enable
bool enabled = false;

//WidgetTerminal terminal(V3);

// Serial components
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial nss(GPS_TX_PIN, 255);            // TXD to digital pin 6

/* Compass */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc coolerLoc;
  coolerLoc.lat = flat;
  coolerLoc.lon = flon;

  Serial.print(coolerLoc.lat, 7); Serial.print(", "); Serial.println(coolerLoc.lon, 7);

  return coolerLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

// Killswitch Hook!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
BLYNK_WRITE(V1) {
  enabled = !enabled;
  
  //Stop the wheels
  stop();
}

// GPS Streaming Hook
BLYNK_WRITE(V2) {
  GpsParam gps(param);
  
  Serial.println("Received remote GPS: ");
  
  // Print 7 decimal places for Lat
  Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

  GeoLoc phoneLoc;
  phoneLoc.lat = gps.getLat();
  phoneLoc.lon = gps.getLon();

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}

// Terminal Hook
BLYNK_WRITE(V3) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

void displayCompassDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Offset
  heading -= DECLINATION_ANGLE;
  heading -= COMPASS_OFFSET;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Map to -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}

//void setServo(int pos) {
  //lidServo.attach(SERVO_PIN);
  //lidServo.write(pos);
  //delay(2000);
  //lidServo.detach();
}

void setSpeedMotorA(int speed) {
  //digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  //digitalWrite(MOTOR_A_IN_2_PIN, HIGH);

  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay1, LOW);
  
  // set speed to 200 out of possible range 0~255
  // may not need
  // analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  // digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  // digitalWrite(MOTOR_B_IN_2_PIN, HIGH);

  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, LOW);
  
  // set speed to 200 out of possible range 0~255
  // may not need 
  // analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

// void setSpeed(int speed)
// {
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  // setSpeedMotorA(speed); // sends command to void setSpeedMotorA

  // turn on motor B
  // setSpeedMotorB(speed); // sends command to void setSpeedMotorB
//}

void stop() {
  // now turn off motors
  // digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  // digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  // digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  // digitalWrite(MOTOR_B_IN_2_PIN, LOW);

  digitalWrite(Relay2, LOW);
  digitalWrite(Relay1, LOW);
  
}

void drive(int distance, float turn) {
  // int fullSpeed = 230;
  // int stopSpeed = 0;
  int fixedSpeed =50;

  // drive to location
  int s = fullSpeed;
  
  //if ( distance < 8 ) {
    //int wouldBeSpeed = s - stopSpeed;
    //wouldBeSpeed *= distance / 8.0f;
    //s = stopSpeed + wouldBeSpeed;
  //}
  
  int autoThrottle = constrain(s, 0, 255) //(s, stopSpeed, fullSpeed);
  //autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  setSpeedMotorA(speedA); !!!!!!!!!!!!!!
  setSpeedMotorB(speedB); !!!!!!!!!!!!!!
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();

  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && enabled) {
    float d = 0;
    //Start move loop here
    do {
      nss.listen();
      coolerLoc = checkGPS();
      bluetoothSerial.listen();
      
      d = geoDistance(coolerLoc, loc);
      float t = geoBearing(coolerLoc, loc) - geoHeading();
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(coolerLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(coolerLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading());
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && enabled && timeout>0);

    stop();
  }
}

void setupCompass() {
   /* Initialise the compass */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displayCompassDetails();
}

void ()
{


  // Compass
  setupCompass();

  // Motor pins
  // pinMode(MOTOR_A_EN_PIN, OUTPUT);
  // pinMode(MOTOR_B_EN_PIN, OUTPUT);
  // pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  // pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  // pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  // pinMode(MOTOR_B_IN_2_PIN, OUTPUT);

  //relay
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //Debugging via serial
  Serial.begin(9600); //previosuly 4800

  //GPS
  nss.begin(9600);

  //Bluetooth
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
  
  //avoidance code led and buzzer
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Ensure relays (and thus motors) are off at startup
  digitalWrite(Relay1, LOW); // Ensure relays (and thus motors) are off at startup
  digitalWrite(Relay2, LOW);

}

// Testing
void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  Serial.println(heading);
  
  while(!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
    Serial.println(heading);
    delay(500);
  }
  
  stop();
}

void loop()
{
  Blynk.run();

  unsigned int uS1 = sonar1.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int uS2 = sonar2.ping();
  unsigned int uS3 = sonar3.ping();
  
  long distance1 = uS1 / US_ROUNDTRIP_CM; // Convert time into distance
  long distance2 = uS2 / US_ROUNDTRIP_CM; // Convert time into distance
  long distance3 = uS3 / US_ROUNDTRIP_CM; // Convert time into distance

 if (Serial.available()) {
    char command = Serial.read(); // Read the command from Bluetooth

   //relay
  digitalWrite(Relay1, HIGH);
	digitalWrite(Relay2, LOW);
	delay(3000);
	digitalWrite(Relay2, HIGH);
	digitalWrite(Relay1, LOW);
	delay(3000);
   
    if (command == '1') {
      // Command to move servo to 90 degrees
      //servo.write(90);
     // digitalWrite(ledPin, HIGH); // Turn on LED

     //  turn off motors 
     digitalWrite(Relay1, LOW);
     digitalWrite(Relay2, LOW);  

     tone(buzzerPin, 1000); // Start buzzer
    } else if (command == '0') {
      // Command to move servo to 0 degrees
      //servo.write(0);
      //digitalWrite(ledPin, LOW); // Turn off LED
      noTone(buzzerPin); // Stop buzzer
    }
  }

  if (distance1 < 5 || distance2 < 5 || distance3 < 5) {
    // Object detected within 5 cm range in any sensor
    digitalWrite(Relay1, LOW); // Turn motors off
    digitalWrite(Relay2, LOW);
    digitalWrite(ledPin, HIGH); // Turn LED on
    tone(buzzerPin, 1000); // Start buzzer
  } else {
    // No object detected within 5 cm range in all sensors
    digitalWrite(Relay1, HIGH); // Turn motors on
    digitalWrite(Relay2, HIGH);
    digitalWrite(ledPin, LOW); // Turn LED off
    noTone(buzzerPin); // Stop buzzer
  }

    delay(100); // Short delay between reads for stability
  
}
