# Luggage-Cart

## Part 1: Pin Definitions

```
#define BLYNK_USE_DIRECT_CONNECT
```
This directive defines a preprocessor macro named BLYNK_USE_DIRECT_CONNECT. It likely indicates the usage of direct connection mode for Blynk, an IoT platform, in the code.

```
// Define Relays
#define Relay1 12
#define Relay2 11
```
These directives define macros Relay1 and Relay2 with values 12 and 11, respectively. They represent pin numbers to which relay modules are connected.

```
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters).
```
This directive defines a macro MAX_DISTANCE with a value of 200. It signifies the maximum distance (in centimeters) that a sensor (such as an ultrasonic sensor) should be configured to measure.

```
// Define LED and Buzzer
#define ledPin 13
#define buzzerPin 10
```
These directives define macros ledPin and buzzerPin with values 13 and 10, respectively. They indicate the pin numbers to which an LED and a buzzer are connected, typically for visual and auditory feedback in the project, respectively.


## Part 2: Library Imports and Directives
```
#include <Wire.h>
```
This line includes the Wire library, which is used for I2C communication between the Arduino and various I2C devices, like sensors and displays. I2C is a communication protocol that allows multiple devices to be connected to the same bus and communicate with the microcontroller.

```
#include <Adafruit_Sensor.h>
```
This includes the Adafruit Unified Sensor library, which provides a common interface and standardized data types for various sensors to simplify software development. It's especially useful when working with multiple sensor types in a project.

```
#include <Adafruit_HMC5883_U.h>
```
This line includes the library for the Adafruit HMC5883L (or similar) 3-axis magnetometer, which is often used as a digital compass. The "_U" suffix indicates this library is designed to work with the Adafruit Unified Sensor library.

```
#include <SoftwareSerial.h>
```
The SoftwareSerial library is included here to allow serial communication on other digital pins of the Arduino, not just the default 0 (RX) and 1 (TX) used for hardware serial. This is useful for projects that need to communicate with multiple serial devices.

```
#include <BlynkSimpleSerialBLE.h>
```
This line includes the Blynk library specifically tailored for establishing a Bluetooth Low Energy (BLE) connection using a simple serial interface. Blynk is a platform that allows you to control Arduino, Raspberry Pi, and similar boards over the Internet.

```
#include <NewPing.h>
```
The NewPing library is included for interfacing with ultrasonic sensors. It's designed to improve the operation of ultrasonic distance sensors, making it easier to measure distances with better timing and control.

```
#include "./TinyGPS.h"
```
This directive includes the TinyGPS library, but the "./" prefix suggests that the library file is located in the same directory as the project file. The TinyGPS library is used for parsing the NMEA data strings provided by GPS modules to extract and manipulate GPS coordinates and other data.

```
#include "./CoolerDefinitions.h"
```
Similar to the TinyGPS include directive, this line includes a local file named CoolerDefinitions.h that is expected to contain definitions, constants, or configurations specific to this project, potentially related to a "cooler" device or concept the project revolves around.


## Part 3: More Setup
```
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE)
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE)
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE)
```
These lines instantiate three objects of the NewPing class for ultrasonic sensors. Each object is initialized with trigger and echo pin numbers as well as the maximum distance (in centimeters) to measure. These objects are used to measure distances using ultrasonic sensors.

```
TinyGPS gps;
```
This line creates an object of the TinyGPS class. It's used for parsing GPS data.

```
bool enabled = false;:
```
This line declares a boolean variable named enabled and initializes it with the value false. It's likely used to control whether certain functionalities are enabled or disabled.

```
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial nss(GPS_TX_PIN, 255);:
```

These lines create instances of SoftwareSerial for serial communication. One instance is likely used for communication with a Bluetooth module (bluetoothSerial), and the other is used for communication with a GPS module (nss). They are initialized with transmit (TX) and receive (RX) pin numbers.

```
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);:
```
This line creates an object mag of the Adafruit_HMC5883_Unified class, which is used for interfacing with a compass module. The object is initialized with a unique sensor ID 12345.

## Part 4: Function Definitions

#### 1-checkGPS 

This function checkGPS() is responsible for reading GPS data and returning the location information. Let's break it down:

```
GeoLoc checkGPS() { ... }
```
This line declares a function named checkGPS() that returns a GeoLoc struct, which presumably contains latitude and longitude coordinates.

```
Serial.println("Reading onboard GPS: ");
```
This line prints a message to the serial monitor indicating that the GPS data is being read.

```
bool newdata = false;
```
This line initializes a boolean variable newdata to false, which is used to track whether new GPS data has been received.

```
unsigned long start = millis();
```
This line initializes an unsigned long variable start with the current value of milliseconds since the Arduino board started running. This is used for timing purposes.

```
while (millis() - start < GPS_UPDATE_INTERVAL) { ... }
```
This loop waits until the specified GPS_UPDATE_INTERVAL has elapsed since the start time. During this time, it continuously checks for new GPS data using the feedgps() function.

```
if (feedgps()) newdata = true;
```
Inside the loop, it calls the feedgps() function. If this function returns true, indicating that new GPS data is available, it sets newdata to true.

```
if (newdata) { return gpsdump(gps); }
```
After exiting the loop, if newdata is true, indicating that new GPS data has been received, it calls the gpsdump() function with the gps object (presumably containing GPS data) as an argument and returns the result.

```
GeoLoc coolerLoc; coolerLoc.lat = 0.0; coolerLoc.lon = 0.0; return coolerLoc;
```
If no new GPS data was received within the specified time, it creates a GeoLoc object named coolerLoc and initializes its latitude and longitude to 0.0. Then, it returns this object. This likely represents a default location when no GPS signal is available.


### 2- gpsdump

This function effectively extracts the GPS location from the TinyGPS object, formats it, and then provides it in a structured form that can be used elsewhere in the program.

```
GeoLoc gpsdump(TinyGPS &gps)
```
The function is named gpsdump and expects a reference to a TinyGPS object as its argument. It returns a GeoLoc struct, which should contain geographic coordinates.

```
float flat, flon;
```
Two floating-point variables, flat and flon, are declared to store the latitude and longitude values, respectively.

```
unsigned long age;
```
This variable will hold the age of the data (how old the GPS data is).

```
gps.f_get_position(&flat, &flon, &age);
```
This line calls the f_get_position method on the TinyGPS object, passing the addresses of flat, flon, and age as arguments. This method updates flat and flon with the current position's latitude and longitude, and age with the data's age.

```
GeoLoc coolerLoc;
coolerLoc.lat = flat;
coolerLoc.lon = flon;
```
A GeoLoc struct object named coolerLoc is created and its lat and lon fields are set to the values of flat and flon, respectively.

```
Serial.print(coolerLoc.lat, 7);
Serial.print(", ");
Serial.println(coolerLoc.lon, 7);
```
This sequence of commands prints the latitude and longitude to the serial monitor with 7 decimal places for precision, separated by a comma.

```
return coolerLoc;
```
Finally, the coolerLoc object containing the current location's latitude and longitude is returned.


### 3- feedgps

This function is designed to continuously read data from a GPS module via a serial interface and feed it to a TinyGPS object for processing. The purpose of this function is to parse the raw GPS data into a usable format that can then be used for various applications, such as determining the device's current location.

```
bool feedgps() {
```
This line declares the function feedgps(), which returns a boolean value. It indicates the purpose of the function is to feed data (presumably GPS data) as it becomes available.

```
while (nss.available())
```
This line starts a while loop that runs as long as there is data available to read from the nss serial port. The nss object is an instance of SoftwareSerial configured to communicate with the GPS module. available() returns the number of bytes available to read.

```
if (gps.encode(nss.read()))
```
Inside the loop, nss.read() reads the next byte from the GPS data stream. gps.encode() is called with this byte as its parameter. The encode() function is a method of the TinyGPS object gps, which processes the byte. The encode() method attempts to parse raw NMEA sentences from the GPS data. If a complete sentence is successfully parsed, encode() returns true.

```
return true;
```
If gps.encode() returns true, indicating that a new, valid piece of GPS data has been successfully parsed, the feedgps() function immediately returns true. This indicates to the caller that new GPS data has been successfully processed.

```
return false;
```
After the while loop, if no complete GPS sentences were successfully parsed (meaning the if condition inside the loop was never true), the function returns false. This signifies that no new GPS data was available or successfully processed during the execution of this function.

### 4- BLYNK_WRITE(V1) - Killswitch Hook

This snippet of code is a function definition for handling events from the Blynk application, specifically for a virtual pin (V1). Blynk is a platform that allows you to connect IoT devices to the internet and create mobile applications to control them.

```
enabled = !enabled;
```
This line toggles the state of a boolean variable named enabled. If enabled is true, it becomes false, and vice versa. This variable likely controls some aspect of the device's operation, such as whether it's actively performing a task or in a standby mode.

```
stop();
```
This calls a function named stop() to stop the wheels of the device. When the V1 button in the Blynk app is pressed, this device will stop moving, regardless of its previous state.


### 5- BLYNK_WRITE(V2) - GPS Streaming Hook

This function serves as an interface between the Blynk app and the device's navigation system. It receives GPS coordinates from the Blynk app, prepares them for navigation, and initiates the process of driving the device to the specified location.
```
GpsParam gps(param);
```
This line constructs a GpsParam object named gps by passing param as an argument. param contains the data sent from the Blynk app to virtual pin V2. The GpsParam class is likely provided by the Blynk library to facilitate handling GPS parameters.

```
Serial.println("Received remote GPS: ");
```
This line prints a message to the serial monitor indicating that remote GPS coordinates have been received from the Blynk app.

```
Serial.print(gps.getLat(), 7);
Serial.print(", ");
Serial.println(gps.getLon(), 7);
```
This line prints the latitude and longitude received from the Blynk app to the serial monitor. The gps.getLat() and gps.getLon() functions retrieve the latitude and longitude values from the gps object, respectively. The 7 inside Serial.print() specifies that the coordinates should be printed with seven decimal places.

```
GeoLoc phoneLoc;
phoneLoc.lat = gps.getLat();
phoneLoc.lon = gps.getLon();
```
This code creates a GeoLoc struct named phoneLoc and assigns the latitude and longitude received from the Blynk app to its lat and lon members, respectively.

```
driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
```
This line calls a function named driveTo() and passes the phoneLoc object and a constant named GPS_STREAM_TIMEOUT as arguments. Presumably, this function is responsible for driving the device to the specified GPS location received from the Blynk app. GPS_STREAM_TIMEOUT is likely a timeout value used to limit the duration of the driving operation.


### 6- BLYNK_WRITE(V3) - Terminal Hook

```
BLYNK_WRITE(V3)
```
This macro provided by the Blynk library defines a function that is automatically called whenever the Blynk server sends a write command to virtual pin V3 from the Blynk app. In this case, the function is triggered when data is sent to V3.

```
Serial.print("Received Text: ");
Serial.println(param.asStr());
```
This section prints a message to the serial monitor indicating that text data has been received from the Blynk app. param.asStr() retrieves the data sent to virtual pin V3 as a string, and Serial.println() prints it to the serial monitor.

```
String rawInput(param.asStr());
```
This line creates a new String object named rawInput and initializes it with the data received from the Blynk app. It's essentially making a copy of the received string for further processing.

```
int colonIndex;
int commaIndex;
```
Declaration of integer variables colonIndex and commaIndex without initialization. These variables will be used to store the indices of the colon and comma characters in the received string.

```
  do {
```
This line starts a do-while loop. The loop will execute at least once and will continue until colonIndex becomes -1.

```
commaIndex = rawInput.indexOf(',');
colonIndex = rawInput.indexOf(':');
```
These lines find the indices of the comma and colon characters in the rawInput string using the indexOf() function. The commaIndex stores the position of the comma, while colonIndex stores the position of the colon.

```
if (commaIndex != -1) {
```
This condition checks if a comma was found in the string. If commaIndex is not equal to -1, it means a comma exists in the string.

```
String latStr = rawInput.substring(0, commaIndex);
String lonStr = rawInput.substring(commaIndex+1);
```
These lines extract substrings from rawInput. latStr contains the characters from the start of the string up to (but not including) the comma, while lonStr contains the characters after the comma.

```
if (colonIndex != -1)
  lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
```
This condition checks if a colon was found in the string. If colonIndex is not equal to -1, it means a colon exists in the string. In this case, lonStr is updated to contain only the characters between the comma and the colon.

```
float lat = latStr.toFloat();
float lon = lonStr.toFloat();
```
These lines convert the extracted substrings representing latitude and longitude to floating-point numbers using the toFloat() function.

```
if (lat != 0 && lon != 0) {
```
This condition checks if both latitude and longitude are non-zero. If they are both non-zero, it indicates that valid coordinates were extracted.

```
GeoLoc waypoint;
waypoint.lat = lat;
waypoint.lon = lon;
Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
```
If valid coordinates are found, a GeoLoc struct named waypoint is created and initialized with the latitude and longitude values. Then, a message indicating the waypoint coordinates is printed to the serial monitor, followed by a call to the driveTo() function with the waypoint coordinates and a timeout value (GPS_WAYPOINT_TIMEOUT) as arguments.

```
rawInput = rawInput.substring(colonIndex + 1);
```
This line updates rawInput to contain only the characters after the colon. It essentially removes the processed portion of the string for the next iteration of the loop.

```
  } while (colonIndex != -1);
```
This line closes the do-while loop. It continues looping as long as colonIndex is not equal to -1, meaning there are still colon characters remaining in the string to be processed.


### 7- displayCompassDetails

This function retrieves detailed information about a magnetometer sensor and prints it to the serial monitor. This can be useful for debugging purposes, to verify the sensor is working correctly, or to get to know the capabilities and limitations of the sensor you're working with.

```
void displayCompassDetails(void)
```
This line declares a function named displayCompassDetails that does not return any value (void) and does not take any parameters.

```sensor_t sensor;
```
Declares a variable named sensor of type sensor_t. This type is defined in the Adafruit Sensor library and is used to store information about a sensor, such as its name, version, unique ID, and measurement ranges.

```
mag.getSensor(&sensor);
```
Calls the getSensor() method on the mag object (which represents the magnetometer) to populate the sensor variable with the magnetometer's details. The & operator is used to pass the address of the sensor variable, allowing getSensor() to modify its contents directly.

```
Serial.println("------------------------------------");
Serial.print  ("Sensor:       "); Serial.println(sensor.name);
Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
```
These commands are used to print the sensor's details to the serial monitor. This includes the sensor's name, driver version, unique ID, maximum and minimum measurable values (in microteslas), and its resolution (also in microteslas).


```
Serial.println("------------------------------------")
Serial.println("");
```
Used to print a line of dashes and a blank space to the serial monitor for formatting purposes, making the output easier to read by separating the sensor details from other serial output.

```
delay(500);
```
Introduces a 500-millisecond pause in the execution of the program. This delay is often used to prevent flooding the serial monitor with data if displayCompassDetails is called in a loop.


## Part 5 Geo

This code uses preprocessor directives to define two constants, DEGTORAD and RADTODEG, if they have not already been defined. 

Preprocessor directives are processed before the actual compilation of code begins, making these checks and definitions an effective way to ensure that these constants are available globally throughout your code without redefining them, which could lead to compilation errors.

```
#ifndef DEGTORAD
```
This line checks if DEGTORAD has not been defined yet. #ifndef stands for "if not defined".

```
#define DEGTORAD 0.0174532925199432957f
```
If DEGTORAD is not defined, this line defines it. DEGTORAD is set to the numerical value 0.0174532925199432957f, which is the equivalent of π/180. This value is used to convert degrees to radians (since there are π radians in 180 degrees).

```
#define RADTODEG 57.295779513082320876f
```
This line defines RADTODEG, assuming DEGTORAD was not previously defined. RADTODEG is set to 57.295779513082320876f, which is the equivalent of 180/π. This value is used to convert radians to degrees.

```
#endif
```
This directive ends the conditional block started by #ifndef. It ensures that DEGTORAD and RADTODEG are defined only once, regardless of how many times this snippet is encountered during the pre-processing of source files.

The use of f at the end of the numerical constants specifies them as float literals, ensuring that the computations involving these constants are performed in single-precision floating-point arithmetic, which is often faster and requires less memory than double precision, although with a potential trade-off in terms of precision for very large or very small numbers.


### GeoBearing

This function calculates the bearing or direction from one geographical location to another. Here's a line-by-line explanation:

```
float geoBearing(struct GeoLoc &a, struct GeoLoc &b)
```
The function geoBearing takes two arguments, a and b, both references to GeoLoc structures (presumably containing latitude and longitude coordinates), and returns a float value. This structure is not shown, but we can assume it contains at least two members: lat for latitude and lon for longitude.

```
Calculate y Component: float y = sin(b.lon-a.lon) * cos(b.lat);
```
This line calculates the y component of the bearing calculation. It computes the sine of the difference in longitude between point b and point a, then multiplies this by the cosine of the latitude of point b. This calculation is part of the formula used to determine the bearing from point a to point b on a spherical Earth model.

```
float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
```
This line calculates the x component of the bearing calculation. It involves a bit more arithmetic, taking the cosine of a's latitude times the sine of b's latitude, then subtracting the product of the sine of a's latitude, the cosine of b's latitude, and the cosine of the difference in longitudes between b and a. This is another part of the spherical Earth model for calculating bearing.

```
return atan2(y, x) * RADTODEG;
```
Finally, the function returns the bearing calculated using the atan2 function, which computes the arc tangent of the quotient of its arguments. The atan2 function is used here to get the angle in radians from the origin to the point (x, y), which corresponds to the bearing from a to b. This angle in radians is then converted to degrees by multiplying it with the RADTODEG constant (defined as 57.295779513082320876f, which is 180/π) to convert the result from radians to degrees, making it more understandable in many contexts where degrees are the standard for angles.

The bearing returned by this function indicates the direction one must travel from point a to reach point b, measured in degrees from the north (0 degrees is north, 90 degrees is east, 180 degrees is south, and 270 degrees is west).


## geoDistance

The geoDistance function calculates the distance between two points on the Earth's surface given their latitudes and longitudes. This calculation uses the Haversine formula, which is particularly useful in navigation because it accounts for the Earth's spherical shape. Here's a breakdown of the code:

```
const float R = 6371000;
```
This line defines the radius of the Earth in meters. The Haversine formula requires this value to calculate the great-circle distance between two points. The Earth is not a perfect sphere, but this value is a reasonable average radius for many practical purposes.

```
float p1 = a.lat * DEGTORAD;
Converts the latitude of point a from degrees to radians.
float p2 = b.lat * DEGTORAD;
Converts the latitude of point b from degrees to radians.
float dp = (b.lat-a.lat) * DEGTORAD;
```
Calculates the difference in latitude between points b and a, and converts that difference from degrees to radians.

```
float dl = (b.lon-a.lon) * DEGTORAD;
```
Calculates the difference in longitude between points b and a, and converts that difference from degrees to radians.

```
float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
```
This line calculates the square of half the chord length between the points, a key part of the Haversine formula. It involves the sine of half the difference in latitude, the cosine of both latitudes, and the sine of half the difference in longitude.

```
float y = 2 * atan2(sqrt(x), sqrt(1-x));
```
This line calculates the angular distance in radians, using atan2 to compute the arc tangent of the quotient, representing the great-circle distance as an angle.

```
return R * y;
```
Finally, the function returns the distance between the two points by multiplying the angular distance in radians (y) by the Earth's radius (R). The result is the distance between points a and b along the surface of the Earth, in meters.


## geoHeading

The geoHeading function calculates the compass heading using readings from a magnetometer sensor (in this case, from the Adafruit_HMC5883_Unified magnetometer). Here's a breakdown of the function:

```
sensors_event_t event;
```
Defines a variable event of type sensors_event_t to hold the sensor data.

```
mag.getEvent(&event);
```
Retrieves the latest magnetometer readings and stores them in event.

```
float heading = atan2(event.magnetic.y, event.magnetic.x);
```
Calculates the raw heading (in radians) using the arctangent of the Y and X magnetic readings. The atan2 function is used to handle the correct quadrant of the calculated angle, providing an angle ranging from -π to π (-180° to 180°). This angle represents the direction to magnetic north.

```
heading -= DECLINATION_ANGLE;
```
Adjusts the heading by subtracting the magnetic declination angle, which is the angle between magnetic north and true north at your location. This correction is necessary for accurate real-world orientation.

```
heading -= COMPASS_OFFSET;
```
Additionally adjusts the heading by a fixed offset specific to the compass's installation or calibration (COMPASS_OFFSET). This is used to correct systematic errors in the sensor readings.

```
if(heading < 0)
  heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;
```
The code corrects the heading to ensure it falls within a 0° to 360° range:
If the heading is negative (heading < 0), 2*PI is added to make it positive.
If the heading exceeds 2*PI (heading > 2*PI), 2*PI is subtracted to bring it within range.

```
float headingDegrees = heading * 180/M_PI;
```
Converts the heading from radians to degrees for easier interpretation and use in navigation.

```
while (headingDegrees < -180) headingDegrees += 360;
while (headingDegrees >  180) headingDegrees -= 360;
```
The last two while loops adjust the headingDegrees to fall within a -180° to 180° range, which is often preferred for navigation and readability. This mapping makes it easier to interpret the heading as a direction relative to true north.

```
return headingDegrees;
```
The function ultimately returns the compass heading in degrees, adjusted for magnetic declination and any fixed offset, normalized to a -180° to 180° scale. This heading can be used in navigation applications to determine the direction the sensor (and by extension, the device or vehicle it's attached to) is facing relative to true north.


### setSpeedMotorA

This function activates Relay2 and deactivate Relay1, which could be part of controlling the direction or operation of Motor A
```
digitalWrite(Relay2, HIGH);
```
This line sets the digital pin connected to Relay2 to HIGH, which typically indicates turning on or activating a relay. In this context, Relay2 might be associated with controlling the motor speed or direction.

```
digitalWrite(Relay1, LOW);
```
This line sets the digital pin connected to Relay1 to LOW. This action might be related to the control of the motor's direction or other aspects of motor operation, depending on the specific circuitry and configuration.


### setSpeedMotorB


This function, setSpeedMotorB, is structured similarly to setSpeedMotorA but with actions that affect a different motor, referred to here as "Motor B." The explanation of each line is as follows:

```
digitalWrite(Relay1, HIGH);
```
This line activates Relay1 by setting its control pin to HIGH. In the context of controlling Motor B, this could imply that Relay1 is involved in setting the direction or operation mode of the motor. The exact effect depends on how the relay is wired in the circuit.

```
digitalWrite(Relay2, LOW);
```
Conversely, this line deactivates Relay2 by setting its control pin to LOW. This action, combined with the activation of Relay1, suggests a configuration where the two relays together control the operation of Motor B, potentially altering its direction or operational state.

The function essentially switches the relay states compared to setSpeedMotorA, indicating that Relay1 and Relay2 are used in tandem to control the direction or operation of two separate motors (Motor A and Motor B) in a complementary manner.


### stop

The stop function is designed to halt the operation of motors, presumably by disengaging the power or control signals that drive the motors. Here's a breakdown of its components:

```
digitalWrite(Relay2, LOW);
```
This command actively sets the control pin for Relay2 to LOW, which would deactivate Relay2. Given the context of previous functions, deactivating Relay2 is part of the process to stop the motors.

```
digitalWrite(Relay1, LOW);
```
Similar to the previous line, this sets the control pin for Relay1 to LOW, deactivating it. This action, in tandem with deactivating Relay2, signifies the stopping of motor operations, assuming Relay1 and Relay2 control the power or direction for Motor A and Motor B.


### drive

This function aims to drive the vehicle towards a target by adjusting its speed based on the distance and direction based on the turn angle. However, some variables and logic appear to be missing or commented out, which could affect its functionality as described.

```
int fixedSpeed = 50;
```
Initializes a variableI for a fixed speed setting, though it's not used later in the provided code snippet.

```
int autoThrottle = constrain(s, 0, 255)
```
Ensures the speed (s) is within the PWM range (0 to 255). However, s is not defined in the provided snippet, suggesting there may be a missing line or a mistake. It might be intended to use fixedSpeed or another variable instead.

```
while (t < -180) t += 360;
while (t > 180) t -= 360;
```
Normalizes the turn angle (t) to a range between -180 and 180 degrees. This adjustment makes it easier to interpret the direction and magnitude of the turn.
Serial Debugging Outputs:
```
Serial.print("turn: ");
Serial.println(t);
Serial.print("original: ");
Serial.println(turn);
```
Prints the adjusted turn value and the original turn value to the serial monitor for debugging or monitoring purposes.

```
float t_modifier = (180.0 - abs(t)) / 180.0;
```
Calculates a modifier based on the turn angle, which diminishes linearly with the absolute value of t. This effectively reduces the speed of one motor to turn the vehicle.

```
float autoSteerA = 1;
float autoSteerB = 1;
```
AutoSteerA and autoSteerB are set to 1 by default, indicating no speed reduction. Depending on the turn direction, one of these will be adjusted using t_modifier to control the turn by reducing the speed on one side of the vehicle.

```
int speedA = (int) (((float) autoThrottle) * autoSteerA);
int speedB = (int) (((float) autoThrottle) * autoSteerB);
```

Calculate the final speed for each motor by applying the autoSteer modifier. This determines how much to slow down one side of the vehicle to achieve the desired turn

```
setSpeedMotorA(speedA);
setSpeedMotorB(speedB);
```
Call functions to adjust the speed of Motor A and Motor B to the calculated values, thereby controlling the vehicle's movement and direction.


### driveTo

```
nss.listen();
```
This line switches the serial port to listen to the GPS module's serial interface, allowing the program to receive data from the GPS.

```
GeoLoc coolerLoc = checkGPS();
```
This line obtains the current GPS location of the vehicle by calling the checkGPS() function and stores it in the coolerLoc variable of type GeoLoc.

```
bluetoothSerial.listen();
```
This line switches the serial port to listen to the Bluetooth serial interface, presumably to maintain communication with a remote device or controller.

```
if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && enabled) {
```
This conditional statement checks if the current latitude and longitude are not zero (indicating a valid GPS fix) and if the navigation is enabled (enabled flag is true). If these conditions are met, the vehicle proceeds with the navigation.

```
do {
```
This line initializes a do-while loop that will execute at least once and then continue executing as long as the conditions inside the loop remain true.

```
nss.listen();
coolerLoc = checkGPS();
```
These lines update the current GPS location (coolerLoc) by calling the checkGPS() function within the loop.

```
d = geoDistance(coolerLoc, loc);
float t = geoBearing(coolerLoc, loc) - geoHeading();
```
These lines calculate the distance (d) to the destination and the required turn angle (t) by calling the geoDistance() and geoBearing() functions, respectively. The turn angle is adjusted based on the difference between the bearing to the destination and the current heading of the vehicle obtained from geoHeading().

```
Serial.print("Distance: ");
Serial.println(geoDistance(coolerLoc, loc));
Serial.print("Bearing: ");
Serial.println(geoBearing(coolerLoc, loc));
Serial.print("heading: ");
Serial.println(geoHeading());
```
These lines print debugging information such as the distance to the destination, the bearing to the destination, and the current heading of the vehicle for monitoring and troubleshooting purposes.

```
drive(d, t);
```
This line calls the drive() function with the calculated distance (d) and turn angle (t) to adjust the vehicle's movement towards the destination.

```
timeout -= 1;
```
This line decrements the timeout counter to prevent infinite looping and timeout if the destination cannot be reached.

```
} while (d > 3.0 && enabled && timeout > 0);
```
This line specifies the conditions for continuing the loop: the distance to the destination (d) must be greater than 3 meters, the system must be enabled, and the timeout must not have expired.

```
stop();
```
Once the loop exits (either the vehicle is within 3 meters of the destination, the system is disabled, or the timeout expires), this line calls the stop() function to halt the vehicle.



### setupCompass

The setupCompass() function is designed to ensure that the HMC5883 magnetometer sensor is properly connected and initialized before proceeding with the rest of the program. It provides immediate feedback if there are any issues with the sensor's connection, which is crucial for troubleshooting hardware issues.

```
if(!mag.begin())
```
This line attempts to initialize the compass sensor using the begin() method of the mag object. This method returns true if the sensor is successfully initialized and false if it fails (e.g., if the sensor is not connected properly).

```
{
  /* There was a problem detecting the HMC5883 ... check your connections */
  Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  while(1);
}
```
If initialization fails (mag.begin() returns false), this block of code executes. It prints an error message to the serial monitor indicating that the HMC5883 sensor was not detected. The while(1); loop then effectively halts the program, preventing further execution until the issue is resolved. This is a safety feature to ensure that the rest of the program does not run without the necessary sensor input.

```
displayCompassDetails();
```
Once the sensor is successfully initialized, this line calls the displayCompassDetails() function. This function prints out some basic information about the compass sensor to the serial monitor, such as its name, driver version, unique ID, maximum and minimum measurable values (in microteslas), and its resolution. This is useful for verifying the sensor's operation and understanding its capabilities.


## Part 6: Setup

This is the setup() function in an Arduino sketch. It is called once when the microcontroller is powered up or reset. Let's go through it step by step:

```
setupCompass();
```
This line calls the setupCompass() function to initialize and configure the compass sensor.

```
pinMode(Relay1, OUTPUT);
pinMode(Relay2, OUTPUT);
```
These lines set the specified pins (Relay1 and Relay2) as output pins. These pins are likely connected to relays that control the motors.

```
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH);
```
These lines configure the built-in LED (usually connected to pin 13 on most Arduino boards) as an output pin and set its initial state to HIGH, which typically turns it off (assuming the LED is active LOW).

```
Serial.begin(9600);
bluetoothSerial.begin(9600);
```
These lines initialize serial communication at a baud rate of 9600 baud. Serial.begin(9600) initializes the hardware serial port for communication with the Arduino IDE Serial Monitor, while bluetoothSerial.begin(9600) initializes a software serial port for communication with a Bluetooth module.

```
nss.begin(9600);
```
This line initializes serial communication with the GPS module using software serial at a baud rate of 9600 baud.

```
Blynk.begin(bluetoothSerial, auth);
```
This line initializes communication with the Blynk server using Bluetooth serial communication and a specified authentication token (auth). Blynk is a platform for building IoT projects, and this line establishes the connection with the Blynk server to enable remote control and monitoring of the Arduino device.

```
pinMode(ledPin, OUTPUT);
pinMode(buzzerPin, OUTPUT);
```
These lines set the specified pins (ledPin and buzzerPin) as output pins. These pins are likely connected to an LED and a buzzer used for an avoidance system.

```
digitalWrite(Relay1, LOW);
digitalWrite(Relay2, LOW);
```
These lines ensure that both relay pins are set to LOW, which typically turns off the connected motors at startup to prevent them from running until the sketch explicitly commands them to do so.


## Part 7: Test Drive North

This function effectively drives the vehicle forward until it aligns itself within a narrow range around the North direction, continuously updating its heading and adjusting its orientation as needed.

```
float heading = geoHeading();
```

This line calls the geoHeading() function to obtain the current heading (orientation) of the vehicle.

```
int testDist = 10;
Serial.println(heading);
```
It initializes a variable testDist to a value of 10, which might represent a test distance in some unit.
It prints the initial heading value to the serial monitor for debugging purposes.

```
while (!(heading < 5 && heading > -5)) {
```
This line starts a while loop that continues as long as the vehicle's heading is not within a narrow range around the North direction (within ±5 degrees).

```
drive(testDist, heading);
heading = geoHeading();
```
Inside the loop, it calls the drive() function to move the vehicle forward with a specified distance (testDist) and orientation (heading).
It updates the heading variable by calling geoHeading() again to get the latest heading after driving.

```
Serial.println(heading);
delay(500);
```
It prints the updated heading value to the serial monitor.
It adds a delay of 500 milliseconds to control the loop execution speed.

```
stop();
```
After the loop exits (when the vehicle is facing North), it stops the vehicle by calling the stop() function.


## Part 8: Loop

```
Blynk.run();
```
This line is responsible for handling Blynk communication. It ensures that the Blynk application and the device communicate with each other.

```
unsigned int uS1 = sonar1.ping();
unsigned int uS2 = sonar2.ping();
unsigned int uS3 = sonar3.ping();
```
These lines send out ultrasonic pings using each of the three ultrasonic sensors (sonar1, sonar2, and sonar3) and store the echo response times in microseconds.

```
long distance1 = uS1 / US_ROUNDTRIP_CM;
long distance2 = uS2 / US_ROUNDTRIP_CM;
long distance3 = uS3 / US_ROUNDTRIP_CM;
```
These lines convert the echo response times obtained from the ultrasonic sensors into distances in centimeters.

```
if (Serial.available()) {
```
This line checks if there is any data available to read from the serial port (Bluetooth).

```
char command = Serial.read();
```
It reads the command sent from the Bluetooth device.

```
digitalWrite(Relay1, HIGH);
digitalWrite(Relay2, LOW);
delay(3000);
digitalWrite(Relay2, HIGH);
digitalWrite(Relay1, LOW);
delay(3000);
```
These lines appear to perform a sequence of actions (possibly activating some components) when a command is received.
The actions include toggling the relays (Relay1 and Relay2) for a specific duration and then waiting for another specific duration.

```
if (command == '1') {
```
If the received command is '1', it executes the associated actions.

```
digitalWrite(Relay1, LOW);
digitalWrite(Relay2, LOW);
tone(buzzerPin, 1000);
```
It turns off the motors by setting both relay pins low and starts the buzzer with a frequency of 1000 Hz.

```
} else if (command == '0') {
```
If the received command is '0', it executes the associated actions.

```
noTone(buzzerPin);
```
It stops the buzzer by disabling the tone on buzzerPin.

```
if (distance1 < 5 || distance2 < 5 || distance3 < 5) {
```
This line checks if any of the ultrasonic sensors detect an obstacle within a 5 cm range.

```
digitalWrite(Relay1, LOW);
digitalWrite(Relay2, LOW);
digitalWrite(ledPin, HIGH);
tone(buzzerPin, 1000);
```
If an obstacle is detected, it turns off the motors, turns on the LED, and starts the buzzer.

```
} else {
```
If no obstacle is detected:

```
digitalWrite(Relay1, HIGH);
digitalWrite(Relay2, HIGH);
digitalWrite(ledPin, LOW);
noTone(buzzerPin);
```
It turns on the motors, turns off the LED, and stops the buzzer.

```
delay(100);
```
It introduces a short delay of 100 milliseconds between sensor readings for stability and to prevent overwhelming the system with rapid iterations.











