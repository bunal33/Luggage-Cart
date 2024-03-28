


// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 13° 24' E (Positive), which is ~13 Degrees, or (which we need) 0.23 radians
// #define DECLINATION_ANGLE 0.23f

// For Tampa it's -6° 5' W (Negative), which is ~ -6 degrees or 0.10472 radians
#define DECLINATION_ANGLE 0.10472f

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 0.0f // need compass offset

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45


// Definitions (don't edit these)
struct GeoLoc {
  float lat;
  float lon;
};


