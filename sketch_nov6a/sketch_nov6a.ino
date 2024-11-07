#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <math.h>

// Create GPS instance and Serial2 for GPS
TinyGPSPlus gps;
HardwareSerial GPS(2); // Use Serial2 for GPS

// Define pins for servos
const int servoPin1 = 2;   // D5
const int servoPin2 = 4;  // D18

// Create servo objects
Servo servo1;
Servo servo2;

// Initialize servo angles
int angle1 = 90;  // Default angle for servo 1
int angle2 = 90;  // Default angle for servo 2

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("Test");
  
  // Initialize Serial2 for GPS communication
  GPS.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for GPS

  // Attach servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  // Set initial positions
  moveServo1(angle1);
  moveServo2(angle2);
}

void loop() {
  bool gpsDataAvailable = false;

  // Read GPS data
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
    gpsDataAvailable = true;  // If data was read, mark it as available
  }

  // Print GPS availability status
  if (gpsDataAvailable) {
    Serial.println("GPS data available.");
  } else {
    Serial.println("GPS data not available.");
  }

  // Print the number of satellites in view if the GPS has a satellite fix
  if (gps.satellites.isValid()) {
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("Satellites data not valid.");
  }


  // Print GPS data if valid
  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }
}


// Calculate distance between two points in meters using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Radius of Earth in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Calculate bearing between two points
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) -
             sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double bearing = atan2(y, x);
  return fmod((degrees(bearing) + 360), 360); // Normalize to 0-360 degrees
}

// Function to move Servo 1
void moveServo1(int angle) {
  angle = constrain(angle, 0, 180); // Ensure angle is within range
  servo1.write(angle);
  Serial.print("Moving Servo 1 to ");
  Serial.print(angle);
  Serial.println(" degrees");
}

// Function to move Servo 2
void moveServo2(int angle) {
  angle = constrain(angle, 0, 180); // Ensure angle is within range
  servo2.write(angle);
  Serial.print("Moving Servo 2 to ");
  Serial.print(angle);
  Serial.println(" degrees");
}

