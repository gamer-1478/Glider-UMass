#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Create GPS instance and Serial2 for GPS
TinyGPSPlus gps;
HardwareSerial GPS(2);  // Use Serial2 for GPS

// Define pins for servos
const int servoPin1 = 2;  // D5
const int servoPin2 = 4;  // D18

// Create servo objects
Servo servo1;
Servo servo2;

// Initialize servo angles
int angle1 = 90;  // Default angle for servo 1
int angle2 = 90;  // Default angle for servo 2

// Constants for controlling the glider
const double minSafeAirspeed = 5.0;      // Minimum safe acceleration in m/s²
const double stallSpeedThreshold = 4.0;  // Stall speed in m/s²
const double recoveryAngle = 15.0;       // Angle to pitch down for recovery
const double correctionAngle = 5.0;      // Angle to pitch up to maintain altitude
double launchAngle = 45.0;               // True magnetic angle at launch

// Target coordinates
double targetLat = 42.390981;   // Example latitude for target
double targetLng = -72.526875;  // Example longitude for target

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Test");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);


  // Initialize Serial2 for GPS communication
  GPS.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX pins for GPS

  // Attach servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  // Set initial positions
  moveServo1(angle1);
  moveServo2(angle2);

  // Calibrate MPU6050
  // TBD How to calibrate
  Serial.println("Setup complete. Starting flight control...");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  bool gpsDataAvailable = false;

  // Read GPS data
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
  }

  // Get orientation angles from MPU6050
  double roll = g.gyro.x;   // Roll in degrees
  double pitch = g.gyro.y;  // Pitch in degrees
  double yaw = g.gyro.z;    // Yaw in degrees

  // Calculate current heading based on yaw and launch angle
  double currentHeading = fmod((launchAngle + yaw + 360.0), 360.0);

  // Print current heading and accelerometer data
  Serial.print("Current Heading: ");
  Serial.println(currentHeading);

  // Read acceleration to check airspeed (using forward component)
  double accelX = a.acceleration.x;

  // Forward acceleration (assume this corresponds to airspeed)
  Serial.print("Forward Acceleration: ");
  Serial.println(accelX);


  // Check GPS data availability and print

  if (accelX < stallSpeedThreshold) {
    // Airspeed below stall speed, pitch down for recovery
    Serial.println("Airspeed below stall speed, pitching down for recovery.");
    servo1.write(angle1 + recoveryAngle);
    servo2.write(angle2 + recoveryAngle);
  } else if (accelX > minSafeAirspeed) {
    if (gps.satellites.isValid() && gps.satellites.value() > 0) {
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());

      if (gps.location.isUpdated()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        double distanceToTarget = calculateDistance(gps.location.lat(), gps.location.lng(), targetLat, targetLng);
        double bearingToTarget = calculateBearing(gps.location.lat(), gps.location.lng(), targetLat, targetLng);
        Serial.print("Distance to Target: ");
        Serial.println(distanceToTarget);
        Serial.print("Bearing to Target: ");
        Serial.println(bearingToTarget);

        // Move servos based on bearing
        moveServos(bearingToTarget, currentHeading);
      }
    } else {
      // Maintain current position
      Serial.println("Maintaining current servo positions.");
      servo1.write(angle1);
      servo2.write(angle2);
    }
  }
}

  // Function to move servos based on the bearing to the target
  void moveServos(double targetBearing, double currentHeading) {
    double headingDifference = targetBearing - currentHeading;
    if (headingDifference < 0) headingDifference += 360;
    if (headingDifference >= 360) headingDifference -= 360;

    // Determine servo angles based on heading difference
    if (headingDifference < 180) {
      // Turn right
      servo1.write(angle1 + headingDifference / 2);
      servo2.write(angle2 - headingDifference / 2);
    } else {
      // Turn left
      servo1.write(angle1 - (360 - headingDifference) / 2);
      servo2.write(angle2 + (360 - headingDifference) / 2);
    }

    Serial.print("Servo 1 Angle: ");
    Serial.print(servo1.read());
    Serial.print(", Servo 2 Angle: ");
    Serial.println(servo2.read());
  }

  // Calculate distance between two points in meters using Haversine formula
  double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000;  // Radius of Earth in meters
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);

    double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
  }

  // Calculate bearing between two points
  double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = radians(lon2 - lon1);
    double y = sin(dLon) * cos(radians(lat2));
    double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    double bearing = atan2(y, x);
    return fmod((degrees(bearing) + 360), 360);  // Normalize to 0-360 degrees
  }

  // Function to move Servo 1
  void moveServo1(int angle) {
    angle = constrain(angle, 0, 180);  // Ensure angle is within range
    servo1.write(angle);
    Serial.print("Moving Servo 1 to ");
    Serial.print(angle);
    Serial.println(" degrees");
  }

  // Function to move Servo 2
  void moveServo2(int angle) {
    angle = constrain(angle, 0, 180);  // Ensure angle is within range
    servo2.write(angle);
    Serial.print("Moving Servo 2 to ");
    Serial.print(angle);
    Serial.println(" degrees");
  }
