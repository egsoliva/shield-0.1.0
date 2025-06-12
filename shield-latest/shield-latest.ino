/**********************************************************************************************************
*
*   SHIELD v1.1 - Smart Hard Hat with Impact Emergency Location Detector for Instant Disaster Response
*
*   COMMENTS:
*     - This version has no threshold and only focuses on setting up the SIM800L EVB module, GPS module,
*       and the ADXL345 module (a temporary threshold is placed to test SMS and calls)
*     - The pins for the modules and the buzzer are defined in the code
*     - The range set in the code varies from -16 to +16g. This is to allow readings from heavy impact due
*       to debris or objects during an emergency
*     - The data rate set in the code is 100 Hz
*     - ADXL345 Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
*     - Threshold should be clearly defined in the next update
*     - Add a debug feature in the next update to monitor errors
*     - Add pitch, roll, and yaw if possible to detect movement from head
*
**********************************************************************************************************/

#include <Wire.h>
#include <ADXL345.h>
#include <TinyGPS++.h>
#include <Filters.h>
#include <AH/STL/cmath>     
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>
#include <Filters/MedianFilter.hpp>

#define BUZZER 4
#define SDA 23
#define SCL 22
#define GPS_RX 17
#define GPS_TX 16
#define SIM800L_RX 19
#define SIM800L_TX 18
#define GPS_BAUD 9600
#define SIM800L_BAUD 9600
#define THRESHOLD 10 // Threshold to compare with SMA
#define ACCELERATION_THRESHOLD 17.658f

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
HardwareSerial sim800Serial(1);

ADXL345 accelerometer;

unsigned long startMillis, currentMillis;
const unsigned long interval = 1000;
float x_accel_raw, y_accel_raw, z_accel_raw, accel_raw, x_filtered, 
      y_filtered, z_filtered, x_medfilt, y_medfilt, z_medfilt, accel, 
      z_accel_raw_g;
double pitch, roll, yaw, filt_pitch, filt_roll, tilt;
float movsum_x = 0, movsum_y = 0, movsum_z = 0;
float signal_magnitude_area;

const char *numbers[] = {"adviserNumber", "guardianNumber", "nurseNumber"}; // Place their numbers in the respective places

double latitude = gps.location.lat();
double longitude = gps.location.lng();

const double f_s = 45; // Sample frequency (Hz)
const double f_c = 10; // Cut-off frequency (Hz)
const double f_n = 2 * f_c / f_s; // Normalized cut-off frequency (Hz)

MedianFilter<3, float> medfilt_X = {0}; // Median filter 
MedianFilter<4, float> medfilt_Y = {0};
MedianFilter<3, float> medfilt_Z = {0};

double minutes = millis()/60000;

void setup() {
  Serial.begin(115200);  
  pinMode(BUZZER, OUTPUT);
  Wire.begin(SDA, SCL);
  accelerometer.begin();
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  sim800Serial.begin(SIM800L_BAUD, SERIAL_8N1, SIM800L_RX, SIM800L_TX);
  setAccelerometerSettings();
  startMillis = millis();
}

void loop() {
  readAccelerometerData();
  readGPSData();
  readGyroscopeData();

  currentMillis = millis();
  if(currentMillis - startMillis < 1000) {
    movsum_x += x_medfilt;
    movsum_y += y_medfilt;
    movsum_z += z_medfilt;
    signal_magnitude_area = movsum_x + movsum_y + movsum_z;
    
    if(signal_magnitude_area > THRESHOLD) {
      if(accel > ACCELERATION_THRESHOLD && gpsSerial.available() > 0) {
        tone(BUZZER, 4000, 5000);
        sendEmergency();
      }
      else {
        if(tilt >= 0 && tilt <= 60) {
          Serial.println("Upright Active");
        }
        else {
          Serial.println("Lying Active");
        }
      }
    }
    else {
      if(tilt >= 0 && tilt <= 60) {
        if(tilt >= 20 && tilt <= 60) {
          Serial.println("Sitting");
        }
        else {
          Serial.println("Standing");
        }
      }
      else {
        Serial.println("Lying");
      }
    }
  }
  else { // Reset variables
    movsum_x = 0;
    movsum_y = 0;
    movsum_z = 0;
    signal_magnitude_area = 0;
    startMillis = currentMillis;
  }
}

// You may modify the range and data rate (other settings can be found on the ADXL345 library)
void setAccelerometerSettings() {
  accelerometer.setRange(ADXL345_RANGE_16G);
  accelerometer.setDataRate(ADXL345_DATARATE_100HZ);
}

// Read data from accelerometer and filter noise
void readAccelerometerData() {
  Vector norm = accelerometer.readNormalize();

  x_accel_raw = norm.XAxis - 0.2;
  y_accel_raw = norm.YAxis;
  z_accel_raw = norm.ZAxis;

  x_medfilt = medfilt_X(x_accel_raw);
  y_medfilt = medfilt_Y(y_accel_raw);
  z_medfilt = medfilt_Z(z_accel_raw);
  accel = sqrt(pow(x_medfilt, 2) + pow(y_medfilt, 2) + pow(z_medfilt, 2));

  //Serial.print(minutes, 3); Serial.print(","); // Represent milliseconds passed
  Serial.print("X:"); Serial.print(x_medfilt, 2);
  Serial.print(" Y:"); Serial.print(y_medfilt, 2);
  Serial.print(" Z:"); Serial.print(z_medfilt, 2);
  Serial.print(" Accel:"); Serial.println(accel, 2);
  delay(10); // Send data every 10ms or at 100 Hz
}

void readGyroscopeData() {
  Vector norm = accelerometer.readNormalize();
  
  x_accel_raw = norm.XAxis - 0.2;
  y_accel_raw = norm.YAxis;
  z_accel_raw = norm.ZAxis;

  x_medfilt = medfilt_X(x_accel_raw);
  y_medfilt = medfilt_Y(y_accel_raw);
  z_medfilt = medfilt_Z(z_accel_raw);

  // Pitch and roll
  pitch = -(atan2(x_accel_raw, sqrt(pow(y_accel_raw, 2) + pow(z_accel_raw, 2))) * 180.0)/M_PI;
  roll  = (atan2(y_accel_raw, sqrt(pow(x_accel_raw, 2) + pow(z_accel_raw, 2))) * 180.0)/M_PI;

  // Filtered pitch and roll
  filt_pitch = -(atan2(x_medfilt, sqrt(pow(y_medfilt, 2) + pow(z_medfilt, 2))) * 180.0)/M_PI;
  filt_roll = (atan2(y_medfilt, sqrt(pow(x_medfilt, 2) + pow(z_medfilt, 2))) * 180.0)/M_PI;

  // Print tilt angle
  tilt = acos(z_medfilt / accel) * 180.0/M_PI;
  Serial.print("Tilt:"); Serial.println(tilt, 2);
}

// Read data from GPS module (from TinyGPS++ library)
void readGPSData() {
  while(gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude:"); Serial.print(latitude, 6);
      Serial.print(" Longitude:"); Serial.print(longitude, 6);
    }
  }
}

void sendEmergency() {
  sendSMS(numbers);
  makeCall(numbers);
}

// SMS sending function
void sendSMS(const char **numbers) {
  for(int i = 0; i < 3; i++) {
    sim800Serial.println("AT+CSQ");
    delay(1000);

    sim800Serial.println("AT");
    delay(1000);
    
    sim800Serial.println("AT+CMGF=1"); // Set SMS mode to text
    delay(1000);
    
    sim800Serial.print("AT+CMGS=\""); 
    sim800Serial.print(numbers[i]); 
    sim800Serial.println("\"");
    delay(1000);

    String latstr = String(latitude, 6);
    String lngstr = String(longitude, 6);

    String text = "EMERGENCY MESSAGE ALERT";
    text += "SEND HELP IMMEDIATELY to my current location: ";
    text += "Latitude= " + latstr + "\n";
    text += "Longitude= " + lngstr + "\n";
    text += "This is an automated message. Please do not reply. meow (^.v.^)/";

    sim800Serial.print(text);
    delay(500);
    sim800Serial.write(26); // End message with Ctrl+Z
    delay(1000);
  }
}

// Send call function
void makeCall(const char **numbers) {
  for(int i = 0; i < 3; i++) {
    sim800Serial.print("ATD");
    sim800Serial.print(numbers[i]);
    sim800Serial.println(";\r\n");

    Serial.println("Making call...");
    delay(8000);

    // End call
    sim800Serial.println(F("ATH"));
    delay(1000);
  }
}
