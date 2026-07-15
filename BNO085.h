#ifndef _BNO085_H
#define _BNO085_H
// Digital Compass 

//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BNO08x myIMU;

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
#define BNO08X_INT  3
//#define BNO08X_INT  -1
#define BNO08X_RST  4
//#define BNO08X_RST  -1


#define BNO08X_ADDR 0x4B  
#define CALIB_DATA_START 0x55  // Starting register for calibration offsets data
#define CALIB_DATA_LENGTH 22   // Length of calibration offsets data block

uint8_t calibData[CALIB_DATA_LENGTH];

struct CMPS14_calibration {
  unsigned int sys : 2;
  unsigned int gyro : 2;
  unsigned int accel : 2;
  unsigned int mag : 2;
};

byte _byteHigh;
byte _byteLow;

extern volatile  int bearing, pitch, roll;

//Get current sensor offsets 
void readIMUsensorOffsets() { 

  Wire.beginTransmission(BNO08X_ADDR);
  Wire.write(CALIB_DATA_START);
  Wire.endTransmission(false); // Send restart condition
  Wire.requestFrom(BNO08X_ADDR, CALIB_DATA_LENGTH);

  for (int i = 0; i < CALIB_DATA_LENGTH; i++) {
    if (Wire.available()) {
      calibData[i] = Wire.read();
    }
  }
}

//send locally saved calibration offsets back to IMU
void writeIMUsensorOffsets() { 
  Wire.beginTransmission(BNO08X_ADDR);
  Wire.write(CALIB_DATA_START);

  for (int i = 0; i < CALIB_DATA_LENGTH; i++) {
    Wire.write(calibData[i]);
  }
  Wire.endTransmission();
}


  // Here is where you define the sensor outputs you want to receive
#define REPORT_FREQ 50 //(milliseconds)
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableGeomagneticRotationVector(REPORT_FREQ) == true) {
    Serial.println(F("Geomagnetic rotation vector enabled"));
  } else {
    Serial.println(F("Could not enable rotation vector"));
  }
  if (myIMU.enableMagnetometer(REPORT_FREQ) == true) {
    Serial.println(F("Magnetometer enabled"));
  } else {
    Serial.println(F("Could not enable magnetometer"));
  }
  //enable gyroscope report
  if (myIMU.enableGyro(REPORT_FREQ) == true) {
    Serial.println(F("Gyroscope enabled"));
  } else {
    Serial.println(F("Could not enable gyroscope"));
  }
  //enable accelerometer report
  if (myIMU.enableAccelerometer(REPORT_FREQ) == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println(F("Could not enable accelerometer"));
  }
}

 #endif _BNO085_H