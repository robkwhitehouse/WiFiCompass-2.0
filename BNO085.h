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


  struct CMPS14_calibration {
    unsigned int sys : 2;
    unsigned int gyro : 2;
    unsigned int accel : 2;
    unsigned int mag : 2;
  };

  byte _byteHigh;
  byte _byteLow;

extern volatile  int bearing, pitch, roll;

  // Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableGeomagneticRotationVector() == true) {
    Serial.println(F("Geomagnetic rotation vector enabled"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  if (myIMU.enableMagnetometer(1) == true) {
    Serial.println(F("Magnetometer enabled"));
  } else {
    Serial.println("Could not enable magnetometer");
  }
  //enable gyroscope report
  if (myIMU.enableGyro(1) == true) {
    Serial.println(F("Gyroscope enabled"));
  } else {
    Serial.println("Could not enable gyroscope");
  }
  //enable accelerometer report
  if (myIMU.enableAccelerometer(1) == true) {
    Serial.println(F("Accelerometer enabled"));
  } else {
    Serial.println("Could not enable accelerometer");
  }
}

/*
  float magnetX = 0;
  float magnetY = 0;
  float magnetZ = 0;

  float accelX = 0;
  float accelY = 0;
  float accelZ = 0;
  // The acceleration along the X-axis, presented in mg 
  // See BNO080_Datasheet_v1.3 page 21
  float accelScale = 9.80592991914f/1000.f; // 1 m/s^2
  
  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;
  // 16bit signed integer 32,768
  // Max 2000 degrees per second - page 6
  float gyroScale = 1.0f/16.f; // 1 Dps
*/

/*
int16_t getBearing()
{

  return bearing;
}

int getRoll()
{
   return roll;
}

int getPitch()
{
   return pitch;
}
*/
 #endif