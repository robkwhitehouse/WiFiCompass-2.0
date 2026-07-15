
/*
 - Electronic Boat Compass
 - RK Whitehouse July 2026
 - version 2.0 adds support for the Sparkfun BNO085 IMU module
 - version 2.0D - sensor report queueing bug fixed
*/

/* Functionality
* The BNO085 sensor module provides an absolute orientation vector
 *  This software coverts the BNO085 output to NMEA 0183 "HDM" message format
 * and transmits this over WiFi
 * The ESP32 provides a WiFi Access Point. SSID = WiFiCompass (no password)
 * This has a Telnet server On port 23- the HDM messages are transmitted 5 times per second
 * There is also an http server running on port 80 - The default (root) node serves a single page web-app 
 * point your browser at 192.168.4.1/sensorCalibration.html to run the calibration web app
 * that makes callbacks to a REST API to perform calibration and configuration 
 *
 * Two different IMU modules have been used, both use the BOsch BNO-085 sensor. These are;
 * the Devantech CMPS14 and the Sparkfun BNO-08x module. Both offer similar functionality. 
 * The CMPS14 has a much simpler interface. The Sparkfun module is much cheaper and more flexible but is much more complex to use. 
 */

/* Software designs

 *  
 *  There are two sets of methods 
 *  
 *  1. Foreground tasks - i.e. user interface tasks
 *  2. Background tasks - run at specific intervals
 *
 *  The background tasks are run by RTOS Tasks without any
 *  direct user interaction. The RTOS scheduler is pre-emptive so any shared variables probably need to be protected by semaphores
 *  
 *  The foreground tasks are called from the main loop()
 *  
 *
 * Communication between background and foreground tasks is via a set of global static
 * objects and variables
 * 
 */

/*
 * Hardware design
 * 
 * Runs on an ESP-32 module with a CMPS14 or Sparkfun BNO085 sensor module attached via
 * I2C. An RGB LED provides status feedback and is connected on pins 6,5 & 2 (could be changed)
 * 
 * This version has been tested on an ESP32 C3 Super-mini but it should work with any ESP32 module
 * Other Arduino controllers might work if they support WiFi
 */


#define BNO085 //remove this line for CMPS14 support. The CMPS14 code has previously worked but has not been compiled or tested for a while.

/* Imported libraries */
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h> 
#include <WiFi.h>
#include <WiFiAP.h>
#include <FS.h>
#include <LittleFS.h>
#include <WebServer.h>

/* Local libs */
#ifdef BNO085
#include "BNO085.h"
#else
#include "Cmps14.h"
#endif

#include "NMEA.hpp"
#include "calibration.h"
#include "webCalibration.h"
#include "TricolourLED.hpp"


#define VERSION "2.0D"
#define TELNET_PORT 23       //Compass heading is output on this port
#define CONFIG_PORT 1024
#define WWW_PORT 80
#define MAX_CLIENTS 4
#define HOSTNAME "WiFiCompass"
#define FSYS LittleFS   //Or could be SPIFFS


//Pins 3 & 4 are reserved for BNO085

//TriColour LED output pins. Can be changed
#define RED_PIN 6
#define GREEN_PIN 5
#define BLUE_PIN 2

//I2C pins - not the default - you could chnge these if wanted
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9


#define IMU_SAMPLERATE_DELAY_MS 100 //Compass chip is sampled 10 times per second

//Pre-Declare background RTOS task methods
void output();
void updateHeading();
void turnOff();

/* Declare Global Singleton Objects */

volatile bool debug = false;

// Non-volatile settings - will be restored on power-up
Preferences settings;

//Handles for the various RTOS tasks. Will be populated later
TaskHandle_t outputTask, updateHeadingTask;


unsigned short  boatHeading = 0; //Heading seen on boat compass, calculated from sensorHeading + boatCompassOffset
unsigned short sensorHeading = 0; //Heading read from IMU
byte calibration = 0; //CMPS14 calibration level
//Calibration levels ( 0 to 3 )
volatile uint8_t magAccuracy,accelAccuracy,gyroAccuracy,headingAccuracy;
volatile int bearing, roll, pitch;

//Set up some storage for the NMEA output messages
HSCmessage hsc;
HDMmessage hdm;

//Create WiFi network object pointers
const char *ssid = "WiFiCompass v20D";  //WiFi network name
WiFiServer *telnetServer = NULL;
WiFiServer configServer(CONFIG_PORT);
//WiFiClient **telnetClients = {NULL}; //Array of WiFi Clients
WiFiClient* telnetClients[MAX_CLIENTS];
WiFiClient configClient;  //Configuration via Telnet - redundant
WebServer webServer(WWW_PORT);

// Create TriColour LED object
TricolourLED myLED(RED_PIN,GREEN_PIN,BLUE_PIN,TricolourLED::COMMON_CATHODE);

extern int16_t compassCard[]; //declared in calibration.h. has compass card offsets for each degree

void setup() {

  Serial.begin(115200);
  delay(200);
  Serial.println("WiFi Compass starting up");
  //setup I2C Bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

#ifdef BNO085
  myIMU.begin(); //Initialise SH2 Interface
  delay(100);
  disableCalibration();  //Stop the IMU from automatic automatic recalibrating
  delay(100);
  setReports(); //Need to tell BNO085 which reports to send
  delay(100);
  Serial.println("BNO085 startup OK");
#endif

  //Setup RGB LED and associated blinker
  myLED.begin();
  myLED.setColour(TricolourLED::AMBER);
  myLED.setState(TricolourLED::ON);





  
// Setup filesystem in Flash memory
//  if (!SPIFFS.begin(true)) 
  if (!FSYS.begin(true)) // "true" - format FSYS if necessary"
    Serial.println("Mounting File System failed");

  Serial.print("WiFiCompass. Version ");
  Serial.println(VERSION);

  settings.begin("compass",false); //Open (or create) settings namespace "compass" in read-write mode
  if ( settings.isKey("compassCard") ) {//We have an existing compassCard in NVRAM
    Serial.println("Loading compass card from flash memory");
    settings.getBytes("compassCard",&compassCard,sizeof(compassCard));
  } else Serial.println("no compass card in flash");
  if (settings.isKey("sensorOffsets")) {
    settings.getBytes("sensorOffsets",&calibData,sizeof(calibData)); //get sensor offsets from Flash
    writeIMUsensorOffsets(); //restore previous calibrated offsets back to the IMU
  } else Serial.println("no sensor offsets in flash");
  
  //Startup the Wifi access point (no password)
  if (!WiFi.softAP(ssid)) Serial.println("Could not start WiFi Access Point") ;
  WiFi.setHostname(HOSTNAME);
  //Initalise WiFiClient Array
  for(int i = 0; i < MAX_CLIENTS; i++)
  {
    telnetClients[i] = NULL;
  }
  //This is the server that outputs the NMEA messages
  telnetServer = new WiFiServer(TELNET_PORT);
  telnetServer->begin();

  
//This server was used for Serial based config, calibration  & debug 
//now replaced by webapp
// configServer.begin();

  webServerSetup(); //Setup the webserver -used for calibration
 
  IPAddress myAddr = WiFi.softAPIP();
  Serial.print("IP Address =");
  Serial.println(myAddr);

  delay(1000);

  //Start the RTOS background tasks
  xTaskCreate(output, "Output", 4000, NULL, 1, &outputTask); 
  xTaskCreate(updateHeading, "updateHDG", 4000, NULL, 1, &updateHeadingTask);

  myLED.setColour(TricolourLED::GREEN);
  myLED.setState(TricolourLED::ON);

}

unsigned loopCounter;
long loopStart, totalLoopTime=0;

void loop() {
  struct CMPS14_calibration cal;
  
  loopStart = micros();

  sh2_service();  //Give the sensor hub bus a kick

  WiFiClient tempClient = telnetServer->available();   // listen for new incoming client connections

  if (tempClient) {                             // if you get a client,
    Serial.println("New NMEA client.");           // print a message out the serial port
    for (int i=0; i<MAX_CLIENTS; i++ ) {   //Add it to the Client array
       if ( telnetClients[i] == NULL ) {
          WiFiClient* client = new WiFiClient(tempClient);
          telnetClients[i] = client;
          break;
       }
       Serial.println("Ran out of WiFiClient slots.");
    }
  }
  //Configuration can also be done via Telnet (different port)
  //But this method is now deprecated - please use a web browser
  //configClient = configServer.available();
  //if (configClient) {
  //  if (configClient.connected()) {
  //    Serial.println("Config Client detected.");
  //    calibrationMenu();
  //  }
  //}

  webServer.handleClient(); //handle incoming http requests (calibration etc.)
  
  
  //Reflect sensor status in the LED
  
   //Check if sensor is OK
  if (sensorHeading == 0 ) { //probably a fault
      myLED.setColour(TricolourLED::RED);
      myLED.setState(TricolourLED::BLINKING);
  } else {  
  //Check if sensor calibration is OK
    cal = (struct CMPS14_calibration)calibration;
    if (cal.mag < 2) {
      myLED.setColour(TricolourLED::AMBER);
      myLED.setState(TricolourLED::BLINKING);
    } else {
     //Assume all OK - set LED to GREEN
      myLED.setColour(TricolourLED::GREEN);
      myLED.setState(TricolourLED::BLINKING);
    }
  } 
 
//Check if debug mode required
  if(Serial.available()) {
    int c = Serial.read();
    if ( c == 'd') {
      debug = !debug;
      Serial.println("Debugging toggled");
    } else
    if ( c == 'c') calibrationMenu(); //Offer calibration commands via Serial port. Deprecated. should use web interface
  }

  totalLoopTime += (micros() - loopStart);
  loopCounter++;
}//end of foreground loop

//Definition of background RTOS tasks

//Output the heading as an NMEA message over WiFi (RTOS Task)
void output(void * pvParameters) {
  char buff[128];
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = 200; //Run every 200ms
  byte calibration;


  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount ();
  
  for (;;) {
     if (Serial && debug) {

#ifdef BNO085
          sprintf(buff, "Sensor: %03d deg. Boat: %03d Roll: %03d Pitch: %03d Cal: G:%02d A:%02d M:%02d H:%02d\n", sensorHeading, boatHeading, roll, pitch, gyroAccuracy,accelAccuracy,magAccuracy,headingAccuracy);
#else
          calibration = getCalibration();
          byte sys = (calibration & 0b11000000) >> 6;
          byte gyroAccuracy = (calibration & 0b00110000) >> 4;
          byte accelAccuracy = (calibration & 0b00001100) >> 2;
          byte magAccuracy = (calibration & 0b00000011);
       sprintf(buff, "Sensor: %03d deg. Boat: %03d Roll: %03d Pitch: %03d Calib. G:%02d A:%02d M:%02d\n", sensorHeading, boatHeading, roll, pitch, gyroAccuracy,accelAccuracy,magAccuracy);
#endif
       Serial.print(buff);  
      } //end of debug output

     //Update the NMEA message with the current boat heading 
     hdm.update(boatHeading);
     //This is the main business - transmit the heading as an NMEA message over Telnet (WiFi) to anybody that is interested   
     for ( int i=0; i<MAX_CLIENTS; i++ ) {
       if ( telnetClients[i] != NULL ) 
         telnetClients[i]->println(hdm.msgString);
     }
     vTaskDelayUntil( &xLastWakeTime, xPeriod );
  }//end of perpetual for loop
}


//Update the heading from the IMU (RTOS task)
void updateHeading(void * pvParameters) {         
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = 20; //Run every 20ms. Note this needs to be more frequent than the report frequency otherwise the queue overflows
  uint8_t sensorEventType;
  float yaw;

  for (;;) {
#ifdef BNO085
    if (myIMU.wasReset()) {
      Serial.print("Sensor was reset.");
      setReports();
    }

  // Has a new event come in on the Sensor Hub Bus?
    while (myIMU.getSensorEvent() == true) {

      sensorEventType = myIMU.getSensorEventID();

      switch(sensorEventType) {
        case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR:
          roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
          pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
          yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
          yaw = 360 - yaw; //yaw comes out of IMU as +/- 180 deg. Not what we want for a compass heading
          if (yaw > 360) yaw -= 360;
          sensorHeading = yaw;
          headingAccuracy= myIMU.getQuatAccuracy();
          break;
        case SENSOR_REPORTID_MAGNETIC_FIELD:
          magAccuracy = myIMU.getMagAccuracy();
          break; 
        case SENSOR_REPORTID_ACCELEROMETER:
          accelAccuracy = myIMU.getAccelAccuracy();
          break;
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
          gyroAccuracy = myIMU.getGyroAccuracy();
          break;
        default: 
          Serial.printf("Unexpected sensor event received %02x\n", sensorEventType);
      } //end of switch 
    } // end of sensor event processing
#else //(sensor is CMPS14)
  //get the raw CMPS14 output
  sensorHeading = getBearing();
#endif

    //Apply compass card offset
    boatHeading = MOD360(sensorHeading - compassCard[sensorHeading]);
    calibration = getCalibration(); //This just does format conversion

    vTaskDelayUntil( &xLastWakeTime, xPeriod );
  } // end of for perpetual for loop
} //end of updateHeading task definition


