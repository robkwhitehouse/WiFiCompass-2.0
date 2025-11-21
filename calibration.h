#ifndef _CALIB_H
#define _CALIB_H
#ifdef BNO085
#include "BNO085.h"
#define CHANNEL_COMMAND 0x01
#define FEATURE_REPORT_ID_ROTATION_VECTOR 0x05
#define SET_FEATURE_COMMAND 0xFD

#else  //CMPS14

#include "Cmps14.h"
#define _i2cAddress         0x60
#define calibrationQuality  0x1E
#endif

#define MOD360(x) (((x)%360 + 360) % 360)



// https://stackoverflow.com/questions/111928 (nice trick)
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

// Timer
unsigned long mytime;

// Character array
char Message[256];

// Compass card - array holds offsets to boat compass
// Allows CMPS14 to be mounted in any orientation

int16_t compassCard[360];

extern volatile uint8_t magAccuracy,accelAccuracy,headingAccuracy, gyroAccuracy;

extern WiFiClient configClient;
extern Preferences settings;
extern WebServer webServer;

//local function prototypes
byte getVersion();
void CalibrationQuality();
void writeToCMPS14(byte n);
void printMenu();
void Countdown(unsigned);
void printTerm(char *);
void printTerm(byte);
void endTransmission();
void createCompassCard();
void resetCompassCard();
void displayCompassCard();
void saveCompassCard();
byte getCalibration();
void saveCalibration();
void disableCalibration();
void calcOffsets(int, int, int, int);
void enableMagnetometerCalib();
void enableAccelerometerCalib();
void enableGyroCalib();
void configurePeriodicDCDSave(bool);
#ifdef BNO085
void sendCommandToBNO085(uint8_t, uint8_t *, uint8_t );
#endif
void configureDynamicCalib(bool);
void factoryReset();
void getCalConfig();

void calibrationBegin() {
  printTerm("----------------------\n");
  printTerm("   IMU Calibration\n"); 
  printTerm("----------------------\n");

  printTerm("CMPS 14 software version v");
  printTerm(getVersion());
  printTerm("\n");
  CalibrationQuality();

}

void calibrationMenu() {
  byte a, junk;

  printMenu();
  do {
    a = 0;
    //Check serial port
    if (Serial.available() > 0 )  {
    // Read User input from serial port
      a = Serial.read();
      while(Serial.available() > 0) junk = Serial.read(); //ditch any trailing EOL chars
    
      // Send some data
      switch (a){

      case 'm':
        printTerm("Magnetometer...\n");
        enableMagnetometerCalib();
        printTerm("Rotate the IMU back and forth by 180 deg in each axis\n");
        Countdown(40);
        CalibrationQuality();
      break;

      case 'a':
        printTerm("Accelerometer...\n");
        enableAccelerometerCalib();
        printTerm("Rotate so that each of the 6 sides is horizontal and keep steady for a second\n");
        Countdown(40);
        CalibrationQuality();
      break;

      case 'g':
         printTerm("Gyro... Keep the IMU stationary for 20seconds\n");
         printTerm("Note that there is currently a bug in the IMU firmware. The gyro always reports calibration as \"0\"\n");
         enableGyroCalib();
         Countdown(20);
         CalibrationQuality();
      break;

      case 'p':
        printTerm("Enable periodic automatic save of calibration data\n");
        configurePeriodicDCDSave(true);
        CalibrationQuality();
      break;

      case 'x':
        printTerm("Disable calibration auto save\n");
        configurePeriodicDCDSave(false);
      break;

      case 'E':
        printTerm("Enable dynamic calibration\n");
        configureDynamicCalib(true);
      break;

      case 'D':
        printTerm("Disable dynamic calibration\n");
        configureDynamicCalib(false);
      break;
      }
      
      // Show calibration quality
      if (a == 'c'){
        CalibrationQuality();
      }

      if (a == 'C') getCalConfig();

      // Store the calibration
      if (a == 's'){
        printTerm("Saving calibration data\n");
        saveCalibration();
      }


    // Reset the calibration
      if (a == 'e'){
        factoryReset(); 

        // Update the User
        printTerm("Saved calibration erased, factory defaults apply\n");
        delay(500);
      }

      //Other (non-I2C commands)

      switch(a) {
        //Print the command menu
        case'h':
        case '?':
          printMenu(); break;
         //Align with boat compass (create a compass card)
        case 'b': createCompassCard(); break;
        case 'd': displayCompassCard(); break;
          //Zero (reset) the compassCard
        case 'z': resetCompassCard(); break;
        //Save the current compassCard to NV memory
        case'n': saveCompassCard(); break;
        //Restart the ESP32
        case 'r': ESP.restart(); 
        //Will never get here but just in case ...
        break;
      }
    }
  } while( a != 'q'); 
}



void CalibrationQuality(){

  byte calibration = getCalibration();
  sprintf(Message,"Calibration " BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(calibration));
  printTerm(Message);
}


void Countdown(unsigned period){

  int i;
  for (i=period; i>0; i--){

    printTerm(i);
    printTerm(" ");
    delay(1000);
  }

  printTerm("OK\n");

}

void printTerm(char *mesg) {
  if (Serial) Serial.print(mesg);
//  if (configClient) configClient.print(mesg);
}

void printTerm(byte mesg) {
  if (Serial) Serial.print(mesg);
//  if (configClient) configClient.print(mesg);
}

void printMenu() {
    
  printTerm("\n");
  printTerm("Enter command;\n");
  printTerm(" - 'h' to print this menu\n");
  printTerm(" - 'c' to see current calibration levels\n");
  printTerm(" - 'C' to get current calibration configuration\n");
  printTerm(" - 'E' to enable dynamic calibration - all sensors\n");
  printTerm(" - 'D' to disable dynamic calibration - all sensors\n");
  printTerm(" - 'g' to calibrate the gyroscope\n");
  printTerm(" - 'a' to calibrate accelerometer\n");
  printTerm(" - 'm' to calibrate magnetometer\n");
  printTerm(" - 's' to save current CMPS calibration\n");
  printTerm(" - 'e' to erase the saved CMPS calibration\n");
  printTerm(" - 'p' to enable periodic auto-save\n");
  printTerm(" - 'x' to disable periodic auto-save\n");
  printTerm(" - 'b' to generate a boat compass card\n");
  printTerm(" - 'd' to display the compass card\n");
  printTerm(" - 'z' to zero (erase) the compass card\n");
  printTerm(" - 'n' to save compass card to ESP32 Non-volatile memory\n");
  printTerm(" - 'r' to reboot the system\n");
  printTerm(" - 'q' to quit settings mode\n");
  printTerm("->? ");

}

/*
 * generate a "compass card" for the CMPS14
 * Allows the CMPS to be mounted in any orientation
 * Generates a mapping table, mapping readings from the CMPS into 
 * real world magnetic compass bearings
 */

//procedure to create the compass card
void createCompassCard() {

  int junk, north, east, south, west;
  float delta;
  char buff[128];  
  unsigned index;

  //Clear input buffers

  //Get sensor heading for north
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due North. Hit enter when the boat compass reads 000 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  north = bearing;
  sprintf(buff,"CMPS reading for North is %03d degrees\n\n",north);
  printTerm(buff);

  //Get sensor heading for east
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due East. Hit enter when the boat compass reads 090 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  east = bearing;
  sprintf(buff,"CMPS reading for East is %03d degrees\n\n", east);
  printTerm(buff);

  //Get sensor heading for south
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due South. Hit enter when the boat compass reads 180 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  south = bearing;
  sprintf(buff,"CMPS reading for South is %03d degrees\n\n",south);
  printTerm(buff);

  //Get sensor heading for west
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due West. Hit enter when the boat compass reads 270 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  west = bearing;
  sprintf(buff,"CMPS reading for West is %03d degrees\n\n", west);
  printTerm(buff);
  
//now calculate the mapping for every possible degree

  calcOffsets(north,east,south,west);
}

//The procedure to calculate and store (in compassCard) the offsets for each degree
//The inputs contain the sensor readings for each cardinal

void calcOffsets(int north, int east, int south, int west)
{
  char buff[256];
  float delta;

//NE quandrant
  //Calculate the average difference per degree. This will likely vary for each quadrant
  unsigned Qsize = MOD360(east-north); //Might not be 90 due to sensor eccentricities etc.
  sprintf(buff,"Q1 size is %d\n",Qsize);
  printTerm(buff);
  delta = (Qsize / 90.0)-1; //force float arithmetic
  sprintf(buff,"Q1 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(north+i);
    compassCard[index] = MOD360((int)(north + (int)round(i*delta)));
  }

//SE quadrant
  //Calculate the average difference per degree. This will likely vary for each quadrant
  Qsize = MOD360(south-east); //Might not be 90 
  delta = (Qsize / 90.0)-1;
  sprintf(buff,"Q2 size is %d\n",Qsize);
  printTerm(buff);
  sprintf(buff,"Q2 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(east+i);
    compassCard[index] = MOD360((int)(east-90 + (int)round(i*delta)));
  }

//SW quadrant
  //Calculate the average difference per degree. This will vary for each quadrant
  Qsize = MOD360(west-south); //Might not be 90 
  delta = (Qsize / 90.0)-1; //force float arithmetic
  sprintf(buff,"Q3 size is %d\n",Qsize);
  printTerm(buff);
  sprintf(buff,"Q3 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(south+i);
    compassCard[index] = MOD360((int)(south-180 + (int)round(i*delta)));
  }
//NW quadrant
  //Calculate the average difference per degree. This will vary for each quadrant
  Qsize = MOD360(north-west); //Might not be 90 
  sprintf(buff,"Q4 size is %d\n",Qsize);
  printTerm(buff);
  delta = (Qsize / 90.0)-1; //force float arithmetic
  sprintf(buff,"Q4 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(west+i);
    compassCard[index] = MOD360((int)(west-270 + (int)round(i*delta)));
  }
}

//Procedure to zero the compass card
void resetCompassCard() {
  for(int i=0; i<360; i++) compassCard[i] = 0;
}

//Procedure to save the compass card to ESP32 NVRAM - this will be automatically restored
//on power up
void saveCompassCard() {
  settings.putBytes("compassCard", compassCard, sizeof(compassCard));
  printTerm("compassCard saved\n");
}

//Display the compass  card
void displayCompassCard() {
  char buff[128];
  printTerm("compassCard;\n");
  for (int i = 0; i<360; i++) {
    sprintf(buff,"compassCard[%d] = %d\n",i,compassCard[i]);
    printTerm(buff);
  }  
}

void disableCalibration() {
  configureDynamicCalib(false);
  configurePeriodicDCDSave(false);
}

#ifdef BNO085
//***************************
//BNO085 specific functions
//***************************

void factoryReset() {
  Serial.println("Performing factory reset.");
  sh2_clearDcdAndReset();
}   

byte getCalibration() {
  return((headingAccuracy << 6) | (gyroAccuracy << 4) | (accelAccuracy << 2) | magAccuracy );
}

void enableMagnetometerCalib() {
  //enable dynamic calibration, magnetometer
  if (myIMU.setCalibrationConfig(SH2_CAL_MAG) != true)  
    Serial.println("Could not enable magnetometr calibration.");
}

void enableAccelerometerCalib() {
  //enable dynamic calibration, accelerometer
  if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL) != true)  
    Serial.println(F("Could not enable accelerometer calibration.\n")); 
}

void enableGyroCalib() {
  if (myIMU.setCalibrationConfig(SH2_CAL_GYRO) != true)  
    Serial.println(F("Could not enable gyro calibration.\n")); 

}

void configureDynamicCalib(bool enable) {

  uint8_t config;
  if (enable) config = SH2_CAL_GYRO || SH2_CAL_ACCEL || SH2_CAL_MAG;
  else config = 0x00;

  //configure dynamic calibration
  if (myIMU.setCalibrationConfig(config) != true)  // configure all three sensors
    Serial.println(F("Could not configure dynamic calibration."));
}

void sendCommandToBNO085(uint8_t command, uint8_t *payload, uint8_t length) {
  Wire.beginTransmission(BNO08X_ADDR);
  Wire.write(command);        // Command ID
  for (int i = 0; i < length; i++) {
    Wire.write(payload[i]);   // Write payload
  }
  Wire.endTransmission();
}

void configurePeriodicDCDSave(bool enable){
  uint8_t payload[17] = {0};

  // Set Feature Report ID for Rotation Vector
  payload[0] = FEATURE_REPORT_ID_ROTATION_VECTOR;

  // Report interval in microseconds (100Hz = 10,000 us)
  uint32_t interval = 10000;
  payload[1] = interval & 0xFF;
  payload[2] = (interval >> 8) & 0xFF;
  payload[3] = (interval >> 16) & 0xFF;
  payload[4] = (interval >> 24) & 0xFF;

  // Batch interval (optional)
  uint32_t batchInterval = 0;
  payload[5] = batchInterval & 0xFF;
  payload[6] = (batchInterval >> 8) & 0xFF;
  payload[7] = (batchInterval >> 16) & 0xFF;
  payload[8] = (batchInterval >> 24) & 0xFF;

  // Sensor-specific config: Feature flags
  // Bit 0 = Sensor-specific config enable
  // Bit 1 = Dynamic calibration enable (set to 0 to disable)
  if (enable)
    payload[9] = 0x01; // Only Bit 0 enabled (Bit 1 = 0 disables dynamic cal)
  else
    payload[9] = 0x00; 

  // Send command
  sendCommandToBNO085(SET_FEATURE_COMMAND, payload, sizeof(payload));
  Serial.println("Dynamic calibration disabled.");

}

void saveCalibration() {
  // Saves the current dynamic calibration data (DCD) to memory
  // Note, by default the BNO08X stores updated Dynamic Calibration Data (DCD) to RAM 
  // frequently (every 5 seconds), so this command is only necessary
  // if dynamic calibration is disabled
  if (myIMU.saveCalibration() != true) 
    Serial.println(F("Calibration data was not saved"));
  else
    printTerm("Calibration profile saved\n");
}

void getCalConfig(){
  uint8_t sensors;
  char buff[64];
  
  if (sh2_getCalConfig(&sensors) < 0)
    printTerm("Error - could not read calibration config\n");
  else {
    sprintf(buff,"Sensor calibration config = %02x \n",sensors);
    printTerm(buff);
  }
}
#else 

//***************************
//CMPS 14 specific functions
//***************************

void writeToCMPS14(byte n){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
    
  // Want the Command Register
  Wire.write(byte(0x00));

  // Send some data    
  Wire.write(n);
  
  endTransmission();
}

//Set CMPS14 into configuration mode
void initCMPSconfig() {
  // Configuation bytes
        writeToCMPS14(byte(0x98));
        writeToCMPS14(byte(0x95));
        writeToCMPS14(byte(0x99));

        // Begin communication with CMPS14
        Wire.beginTransmission(_i2cAddress);

        // Want the Command Register
        Wire.write(byte(0x00));
}


void factoryReset() {
        writeToCMPS14(byte(0xE0));
        writeToCMPS14(byte(0xE5));
        writeToCMPS14(byte(0xE2));    
}

byte getCalibration() {

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
  delay(20);
  // Tell register you want some data
  Wire.write(calibrationQuality);
  endTransmission();

  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(_i2cAddress, 1);

  // Timed out so return
  if (nReceived != 1) {
    sprintf(Message,"calibrationQuality - invalid return from Wire.requestFrom() == %d\n", nReceived);
    printTerm(Message);
    return(0);
  }
  
  // Read the values
  return(Wire.read());
}

byte getVersion(){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
    
  // Want the Command Register
  Wire.write(byte(0x00));

  // Send some data    
  Wire.write(byte(0x11));
  
  endTransmission();

  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(_i2cAddress, 1);

  // Read the values
  byte ver = Wire.read();

  return ver;
}


void endTransmission() {
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0)
    printTerm("communication error\n")
       ;
  // Wait 100ms
  delay(100);
}

//Enable/disable dynamic calibration - all sensors
void configureDynamicCalib(bool enable){
  initCMPSconfig();
  if (enable) Wire.write(byte(B10000111));
  else Wire.write(byte(B10000000));
  endTransmission();
}

void configurePeriodicDCDSave(bool enable){
  initCMPSconfig();
  if (enable)
    Wire.write(byte(B10010000));
  else
    Wire.write(byte(B10000000));
  endTransmission();
}


void enableMagnetometerCalib() {
  initCMPSconfig();
  Wire.write(byte(B10000001));
  endTransmission();
}

void enableAccelerometerCalib() {
  initCMPSconfig();
  Wire.write(byte(B10000010));
  endTransmission();
}

void enableGyroCalib() {
  initCMPSconfig();
  Wire.write(byte(B10000100));
  endTransmission();
}

void saveCalibration() {

  writeToCMPS14(byte(0xF0));
  writeToCMPS14(byte(0xF5));
  writeToCMPS14(byte(0xF6));      

  // Update the User
  printTerm("Calibration profile saved\n");
}
#endif

#endif _CALIB_H