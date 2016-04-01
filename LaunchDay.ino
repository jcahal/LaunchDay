/****************************************************
 * Phoenix College Acsend Launch Day Instructions Spring 2016
 *
 * This file contains the code for collecting data from:
 * 
 * +(IMU) - Adafruit 10-DOF IMU Breakout
 * 
 * +(MQ4) - Sparkfun LPG Sensor - 
 * +(MQ6) - Sparkfun Methane Sensor -
 * +(MQ7) - Sparkfun Carbon Monoxide Sensor -
 * 
 * +(Barometer) - SparkFun Barometric Pressure Sensor Breakout - BMP180
 * +(GPS) - Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3
 * 
 * Authors: Phoenix College Acsend Team 2015 - 2016
 * 
 * Version 1.3.0.s
 * 
 * TODO's: 
 *  Test altDetect
 *  Test logging
 *  IMU commented content?
 *  Wire conflicts?
 *  
 *   
 ***************************************************/

/////////////////////////////////////////////////////////
//Includes, (Up to date libraries folder can be downloaded at: https://github.com/PC-Ascend-Team/libraries)
#include <Wire.h>                //IMU, Borometer, RB & Motor Driver Comm.

#include <Adafruit_Sensor.h>   //General Adafruit Sensor Library

#include <Adafruit_10DOF.h>    //IMU
#include <Adafruit_LSM303_U.h> //IMU
#include <Adafruit_L3GD20_U.h> //IMU
#include <Adafruit_BMP085_U.h> //IMU
#include <Adafruit_Simple_AHRS.h> //IMU - AHRS conversions

#include <SFE_BMP180.h>        //Barometer

#include <Adafruit_GPS.h>      //GPS
#include <SoftwareSerial.h>    //GPS

//IMU Definitions
/////////////////////////////////////////////////////////
//Assign a unique ID to the sensors
Adafruit_LSM303_Accel_Unified A   = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   M   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       B   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       G   = Adafruit_L3GD20_Unified(20);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&A, &M);

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// GAS Definitions
/////////////////////////////////////////////////////////
//Hardware pin definitions (Analog input)
int mq4 = A1,
    mq6 = A2,
    mq7 = A3;     //Initialize the Gas Sensor variables
 
int mq4_ppm,
    mq6_ppm,
    mq7_ppm;      //Initialize the Gas Sensor ppm values

//Barometer Definitions
/////////////////////////////////////////////////////////
SFE_BMP180 pressure;
#define ALTITUDE 337.1088 // Altitude of Phoenix in meters


//GPS Definitions
////////////////////////////////////////////////////////
SoftwareSerial mySerial(3, 2); // TX - D3, RX - D2
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false // we do NOT want to echo raw GPS data to serial
boolean usingInterrupt = false;


//Phoenix College Acsend Team variables
////////////////////////////////////////////////////////
int state = 0;                               // for state machine switch statement
int t = 0;                                    // time keeper (seconds)
String message = "";                         // sent by wire to RockBLOCK
int altBreakPoint[5] = {0, 0, 0, 0, 0};      // alts that trigger gas sample
float gpsAlt[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // used to average alts 
int offset = 0;                              // offset for collecting alt data points before averaging
int abpOffset = 0;                           // for iterating through altBreakPoint
float oldAlt, alt;                           // used in altDetect()
boolean falling = false;                     // balloon burst flag
boolean forOldAlt = true;                    // flag for switching between assigning to oldAlt and alt
boolean checkAlt = false;                    // flag when oldAlt and alt have been assigned, time to altDetect()
char sampleCode = '\0';                      // tells the MotorDiver what motor to drive
boolean sampled = false;                     // flags a sample ahs been completed
char rbStatus = false;                       // if RockBLOCK has ran into some error
int rbTransmissions, rbRecives;              // the number of messages the RB has recived and transmitted

//LEDs
int LD0 = 13; // D13, sysLED
int heater = 9; // D9


//Funcions Prototypes
 /////////////////////////////////////////////////////////

 // int averageAnalogRead(int pinToRead) - Takes an average of readings on a given pin. Returns the average
 int averageAnalogRead(int pinToRead);

 // void printError(byte error) - Used by the Luminosity sensor to display errors
 void printError(byte error);

 // void useInterrupt(boolean v) - Used by the GPS
 void useInterrupt(boolean);

 // SIGNAL(TIMER0_COMPA_vect) - Used by the GPS
 SIGNAL(TIMER0_COMPA_vect);

 // void altDetect(float oldAlt, float alt, int altBreakPoint[]) detects the apex and fills the breakpoint
void altDetect(float oldAlt, float alt, int altBreakPoint[]);

 //SETUP
 /////////////////////////////////////////////////////////
 void setup(void){
  Serial.begin(115200); // begin serial conncetion, determined by recomended baud rate for GPS

 //IMU SETUP
 //////////////////////////////////////////////////////
 //Initialise the sensors & check connections
 if(!A.begin()){
   Serial.println(F("No LSM303 detected."));
    // light LED code
 }
 if(!M.begin()){
   Serial.println(F("No LSM303 detected."));
    // light LED code
 }
 if(!B.begin()){
   Serial.print(F("No BMP085 detected."));
    // light LED code
 }
 if(!G.begin()){
   Serial.print(F("No L3GD20 detected."));
    // light LED code
 }
 
 sensor_t sensor;
 // Define sensors
 A.getSensor(&sensor);
 G.getSensor(&sensor);
 M.getSensor(&sensor);
 B.getSensor(&sensor);

 M.enableAutoRange(true); // have mag use auto range

 //BAROMETER SETUP
 /////////////////////////////////////////////////////////
 pressure.begin();
   
 // For error checks
 // Initialize the sensor (it is important to get calibration values stored on the device).
 if (pressure.begin()) {
   // Print nothing upon success
 }
 else
 {
   // light LED code
 }

 //GAS SENSOR SETUP
 //////////////////////////////////////////////////////
 pinMode(1, INPUT);
 pinMode(2, INPUT);
 pinMode(3, INPUT); 

 //GPS SETUP
 //////////////////////////////////////////////////////
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
 GPS.sendCommand(PGCMD_ANTENNA);
 useInterrupt(true);

 //Phoenix College Acsend Team SETUP
 //////////////////////////////////////////////////////
 //pinModes
 pinMode(LD0, OUTPUT);
 pinMode(heater, OUTPUT);
 Wire.begin(); // begin for Master
   
 //Output Header
 //////////////////////////////////////////////////////
 // Header: roll,pitch,yaw,IMUP,IMUT,IMUA,BMP,BMPT,BMPA,LPG,CH4,CO,H:M:S.ms,DD/MM/20YY,Fix,FixQ,Lat,Long,LatD,LongD,Speed,Angle,GAlt,Sats,rbStat,sCode,Sampled,RunTime
 Serial.println(F("roll,pitch,yaw,IMUP,IMUT,IMUA,BMP,BMPT,BMPA,LPG,CH4,CO,H:M:S.ms,DD/MM/20YY,Fix,FixQ,Lat,Long,LatD,LongD,Speed,Angle,GAlt,Sats,rbStat,sCode,Sampled,RunTime"));
 delay(1000);
 
}
/////////////////////////////////////////////////////////
//END SETUP


// GPS STUFF
/////////////////////////////////////////////////////////
// Adafruit put this code here in their GPS parsing example. Im not going to argue with them. ;)
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

 /////////////////////////////////////////////////////////
 // END GPS STUFF 


//LOOP
/////////////////////////////////////////////////////////
 void loop() {

  //Launch Day State Machine
  switch(state) {
    case 0:   //State 00, Waiting State

      state++;
      break;
  
    case 1:   //State 01, Data Collection State
   
       //IMU OPERATIONS
       //////////////////////////////////////////////////////
       //Get a new sensor event
       //IMU Raw data
       //
       //Raw sensor event data - raw IMU data
       sensors_event_t A_event;
       sensors_event_t M_event;
       sensors_event_t G_event;
       sensors_event_t B_event;
           
       float temperature; //Ambient temperature
         
       //Retrieve sensor data
       A.getEvent(&A_event);
       M.getEvent(&M_event);
       G.getEvent(&G_event);
         
       /*
       //Display Raw Sensor Information: Accel, Mag, Gyro
       Serial.print(A_event.acceleration.x); Serial.print(F(","));
       Serial.print(A_event.acceleration.y); Serial.print(F(","));
       Serial.print(A_event.acceleration.z); Serial.print(F(","));
       Serial.print(M_event.magnetic.x);     Serial.print(F(","));
       Serial.print(M_event.magnetic.y);     Serial.print(F(","));
       Serial.print(M_event.magnetic.z);     Serial.print(F(","));
       Serial.print(G_event.gyro.x);         Serial.print(F(","));
       Serial.print(G_event.gyro.y);         Serial.print(F(","));
       Serial.print(G_event.gyro.z);         Serial.print(F(","));
      */  

      //IMU - AHRS Orientation data 
      sensors_vec_t   orientation;   //sensor vector data - calculated IMU-AHRS orientation
      
      //Retrieve IMU sensors orientation data
      if (ahrs.getOrientation(&orientation))

      //Display Sensor Information: Orientation (Roll, Pitch, Heading)
      /* 'orientation' should have valid .roll and .pitch fields */
      Serial.print(orientation.roll);     Serial.print(F(","));
      Serial.print(orientation.pitch);    Serial.print(F(","));
      Serial.print(orientation.heading);  Serial.print(F(","));
       
      //Display Sensor Information: Pressure, Temp, Alt
      B.getEvent(&B_event);
      if (B_event.pressure){
        Serial.print(B_event.pressure);     Serial.print(F(","));
        B.getTemperature(&temperature);
        Serial.print(temperature);          Serial.print(F(","));
      }      

      //BAROMETER OPERATIONS
       /////////////////////////////////////////////////////////
       char status;
       double T,P,p0,a;
      
       status = pressure.startTemperature();
       if (status != 0)
       {
         // Wait for the measurement to complete:
         delay(status);
    
         // Retrieve the completed temperature measurement:
         // Note that the measurement is stored in the variable T.
         // Function returns 1 if successful, 0 if failure.
    
         status = pressure.getTemperature(T);
         if (status != 0)
         {
          
           // Start a pressure measurement:
           // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
           // If request is successful, the number of ms to wait is returned.
           // If request is unsuccessful, 0 is returned.
    
           status = pressure.startPressure(3);
           if (status != 0)
           {
             // Wait for the measurement to complete:
             delay(status);
    
             // Retrieve the completed pressure measurement:
             // Note that the measurement is stored in the variable P.
             // Note also that the function requires the previous temperature measurement (T).
             // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
             // Function returns 1 if successful, 0 if failure.
    
             status = pressure.getPressure(P,T);
             if (status != 0)
             {
    
               // The pressure sensor returns abolute pressure, which varies with altitude.
               // To remove the effects of altitude, use the sealevel function and your current altitude.
               // This number is commonly used in weather reports.
               // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
               // Result: p0 = sea-level compensated pressure in mb
    
               p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
            
               // On the other hand, if you want to determine your altitude from the pressure reading,
               // use the altitude function along with a baseline pressure (sea-level or other).
               // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
               // Result: a = altitude in m.
    
               a = pressure.altitude(P,p0);
             }
             else Serial.println("error retrieving pressure measurement\n");
           }
           else Serial.println("error starting pressure measurement\n");
         }
         else Serial.println("error with baro");
       }
       else Serial.println("error with baro\n");

       // Check the Temp, if it's >= 0 indicate and turn on the heater
       if(T <= 0)
       {
         digitalWrite(heater, HIGH); // turn on the heat
       }
       if(T > 0)
       {
         digitalWrite(heater, LOW); // turn off the heat
       }
       
       Serial.print(T,2);                    Serial.print(F(","));
       Serial.print(P,2);                    Serial.print(F(","));
       Serial.print(p0,2);                   Serial.print(F(","));       

       //GAS SENSOR OPERATIONS
       //////////////////////////////////////////////////////
       mq4 = averageAnalogRead(1);        // read LPG analog input pin 1
       mq6 = averageAnalogRead(2);        // read CH4 analog input pin 2
       mq7 = averageAnalogRead(3);        // read CO analog input pin 3

       float mq4_ppm = mapfloat(mq4,0,1023,300,10000);    //Convert the voltage to PPM
       float mq6_ppm = mapfloat(mq6,0,1023,300,10000);    //Convert the voltage to PPM
       float mq7_ppm = mapfloat(mq7,0,1023,10,1000);      //Convert the voltage to PPM

       Serial.print(mq4_ppm, DEC);           Serial.print(F(","));
       Serial.print(mq6_ppm, DEC);           Serial.print(F(","));
       Serial.print(mq7_ppm, DEC);           Serial.print(F(","));

     
      //GPS OPERATIONS
      //////////////////////////////////////////////////////
      if (! usingInterrupt) {
        char c = GPS.read();
      }
      
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
          return;
      }
       
      Serial.print(GPS.hour, DEC);       Serial.print(F(":")); 
      Serial.print(GPS.minute, DEC);     Serial.print(F(":"));
      Serial.print(GPS.seconds, DEC);    Serial.print(F("."));
      Serial.print(GPS.milliseconds);    Serial.print(F(","));
      Serial.print(GPS.day, DEC);        Serial.print(F("/"));
      Serial.print(GPS.month, DEC);      Serial.print(F("/20"));
      Serial.print(GPS.year, DEC);       Serial.print(F(","));
      Serial.print((int)GPS.fix);        Serial.print(F(","));
      Serial.print((int)GPS.fixquality); Serial.print(F(","));
      //if no GPS fix print commas
      if(!GPS.fix) {
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
      }
      if (GPS.fix) {
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);    Serial.print(F(",")); 
        Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);   Serial.print(F(","));
        Serial.print(GPS.latitudeDegrees, 4);                    Serial.print(F(","));
        Serial.print(GPS.longitudeDegrees, 4);                   Serial.print(F(","));
        
        Serial.print(GPS.speed);                                 Serial.print(F(","));
        Serial.print(GPS.angle);                                 Serial.print(F(","));
        Serial.print(GPS.altitude);                              Serial.print(F(","));
        Serial.print((int)GPS.satellites);                       Serial.print(F(","));

        // Sends RockBLOCK a message to transnsmit every 30s
        if((millis() / 1000) % 30 == 0) {
          Wire.beginTransmission(8);
          Wire.write("LAT: ");
          message.remove(0); // clear message
          message += GPS.latitudeDegrees;
          Wire.write(message.c_str());
          Wire.write(", ");
          Wire.write("LON: ");
          message.remove(0); // clear message
          message += GPS.longitudeDegrees;
          Wire.write(message.c_str());
          Wire.write(", ");
          Wire.write("FLIGHT STAGE: ");
          Wire.write((falling)? "DESC, " : "ASC, ");
          Wire.write("AS STAGE: ");
          Wire.write(sampleCode);
          Wire.write(", ");
          Wire.write("AS STATUS: ");
          Wire.write((sampled)? "COMPLETE" : "SAMPLING");
          Wire.write('\0');
          Wire.endTransmission(); 
        }
    
        /*
         * Checks if falling every 10s
         * 
         * Explanation:
         * collects 5 seconds of altitude data points, averages them and stores the average 
         * in oldAlt. The process repeats and stores the average into alt, then checks if 
         * falling.
         */
        if((millis() / 1000) % 2) {
          if(offset < 5) {
            gpsAlt[offset] = GPS.altitude;
            offset++;
          } else { // i.e. offset is 5
            float total = 0;
            for(int i = 0; i < 5; i++) {
              total += gpsAlt[i];
            }
            float average = total / 5.0;
    
            if(!falling) {
              // get ready to call altDetect
              if(forOldAlt) {
                oldAlt = average;
                forOldAlt = false;
              } else { //i.e. for alt
                alt = average;
                forOldAlt = true;
                checkAlt = true;
              }
      
              // call altDetect
              if(checkAlt) {
                altDetect(oldAlt, alt, altBreakPoint);
                checkAlt = false;
              }
            } else {
              if(average < altBreakPoint[abpOffset]) {
                char sampleCode = '0' + abpOffset;
                Wire.beginTransmission(9);
                Wire.write(sampleCode);
                Wire.endTransmission();
                abpOffset++;
              }
            }
            
          }
        }
        
      }

      Wire.requestFrom(8, 3);
      while(Wire.available()) {
        rbStatus = Wire.read();
        rbTransmissions = Wire.read();
        rbRecives = Wire.read();
      }
      Wire.requestFrom(9, 2);
      while(Wire.available()) {
        sampleCode = Wire.read();
        sampled = Wire.read();
      }
    
      Serial.print(F(rbStatus));            Serial.print(F(","));
      Serial.print(F(rbTransmissions));     Serial.print(F(","));
      Serial.print(F(rbRecives));           Serial.print(F(","));
      Serial.print(F(sampleCode));          Serial.print(F(","));
      Serial.print(F(sampled));             Serial.print(F(","));
      Serial.print(F(millis() / 1000));     Serial.print(F(","));
      Serial.println(F("")); //print new line

      delay(1000); // delay 1 sec
      digitalWrite(LD0, (digitalRead(LD0) == HIGH)? LOW : HIGH); // toggles LD0 on and off
      break;
   }
}
/////////////////////////////////////////////////////////
//END LOOP


//FUNCTION DEFINITIONS
//////////////////////////////////////////////////////

//int averageAnalogRead(int pinToRead) - Takes an average of readings on a given pin. Returns the average
//////////////////////////////////////////////////////
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
  runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}


//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SIGNAL(TIMER0_COMPA_vect) - Used by the GPS.
// Adafruit put this code here in their GPS parsing example. Im not going to argue with them. ;)
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

// void useInterrupt(boolean v) - Used by the GPS
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void altDetect (float oldAlt, float alt, int altBreakPoint[]){
  int stepSize = 0;
  int deltaAlt = alt - oldAlt;
  if (deltaAlt < 0) {
    stepSize = alt/5;
    for(int y = 0; y < 5; y++){
      altBreakPoint[5 - y] = {alt - ((y + 1) * stepSize)}; // stores break points in decsending order
    }
    falling = true; 
  }  
}
