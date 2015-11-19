/****************************************************
 * Phoenix College Acsend Launch Day Logic Fall 2015
 *
 * This file contains the code for collecting data from:
 * 
 * (IMU) - Adafruit 10-DOF IMU Breakout
 * (UV) - SparkFun UV Sensor Breakout - ML8511
 * (RBG) - SparkFun RGB Light Sensor - ISL29125
 * (Luminosity) - SparkFun Luminosity Sensor Breakout - TSL2561
 * (Barometer) - SparkFun Barometric Pressure Sensor Breakout - BMP180
 * (GPS) - Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3
 * 
 * Authors: Phoenix College Acsend Team 2015 - 2016
 * 
 * Version 1.0rc12
 * 
 * TODO's: 
 *  Failure incapsulation:
 *    Get rid of while(1) statements, they loop forever.
 *    Keep output file integrety by printing "," in place of failed sensor.
 *  Settings:
 *    IMU - Acceleration scale, if we can.
 *    Luminosity - gain and integration rate
 *    Barometer - Altitude constant
 *  LED outputs.
 *    Sensor errors. - lum, baro, GPS
 *  Finish header comments.
 *    explain output file.
 *  Wiring Guide.
 * 
 ***************************************************/

/////////////////////////////////////////////////////////
//Includes, (Up to date libraries folder can be downloaded at: https://github.com/PC-Ascend-Team/libraries)
#include <Wire.h>          	    //IMU, Luminosity, Borometer, RGB

#include <Adafruit_Sensor.h>   //IMU
#include <Adafruit_10DOF.h>    //IMU
#include <Adafruit_LSM303_U.h> //IMU
#include <Adafruit_L3GD20_U.h> //IMU
#include <Adafruit_BMP085_U.h> //IMU

#include <SparkFunTSL2561.h>   //Luminosity

#include <SFE_BMP180.h>    	   //Barometer

#include <SparkFunISL29125.h>  //RGB

#include <Adafruit_GPS.h>      //GPS
#include <SoftwareSerial.h>    //GPS

//IMU Definitions
/////////////////////////////////////////////////////////
//Assign a unique ID to the sensors
Adafruit_LSM303_Accel_Unified A   = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   M   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       B   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified   	  G   = Adafruit_L3GD20_Unified(20);

/*
//UV Definitions
/////////////////////////////////////////////////////////
//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
*/


//RGB Definitions
/////////////////////////////////////////////////////////
SFE_ISL29125 RGB_sensor; // Declare sensor object


//Luminosty Definitions
/////////////////////////////////////////////////////////
SFE_TSL2561 light; // Create an SFE_TSL2561 object, here called "light":
boolean gain; 	// Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds


//Barometer Definitions
/////////////////////////////////////////////////////////
SFE_BMP180 pressure;
#define ALTITUDE 1655.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters

//GPS Definitions
////////////////////////////////////////////////////////
SoftwareSerial mySerial(3, 2); // TX - D3, RX - D2
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false // we do NOT want to echo raw GPS data to serial
boolean usingInterrupt = false;


//Phoenix College Acsend Team variables
////////////////////////////////////////////////////////
int state = 0; //for state machine switch statement
int t = 0; 	//time keeper (seconds)

//LEDs
int LD0 = 6; // D6
int LD1 = 7; // D7
int LD2 = 8; // D8
int LD3 = 9; // D9

int heater = 10; // D10


//Funcions Prototypes
/////////////////////////////////////////////////////////

// int averageAnalogRead(int pinToRead) - Takes an average of readings on a given pin. Returns the average
int averageAnalogRead(int pinToRead);

// void printError(byte error) - Used by the Luminosity sensor to display errors
void printError(byte error);

// void useInterrupt(boolean v) - Used by the GPS
void useInterrupt(boolean);


//SETUP
/////////////////////////////////////////////////////////
void setup(void){
  Serial.begin(115200); //begin serial conncetion

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

  /*
  //UV SETUP
  //////////////////////////////////////////////////////
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
*/

  //RGB SETUP
  //////////////////////////////////////////////////////
  // Initialize the ISL29125 with simple configuration so it starts sampling
  if (RGB_sensor.init()) {
    
  }else {
    // Light LED code
  }
  

  //LUMINOSITY SETUP
  //////////////////////////////////////////////////////
  light.begin();
  
  // If gain = false (0), device is set to low gain (1X)
  // If gain = high (1), device is set to high gain (16X)
  gain = 0;

  // If time = 0, integration will be 13.7ms
  // If time = 1, integration will be 101ms
  // If time = 2, integration will be 402ms
  // If time = 3, use manual start / stop to perform your own integration
  unsigned char time = 2;

  light.setTiming(gain,time,ms);
  light.setPowerUp();
 

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
  	Serial.println("BMP180 init fail\n\n");
  	while(1); // Pause forever.
  }
 
 

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
  pinMode(LD1, OUTPUT);
  pinMode(LD2, OUTPUT);
  pinMode(LD3, OUTPUT);
  pinMode(heater, OUTPUT);
    

  //Output Header
  //////////////////////////////////////////////////////
  //Serial.println(F("Ax,Ay,Az,Mx,My,Mz,Gx,Gy,Gz,Bp,Bt,UVl,UVi,R,G,B,Vl,Il,LUX,T,P,BAlt,H:M:S.ms,DD/MM/20YY,Fix,FixQ,Lat,Long,LatD,LongD,Speed,Angle,GAlt,Sats"));
  Serial.println(F("Ax,Ay,Az,Mx,My,Mz,Gx,Gy,Gz,Bp,Bt,R,G,B,Vl,Il,LUX,T,P,BAlt,H:M:S.ms,DD/MM/20YY,Fix,FixQ,Lat,Long,LatD,LongD,Speed,Angle,GAlt,Sats"));
  delay(500);
 
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
  	case 0:  	//State 00, Waiting State
  
   	  //To setup a 2 min time delay (Untested), comment ou the line below
      //and uncomment the 3 below that.
   	  state = 1;
  
      //Uncomment these 3 lines to setup a 2 min time delay (Untested)
      /*
    	if(t >= 120) { state++; } //transision to next state
    	delay(1000); //delay 1s
    	t++; //keep time
      */    
    	break;
  
  	case 1:  	//State 01, Data Collection State

      LD0 = HIGH; //Light LD0 to indicate start data loging
   
    	//IMU OPERATIONS
    	//////////////////////////////////////////////////////
    	//Get a new sensor event
    	sensors_event_t A_event;
    	sensors_event_t M_event;
    	sensors_event_t G_event;
    	sensors_event_t B_event;
   	 
   	 
    	float temperature; //Ambient temperature
 	 
    	//Retrieve sensor data
    	A.getEvent(&A_event);
    	M.getEvent(&M_event);
    	G.getEvent(&G_event);
 	 
    	//Display Sensor Information: Accel, Mag, Gyro
    	Serial.print(A_event.acceleration.x); Serial.print(F(","));
    	Serial.print(A_event.acceleration.y); Serial.print(F(","));
    	Serial.print(A_event.acceleration.z); Serial.print(F(","));
    	Serial.print(M_event.magnetic.x);   	Serial.print(F(","));
    	Serial.print(M_event.magnetic.y);   	Serial.print(F(","));
    	Serial.print(M_event.magnetic.z);   	Serial.print(F(","));
    	Serial.print(G_event.gyro.x);       	Serial.print(F(","));
    	Serial.print(G_event.gyro.y);       	Serial.print(F(","));
    	Serial.print(G_event.gyro.z);       	Serial.print(F(","));
 	 
    	//Display Sensor Information: Pressure, Temp, Alt
    	B.getEvent(&B_event);
    	if (B_event.pressure){
      	Serial.print(B_event.pressure);   	Serial.print(F(","));
      	B.getTemperature(&temperature);
      	Serial.print(temperature);        	Serial.print(F(","));
    	}      
   	 /*
    	//UV OPERATIONS
    	//////////////////////////////////////////////////////
    	int uvLevel = averageAnalogRead(UVOUT);
    	int refLevel = averageAnalogRead(REF_3V3);
   	 
    	//Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    	float outputVoltage = 3.3 / refLevel * uvLevel;
   	 
    	float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
   	 
    	Serial.print(uvLevel);              	Serial.print(F(","));
    	Serial.print(uvIntensity);          	Serial.print(F(","));
     */
     
      
    	//RGB OPERATIONS
    	//////////////////////////////////////////////////////
    	// Read sensor values (16 bit integers)
    	unsigned int red = RGB_sensor.readRed();
    	unsigned int green = RGB_sensor.readGreen();
    	unsigned int blue = RGB_sensor.readBlue();
   	 
    	// Print out readings, change HEX to DEC if you prefer decimal output
    	Serial.print(red,DEC);              	Serial.print(F(","));
    	Serial.print(green,DEC);            	Serial.print(F(","));
    	Serial.print(blue,DEC);             	Serial.print(F(","));
 	 
 	 
    	//LUMINOSITY OPERATIONS
    	//////////////////////////////////////////////////////
    	//Once integration is complete, we'll retrieve the data.
 
    	unsigned int data0, data1;//Retrieve the data from the device:
   	 
    	if (light.getData(data0,data1))
    	{
     	 
      	//To calculate lux, pass all your settings and readings
      	//to the getLux() function.
   	 
      	double lux;	//Resulting lux value
      	boolean good;  //True if neither sensor is saturated
     	 
      	good = light.getLux(gain,ms,data0,data1,lux); //Perform lux calculation:
     	 
      	// Print out the results:
      	Serial.print(data0);              	Serial.print(F(","));
      	Serial.print(data1);              	Serial.print(F(","));
      	Serial.print(lux);                	Serial.print(F(","));
     	 
    	}
    	else
    	{
      	//getData() returned false because of an I2C error, inform the user.
      	byte error = light.getError();
      	printError(error);
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
      if(T <= 0){LD2 = HIGH; heater = HIGH;}
      if(T > 0){LD2 = HIGH; heater = HIGH;}
 	    
    	Serial.print(T,2);                  	Serial.print(F(","));
    	Serial.print(P,2);                  	Serial.print(F(","));
    	Serial.print(p0,2);                 	Serial.print(F(","));
    	
         	 

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
      if(GPS.fix == 0) {
        LD1 = LOW;
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
      }
      if (GPS.fix == 1) {
        LD1 = HIGH;
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);    Serial.print(F(",")); 
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon); Serial.print(F(","));
        Serial.print(GPS.latitudeDegrees, 4);                    Serial.print(F(","));
        Serial.print(GPS.longitudeDegrees, 4);                   Serial.print(F(","));
        
        Serial.print(GPS.speed);                                 Serial.print(F(","));
        Serial.print(GPS.angle);                                 Serial.print(F(","));
        Serial.print(GPS.altitude);                              Serial.print(F(","));
        Serial.print((int)GPS.satellites);                       Serial.print(F(","));
      }
  
      Serial.println(F("")); //print new line

      delay(1000); // delay 1 sec

      LD0 = LOW; // finished one iteration of data logging
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


//void printError(byte error) - Used by the Luminosity sensor to display errors
//////////////////////////////////////////////////////
void printError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
 
  switch(error)
  {
	case 0:
  	Serial.println("success");
  	break;
	case 1:
  	Serial.println("data too long for transmit buffer");
  	break;
	case 2:
  	Serial.println("received NACK on address (disconnected?)");
  	break;
	case 3:
  	Serial.println("received NACK on data");
  	break;
	case 4:
  	Serial.println("other error");
  	break;
	default:
  	Serial.println("unknown error");
  }
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

