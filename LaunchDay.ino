/****************************************************
 * Phoenix College Acsend Launch Day Logic Fall 2015
 * 
 * This file contains the code for collecting data from: 
 * (IMU) - Adafruit 10-DOF IMU Breakout
 * (Luminosity) - SparkFun Luminosity Sensor Breakout - TSL2561
 * (UV) - SparkFun UV Sensor Breakout - ML8511
 * (GPS) - Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3
 * 
 * Authors: 
 ***************************************************/

/////////////////////////////////////////////////////////
//Includes, (these must be put into the libraries folder before coe will run)
#include <Wire.h>              //IMU, Luminosity

#include <Adafruit_Sensor.h>   //IMU
#include <Adafruit_LSM303_U.h> //IMU
#include <Adafruit_BMP085_U.h> //IMU
#include <Adafruit_L3GD20_U.h> //IMU
#include <Adafruit_10DOF.h>    //IMU

#include <SparkFunTSL2561.h>   //Luminosity


//IMU Definitions
/////////////////////////////////////////////////////////
//Assign a unique ID to the sensors
Adafruit_LSM303_Accel_Unified A   = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   M   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       B   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       G   = Adafruit_L3GD20_Unified(20);

//Luminosty Definitions
/////////////////////////////////////////////////////////

SFE_TSL2561 light; // Create an SFE_TSL2561 object, here called "light":
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds

//UV Definitions
/////////////////////////////////////////////////////////
//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board



//Phoenix College Acsend Team variables
/////////////////////////////////////////////////////////
int state = 0; //for state machine switch statement
int t = 0;     //time keeper (seconds)


//Funcions Prototypes
/////////////////////////////////////////////////////////

//printError(byte error) - Used by the Luminosity sensor to display errors
void printError(byte error);


//SETUP
/////////////////////////////////////////////////////////
void setup(void){
  Serial.begin(9600);

  //IMU SETUP
  //////////////////////////////////////////////////////
  /* Initialise the sensors & check connections*/
  if(!A.begin()){
    Serial.println(F("No LSM303 detected."));
    while(1);
  }
  if(!M.begin()){
    Serial.println(F("No LSM303 detected."));
    while(1);
  }
  if(!B.begin()){
    Serial.print(F("No BMP085 detected."));
    while(1);
  }
  if(!G.begin()){
    Serial.print(F("No L3GD20 detected."));
    while(1);
  }

  sensor_t sensor;

  // Define sensors
  A.getSensor(&sensor);
  G.getSensor(&sensor);
  M.getSensor(&sensor);
  B.getSensor(&sensor);
  

  //LUMINOSITY SETUP
  //////////////////////////////////////////////////////
  light.begin();

  //TODO: Change Settings to what we want
  
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

  //UV SETUP
  //////////////////////////////////////////////////////
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  

  //RGB SETUP
  //////////////////////////////////////////////////////
  

  //GPS SETUP
  //////////////////////////////////////////////////////
 

  //Output Header
  Serial.println(F("Ax, Ay, Az, Mx, My, Mz, Gx, Gy, Gz, Bp, Bt, Ba"));
  delay(500);
  
}
/////////////////////////////////////////////////////////
//END SETUP


//LOOP
/////////////////////////////////////////////////////////
void loop() {

  //Launch Day State Machine
  switch(state) {
    case: 0      //State 00, Waiting State  
      
      if(t >= 120) { state++; } //transision to next state
      delay(1000); //delay 1s
      t++; //keep time
      
      break;

    case: 1      //State 01, Data Collection State

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
        Serial.print(A_event.acceleration.x);   Serial.print(F(","));
        Serial.print(A_event.acceleration.y);   Serial.print(F(","));
        Serial.print(A_event.acceleration.z);   Serial.print(F(","));
        Serial.print(M_event.magnetic.x);       Serial.print(F(","));
        Serial.print(M_event.magnetic.y);       Serial.print(F(","));
        Serial.print(M_event.magnetic.z);       Serial.print(F(","));
        Serial.print(G_event.gyro.x);           Serial.print(F(","));
        Serial.print(G_event.gyro.y);           Serial.print(F(","));
        Serial.print(G_event.gyro.z);           Serial.print(F(","));
      
        //Display Sensor Information: Pressure, Temp, Alt
        B.getEvent(&B_event);
        if (B_event.pressure){
          Serial.print(B_event.pressure);       Serial.print(F(","));
          B.getTemperature(&temperature);
          Serial.print(temperature);            Serial.print(F(","));
        
        
        //LUMINOSITY OPERATIONS
        //////////////////////////////////////////////////////
        //Once integration is complete, we'll retrieve the data.
  
        unsigned int data0, data1;//Retrieve the data from the device:
        
        if (light.getData(data0,data1))
        {
          
          //To calculate lux, pass all your settings and readings
          //to the getLux() function.
        
          double lux;    //Resulting lux value
          boolean good;  //True if neither sensor is saturated
          
          good = light.getLux(gain,ms,data0,data1,lux); //Perform lux calculation:
          
          // Print out the results:
          Serial.print(data0);                  Serial.print(F(","));
          Serial.print(data1);                  Serial.print(F(","));
          Serial.print(lux);                    Serial.print(F(","));
          
        }
        else
        {
          //getData() returned false because of an I2C error, inform the user.
          byte error = light.getError();
          printError(error);
        }
        
        
        //UV OPERATIONS
        //////////////////////////////////////////////////////
        
      
        //RGB OPERATIONS
        //////////////////////////////////////////////////////
        
      
        //GPS OPERATIONS
        //////////////////////////////////////////////////////
        

        Serial.println(F("")) //print new line
      break;
  }
  
}
/////////////////////////////////////////////////////////
//END LOOP


//FUNCTION DEFINITIONS
//////////////////////////////////////////////////////

//printError(byte error) - Used by the Luminosity sensor to display errors
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
