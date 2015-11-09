/****************************************************
 * Phoenix College Acsend Launch Day Logic Fall 2015
 * 
 * Authors: 
 ***************************************************/

void setup() {

  int state = 0;     //for launch day switch statement
  int t = 0;         //time keeper (miliseconds)
  int initLoop = 1;  //initial loop indicator

  Serial.begin(9600);

  Serial.print("$ , , , , , , , , , , , , UV Level, UV Intensity (mW/cm^2) ");
  Serial.print("\n");

} //end setup()

void loop() {

  //Launch Day State Machine
  switch(state) {
    case: 0      //State 00, Waiting State  
      
      if(t >= 120000) { state++; } //transision to next state
      delay(1); //delay 1ms
      t++; //keep time
      
      break;

    case: 1      //State 01, Data Collection State

      //print data
      
      
      break;
  } //end state machine
} //end loop()
