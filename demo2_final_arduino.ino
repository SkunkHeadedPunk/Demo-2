// Logan Meyer, Thomas Klinedinst, Jeffrey Hostetter     EENG 350           Last Modified: 4 19 2021
// This is the Arduino code used to accomplish the tasks for Demo2. 

/* For this file to run properly:  
    - Must include libraries containing "Encoder.h", "DualMC33926MotorShield.h", and "Wire.h"
    - Must have Respberry Pi with proper I2C communication running "CV_Demo2.py"  
 
 In this code, the Arduino is running as a Finite State Machine with the following states:
  0: Robot spins at constant speed until Pi detects marker (piState == 1). Robot then stops to let Pi take high-res photo
  1: Arduino receives data from Pi, resets angular positions, and does initial calculations 
  2: Arduino turns to face the marker, using a PI control system
  3: Stops motors, determines distance needed to drive to the marker
  4: Robot drives to the marker using a PI control system 
  5: Robot spins 90 degrees to the right (hard-coded) 
  6: Robot spins in circle around marker (hard-coded) 
  7: Robot stops motors. 
*/



#include <Encoder.h>
#include <DualMC33926MotorShield.h>
#include <Wire.h>

#define diameter          0.5 // Diameter of the wheel, in ft 
#define countsPerRotation 3200 

// ~~~~~~~~~~~~~~~~~~~~~~~~~~PIN LOCATTIONS FOR THE ARDUINO~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define motorRPWM         10
#define motorLPWM         9 
#define voltageRDir       7   // Direction for Left motor
#define voltageLDir       8   // Direction for Right motor
#define pinD2             4 
#define ADDRESS           0x04 

Encoder rightWheel(2,5);  // 2 is an interrupt pin (A/ Yellow Right)    ***DON'T USE PIN 1***
Encoder leftWheel(3,6);   // 3 is an interrupt pin (A/ Yellow Left)    
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DualMC33926MotorShield md;

int state = 0;
int piState = 0;   
int motorSpeedL = 0; 
int motorSpeedR = 0;
bool loganCSdone = false;
float angleToSpin; // Angle to spin, in (degrees or rads)   
float angleRead = 180;    // Angle taken from the Pi 
float distanceToGo = 0; // Needed distance, in feet ***SENT FROM PI, read is redundant
float distanceRead = 0; // Distance from the cam to the marker, in feet 
float turn90LinearPosOld;  
float turn90LinearPosDest;
long oldPositionL = -999; // Included in the basic encoder example code
long oldPositionR = -999;
byte dataFromPi[32];


//******CONTROL SYSTEM INFO AND VARIABLES***////
float  goalAngle = 3.14159;   // Desired angle of rotation, in radians (CHANGE PER RUN)  
float goalAngleAngle = 0;
float goalDistanceAngle = 0;
float goalDistance = 0;            // Desired linear distance traveled, in ft (CHANGE PER RUN)
float directiongoing = -1; ///  1 if counter clockwise    ////-1 if backwards or clockwise
float angularPositionL;
float linearPositionL;
float angularPositionR; 
float linearPositionR;
float radPerCount;
float circumference; 
float speedR = 100;//20;   // Built-in variable for the speed of the motor (Right), Between -400, 400
float speedL = 100;//20;   // Built-in variable for the speed of the motor (Left), Between -400, 400

/////control sytem variables here for rotational control
float fudge = 0; // Correction factor
float KpLs = 0.74; // 0.64  Proportional gain of left motor in spin test (unitless)
float KiLs = 0.00019; //0.0001;    0.00019 for pi/2 and under 0.00016 for pi and 0.00012 for 2*pi   Integrator gain of left motor in spin test (v/encoder rads)
float KpRs = 0.74; // 0.64;
float KiRs = 0.00019; //0.0001;

/////control sytem variables here for linear control
float KpL = 0.256; // 0.64;    Proportional gain of left motor in linear test
float KiL = 0.0002; //0.0001;    0.0002 for 5 feet 0.0001 for 10  Integrator gain of left motor in linear test (v/encoder rads)
float KpR = 0.24; // 0.64;
float KiR = 0.0002; //0.0001; 

float delayValue = 180;//40;      // Makes sure the Control system doesn't run twice with the same data (ensures no div/0)
float timetogo = 5000;
float triggertime = 0;

  ///static control variables
  float rL, rR, yL, yR, eL, eR;
  float IL = 0;
  float IR = 0;
  float Ts = 0;
  float Tc = 0;


//defines for the control system
#define diameter 0.5 // Diameter of the wheel, in ft  

//float diametermod = (float) 0.501 - ( (float) (( (float) goalDistance / (float) 10) * (float) 0.01) );  
#define diametermod 0.5008 // More accurate diameter based on testing                  // 10ft: 0.5, 8ft: 0.5007 5ft: 0.501
#define diametermodspin 0.498     // Different to account for minor slippage on rotation (wheel following curved path)
#define wheelRadius  0.25
#define wheelbase  1.167  // Wheelbase, in ft (Logan measured 1.166666)
#define turnRadius  0.584  // Half of the wheelbase 
//defines



void setup() {
  circumference = (float) PI * diameter; // Calculates wheel circumference in ft 
  radPerCount = (float) 2 * PI / (float) countsPerRotation;
  Wire.begin(ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(9600); 
  md.init();
}

void loop() {
  static float uR = speedR;
  static float uL = speedL;
  
  //***************************************************************
  //***     Reading Encoders and Calculating Angular Position   *** 
  //***************************************************************
  // This section copied from the previous demo
  long newPositionL = leftWheel.read(); 
  long newPositionR = rightWheel.read(); 
  if (newPositionL != oldPositionL) {   
    if (oldPositionL != -999) {    // Makes sure angularPosition isn't calculated using the initial value for oldPosition
      angularPositionL = angularPositionL + (newPositionL - oldPositionL) * radPerCount; // Calculated based on position diff. 
    }                                                                                // to allow for easy reset w/o resetting
    oldPositionL = newPositionL;                                                       // position variables. 
  }   

  if (newPositionR != oldPositionR) {   
    if (oldPositionR != -999) {    // Makes sure angularPosition isn't calculated using the initial value for oldPosition
      angularPositionR = angularPositionR + (newPositionR - oldPositionR) * radPerCount; // Calculated based on position diff. 
    }                                                                                    // to allow for easy reset w/o resetting
    oldPositionR = newPositionR;                                                         // position variables. 
  } 

   // Mechanism for resetting the positions to zero
  if (Serial.read() == 'r') {   
    angularPositionR = 0; 
    angularPositionL = 0;
    Serial.println("Reset angular positions");
  } 

  // Calculating linear position based on angular positions:
  linearPositionL = -( angularPositionL * circumference / (float) (2*PI) );   // Negative just to account for the proper direction
  linearPositionR = ( angularPositionR * circumference / (float) (2*PI) );  

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//          F   I   N   I   T   E      S   T   A   T   E      M   A   C   H   I   N   E   
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  switch (state)
  {
    case 0: 
        motorSpeedL = 70;
        motorSpeedR = 70;
       
        //Robot spins slowly until detecting Aruco 
        if (piState == 1){  
          md.setM1Speed(0); // Hardcode to turn at constant speed until detected
          md.setM2Speed(0);
          delay(500);
          motorSpeedL = 0;
          motorSpeedR = 0; // Stops motors
          state = 1; 
          // send state value to Pi in order to get precise image 
        }
        break;
    case 1: 
        if(angleRead != 149){       // Once angleRead changes, Arduino knows that the Pi sent the accurate angle
          angleToSpin = PI * (float) angleRead / (float) 180;
          goalAngle = angleToSpin;
          goalAngleAngle = (goalAngle*(float) wheelbase/ (float) diametermodspin ) - (float) 0.08; //wheelbas*wheelbase/r
          state = 2;
          IL = 0;
          IR = 0;
          angularPositionL = 0;
          angularPositionR = 0;

          if(abs(angleRead) > 12){
             fudge = (float) 0.26;

          }
           if(abs(angleRead) < 11){
             fudge = (float) 0.1;
          }
        }  

        break;
        
    case 2:          
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          // START CONTROL SYSTEM FOR TURNING angleToSpin 
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
          
          rL = goalAngleAngle;
          rR = goalAngleAngle;
          yL = angularPositionL*directiongoing;
          yR = angularPositionR*directiongoing;

          eL = rL-yL;
          eR = rR-yR;
    
          IL = IL + Ts*eL;
          IR = IR + Ts*eR;

          uL = KpLs*eL + KiLs*IL;
          uR = KpRs*eR + KiRs*IR;

          if(abs(angularPositionL) >= (abs(goalAngleAngle)-fudge)){
              loganCSdone = true;

          }

          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          // END END END CONTROL SYSTEM FOR TURNING angleToSpin
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 

          if(loganCSdone) {  
            loganCSdone = false;
            state = 3;
            IL = 0;
            IR = 0;
            angularPositionL = 0;
            angularPositionR = 0;
            speedR = 20;//20;   // Built-in variable for the speed of the motor (Right), Between -400, 400
            speedL = 20;
            
          }
          motorSpeedL = (speedL*uL* (float) -1);
          motorSpeedR = (speedR*uR* (float) -1);
          Serial.println(motorSpeedL);
          Serial.println(motorSpeedR);
          Serial.println("---");

          break; 
          
   case 3:
          Serial.println("Case 3");

          motorSpeedL = 0;
          motorSpeedR = 0;   
          if(distanceRead != 0){ // Once distance changes, indicates Pi sent accurate distance
            distanceToGo = (distanceRead / (float) 12) - (float) 1.1; // Converts Pi distance to feet, then accounts for stopping away from the beacon
            goalDistanceAngle = (float) 1*distanceToGo* (float) 2/ (float) diametermod;
            state = 4;
            directiongoing = 1;
          }
          break;

   case 4:
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          // START START START CONTROL SYSTEM FOR DRIVING TO distanceToGo 
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          rL = goalDistanceAngle;
          rR = goalDistanceAngle;
          yL = angularPositionL*directiongoing*((float) -1);  // y is actual angular position
          yR = angularPositionR*directiongoing;

          eL = rL-yL;
          eR = rR-yR;

 
          IL = IL + Ts*eL;
          IR = IR + Ts*eR;

          uL = KpL*eL + KiL*IL;
          uR = KpR*eR + KiR*IR;

          if((((rL-yL))<0.3) || ((yL-rL)>0.3)){
            uL = uL - KiL*IL;
            loganCSdone = true;
          }
          if((((rR-yR))<0.3) || ((yR-rR)>0.3)){
            uR = uR - KiR*IR;
            loganCSdone = true;
          }

          uL = uL*directiongoing*((float) -1);
          uR = uR*directiongoing;
          
          motorSpeedL = (speedL*uL* (float) -1);
          motorSpeedR = (speedR*uR* (float) -1);

           //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          // END END END CONTROL SYSTEM FOR DRIVING TO distanceToGo 
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
           if (loganCSdone) {  
            loganCSdone = false;
            state = 5;                                                                    // ~~~~~FOR DOING DEMO PART 2, CHANGE TO GO TO STATE 7 ~~~~~~~~~~~~~
            IL = 0;                                                                       //  (This could be done with a button/ jumper wire/ Pi communication)
            IR = 0;
            angularPositionL = 0;
            angularPositionR = 0;
            motorSpeedL = 0;
            motorSpeedR = 0;   
          }
          break;  
          
   case 5:
          //hardcoded to spin 90 degrees to the right 
          //turn90LinearPosDest = ( PI / (float) 2 ) * ( (float) wheelbase / (float) 2);
          motorSpeedL = 80;
          motorSpeedR = 80; 
         
          turn90LinearPosOld = ((PI/(float)2)*(float) wheelbase/ (float) diametermodspin ) * (float) 1;  // Determines 90 degree turn based on arc length traced by wheel
         if(abs(angularPositionL) >= turn90LinearPosOld){
          
            motorSpeedL = 0;
            motorSpeedR = 0;
            state = 6;
            IL = 0;
            IR = 0;
            angularPositionL = 0;
            angularPositionR = 0;
         }
          break;
         
               
   case 6:
            // Turns in circle around marker
            motorSpeedL = 80; 
            motorSpeedR =  - (float) 1.9*motorSpeedL;     // Hardcode to turn in circle, scale is the difference in circumference for each wheel 
            if(abs(angularPositionR) >= (float) 46.5){      // 46.5 is number of radians the outer wheel needs to turn (adjusts how much of the circle is completed)
              state = 7;
            }
        break;
       
    case 7:  
        motorSpeedL = 0;
        motorSpeedR = 0; // Stops motors
        // Doesn't need to update booleans or anything like that since that's automatically
        // done on the Pi side
        break; 
    }

    Ts = millis()-Tc;
    Tc = millis();
    //delay(delayValue - Ts);
    md.setM1Speed(motorSpeedL); // Hardcode to turn at constant speed until detected
    md.setM2Speed(motorSpeedR);
 
    //Serial.println (state); 
} 


//Callback for receiving data from Pi
void receiveData(int byteCount) { 
  //Serial.println("R E C E I V I N G");
  int i = 0;
  while (Wire.available()) {
    dataFromPi[i] = Wire.read();
    i++;
  }
  piState = dataFromPi[1];
  angleRead = dataFromPi[2] - 31;
  distanceRead = dataFromPi[3]; 
  //Serial.println(piState);
  Serial.println(angleRead);
  Serial.println(distanceRead);
}

//Callback for sending data to Pi
void sendData() {
  Wire.write(state);
}
