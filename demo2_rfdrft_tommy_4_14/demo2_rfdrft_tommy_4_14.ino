
//arduino running as a fsm.
// lowercase variables are data that pi gives though arduino can modify, ie set to 0.  upperacase are variables that arduino gives
// 0: Initialized and starts spinning 180 degrees to one direction. ifDetectedAruco = false;  ifaccuratereading = false; ifaccurateangle = false; angletospin = 0;  ifdistancefound = false; distancetogo = 0; 
// 0: If detected aruco robot stops. ifDetectedAruco = true;
// 1: PI gets more accurate reading.  ifaccurateangle = true; angletospin = bits of angle not sure;
// 2: Arduino spins back to correct angle then goes to state 3;
// 3: PI reads distance to aruco and passes ifdistancefound = true; distancetogo = distance to go not sure;
// 4: Arduino goes to aruco and once reaches aruco goes to state 4;
// 5: at the marker.
// *Note for setting up the  PI on a fsm simply pass pass the value of switch somewhere through to the pi.

//not sure about this but just thinking I don't think we can’t do that thing where we just pass the aruco for the final demo, because there is the possibility of 2 or more arucos in frame and unsure of angle, not sure about this though
// if we do it like the exmaple it ensures that there is never more then one aruco code in frame at any one time.

#include <Encoder.h>
#include <DualMC33926MotorShield.h>
#include <Wire.h>

#define demo              2 // WHICH PART OF THE DEMO THE CODE IS FOR 
#define diameter          0.5 // Diameter of the wheel, in ft (PRE-TAPE) 
#define countsPerRotation 3200 
#define motorRPWM         10
#define motorLPWM         9 
#define voltageRDir       7   // Direction for Left motor
#define voltageLDir       8   // Direction for Right motor
#define pinD2             4 
#define ADDRESS           0x04 

Encoder rightWheel(2,5);  // 2 is an interrupt pin (A/ Yellow Right)    ***DON'T USE PIN 1***
Encoder leftWheel(3,6);   // 3 is an interrupt pin (A/ Yellow Left)   
DualMC33926MotorShield md;


int state = 0;
int piState = 0;   
int motorSpeedL = 0; 
int motorSpeedR = 0;
bool loganCSdone;
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
float directiongoing = 1; ///  1 if counter clockwise    ////-1 if backwards or clockwise
//////////
float angularPositionL;
float linearPositionL;
float angularPositionR; 
float linearPositionR;
float radPerCount;
float circumference; 
float speedR = 200;//20;   // Built-in variable for the speed of the motor (Right), Between -400, 400
float speedL = 200;//20;   // Built-in variable for the speed of the motor (Left), Between -400, 400
/////control sytem variables here for rotational control


float KpLs = 0.34; // 0.64  Proportional gain of left motor in spin test (unitless)
float KiLs = 0.00019; //0.0001;    0.00019 for pi/2 and under 0.00016 for pi and 0.00012 for 2*pi   Integrator gain of left motor in spin test (v/encoder rads)
float KpRs = 0.34; // 0.64;
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




//defines for the controll SYSTEM
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
  //Serial.println(piState);
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*Jeffery need to get values for each of these here. I assume easiest to bitwise and to isolate and shift >> for each bool.  for each angle i assume similar
ifDetectedAruco = fixme;  //ie (big_32_Byte_Value & 001) >>1)  i assume something like this but not sure how to convert the value from pi to here.
ifaccuratereading = fixme; // Could be replaced by just checking if the array is populated
*****ifaccurateangle = fixme; // For this demo, the same as ifaccuratereading
*****ifdistancefound = fixme; 
ifArucoLocalized = fixme; // Signals that the distance/angle have been determined
angletospin = fixme;
distancetogo = fixme;
*/

  switch (state)
  {
    case 0: 
        motorSpeedL = 70;
        motorSpeedR = 70;
       
        //Robot spins slowly until detecting Aruco 
        if (piState == 1){  
          motorSpeedL = 0;
          motorSpeedR = 0; // Stops motors
          state = 1; 
          // send state value to Pi in order to get precise image 
        }
        break;
    case 1: 
        //Serial.println(angleRead); 
        //Serial.println(distanceRead);  
        if(angleRead != 149){  
          Serial.println("MADE IT! AngleRead: ");
          Serial.println(angleRead); 
          Serial.println(distanceRead);
          angleToSpin = PI * (float) angleRead / (float) 180;
          goalAngle = angleToSpin;
          goalAngleAngle = (goalAngle*(float) wheelbase/ (float) diametermodspin ) - (float) 0.08; //wheelbas*wheelbase/r
          state = 2;
          IL = 0;
          IR = 0;
          angularPositionL = 0;
          angularPositionR = 0; 
        }  
        Serial.println("End of case 1");
        break;
        
    case 2:          
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          // START START START CONTROL SYSTEM FOR TURNING angleToSpin 
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
          Serial.println("Case 2");
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

          if(directiongoing > 0){
            if((((rL-yL))<0.1) || ((yL-rL)>0.1)){
              uL = uL - KiLs*IL;
              loganCSdone = true;
             
            }
            if((((rR-yR))<0.1) || ((yR-rR)>0.1)){
              uR = uR - KiRs*IR;
              loganCSdone = true;
             
            }
         }
   
         if(directiongoing < 0){
            if((((rL-yL))<0.1) || ((yL-rL)>0.1)){
            uL = uL - KiLs*IL;
            loganCSdone = true;
             
            }
            if((((rR-yR))<0.1) || ((yR-rR)>0.1)){
              uR = uR - KiRs*IR;
              loganCSdone = true;
              
            }
          }

          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          // END END END CONTROL SYSTEM FOR TURNING angleToSpin
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 

          if (loganCSdone) {  
            loganCSdone = false;
            state = 3;
            IL = 0;
            IR = 0;
            angularPositionL = 0;
            angularPositionR = 0;
            
          }
          motorSpeedL = (speedL*uL* (float) -1);
          motorSpeedR = (speedR*uR* (float) -1);
          break; 
          
   case 3: 
          Serial.println("Case 3");
          motorSpeedL = 0;
          motorSpeedR = 0;   
          if(distanceRead != 0){ ////////////////////////////////////////// fix me fixme ^((^^(^69696969696969696696966996696969696969696969696969696969696need to know when pi has correct disatnc
            distanceToGo = ( distanceRead / (float) 12 ) - (float) 12; // Uncommented this line
            goalDistanceAngle = (float) 1*distanceToGo* (float) 2/ (float) diametermod;
            state = 4;
          }
          break;

   case 4: 
          Serial.println("Case 4"); 
          Serial.println("Goal Distance Angle: ");
          Serial.println(goalDistanceAngle);
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

          if((((rL-yL))<0.1) || ((yL-rL)>0.1)){
            uL = uL - KiL*IL;
            loganCSdone = true;
          }
          if((((rR-yR))<0.1) || ((yR-rR)>0.1)){
            uR = uR - KiR*IR;
            loganCSdone = true;
          }

          uL = uL*directiongoing*((float) -1);
          uR = uR*directiongoing;
          
          motorSpeedL = (speedL*uL* (float) -1);
          motorSpeedR = (speedR*uR* (float) -1);

           //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          // END END END START START START CONTROL SYSTEM FOR DRIVING TO distanceToGo 
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
           if (loganCSdone) {  
            loganCSdone = false;
            state = 5;///////////////////////////////////////////////////////////////////////////if demo 2 just make this state 7
            IL = 0;
            IR = 0;
            angularPositionL = 0;
            angularPositionR = 0;   
          } 
          break;  
          
   case 5:
          //hardcoded to spin 90 degrees to the right 
          turn90LinearPosDest = ( PI / (float) 2 ) * ( (float) wheelbase / (float) 2);
          motorSpeedL = -80;
          motorSpeedR = 80; 

          turn90LinearPosOld = linearPositionL;  
          while( (linearPositionL - turn90LinearPosOld) != turn90LinearPosDest);
          motorSpeedL = 0;
          IL = 0;
          IR = 0;
          state = 0;
          angularPositionL = 0;
          angularPositionR = 0; 
          break;
         
               
   case 6:
  
            motorSpeedL = 80; 
            motorSpeedR =  - (float) 2.3*motorSpeedL;     // Hardcode to turn in circle, scale is the difference in circumference for each wheel 
         // state = 6;    
        
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
  Serial.println("R E C E I V I N G");
  int i = 0;
  while (Wire.available()) {
    dataFromPi[i] = Wire.read();
    i++;
  }
  piState = dataFromPi[1];
  angleRead = dataFromPi[2] - 31;
  distanceRead = dataFromPi[3]; 
  Serial.println(piState);
  Serial.println(angleRead);
  Serial.println(distanceRead);
}

//Callback for sending data to Pi
void sendData() {
  Wire.write(state);
}
