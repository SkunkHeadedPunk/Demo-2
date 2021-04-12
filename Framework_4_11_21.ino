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

#define demo              1 // WHICH PART OF THE DEMO THE CODE IS FOR 
#define diameter          0.5 // Diameter of the wheel, in ft (PRE-TAPE)
#define countsPerRotation 3200 
#define motorRPWM         10
#define motorLPWM         9 
#define voltageRDir       7   // Direction for Left motor
#define voltageLDir       8   // Direction for Right motor
#define pinD2             4  

Encoder rightWheel(2,5);  // 2 is an interrupt pin (A/ Yellow Right)    ***DON'T USE PIN 1***
Encoder leftWheel(3,6);   // 3 is an interrupt pin (A/ Yellow Left)   
DualMC33926MotorShield md;

int state = 0;
int piState = 0;  
bool loganCSdone;
float angleToSpin; // Angle to spin, in (degrees or rads)   
float angleRead = 360;    // Angle taken from the Pi 
float distanceToGo; // Needed distance, in feet ***SENT FROM PI, read is redundant
float distanceRead = 0; // Distance from the cam to the marker, in feet
float angularPositionL;
float linearPositionL;
float angularPositionR; 
float linearPositionR;
float radPerCount;
float circumference; 
float speedR;   // Built-in variable for the speed of the motor (Right), Between -400, 400
float speedL;   // Built-in variable for the speed of the motor (Left), Between -400, 400
float motorVoltageR;    
float motorVoltageL;
long oldPositionL = -999; // Included in the basic encoder example code
long oldPositionR = -999;

void setup() {
  circumference = (float) PI * diameter; // Calculates wheel circumference in ft
}

void loop() {
  // set variables equal to jeffrey here 
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

  motorVoltageL = speedL * ( (float) 7 / (float) 400 );   // Carried over from Demo 1
  motorVoltageR = speedR * ( (float) 7 / (float) 400 );

  Serial.print("LinearPositionL: ");
  Serial.print(linearPositionL);  
  Serial.print("   motorVoltageL: ");
  Serial.print(motorVoltageL);

  Serial.print(" LinearPositionR: ");
  Serial.print(linearPositionR);
  Serial.print("   motorVoltageR: ");
  Serial.println(motorVoltageR);
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
        //Robot spins slowly until detecting Aruco 
        // HARDCODE TO TURN AT A CERTAIN SPEED UNTIL DETECTED 
        if (piState == 1){
          md.setM1Speed(0);  // Immediately stops left motor
          md.setM2Speed(0);  // Immediately stops right motor
          state = 1; 
          // send state value to Pi in order to get precise image
        }
        break;
        
    case 1:
        if(angleRead != 360) {     
          distanceToGo = distanceRead / (float) 12;  // 0.9 is distance from the beacon to cam after move
          angleToSpin = PI * (float) angleRead / (float) 180;
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          // CONTROL SYSTEM FOR TURNING angleToSpin
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
          if (loganCSdone) {  
            loganCSdone = false;
            state = 2;   
          } 
          break; 
          
   case 2:
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          // CONTROL SYSTEM FOR DRIVING TO distanceToGo 
          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
           if (loganCSdone) {  
            loganCSdone = false;
            state = 3;   
          } 
          break;  
           
   case 3:
          if (demo == 2) { 
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // CONTROL SYSTEM FOR TURNING AROUND MARKER
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          } 
          state = 4;
        }
        break;
       
    case 4: 
        md.setM1Speed(0);
        md.setM2Speed(0); // Stops motors 
        // Doesn't need to update booleans or anything like that since that's automatically
        // done on the Pi side
        break; 
  }


}
