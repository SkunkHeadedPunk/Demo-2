
//arduino running as a fsm.
// lowercase variables are data that pi gives though arduino can modify, ie set to 0.  upperacase are variables that arduino gives
// 0: Innitialized and starts spinning 180 degrees to one direction. ifdetectedaruco = false;  ifaccuratereading = false; ifaccurateangle = false; angletospin = 0;  ifdistancefound = false; distancetogo = 0; 
// 0: If detected aruco robot stops. ifdetectedaruco = true;
// 1: PI gets more accurate reading.  ifaccurateangle = true; angletospin = bits of angle not sure;
// 2: Arduino spins back to correct angle then goes to state 3;
// 3: PI reads distance to aruco and passes ifdistancefound = true; distancetogo = distance to go not sure;
// 4: Arduino goes to aruco and once reaches aruco goes to state 4;
// 5: at the marker.
// *Note for setting up the  PI on a fsm simply pass pass the value of switch somewhere through to the pi.

//not sure about this but just thinking I don't think we can’t do that thing where we just pass the aruco for the final demo, because there is the possibility of 2 or more arucos in frame and unsure of angle, not sure about this though
// if we do it like the exmaple it ensures that there is never more then one aruco code in frame at any one time.

void setup() {
  state = 0;
}




void loop() {

/*Jeffery need to get values for each of these here. I assume easiest to bitwise and to isolate and shift >> for each bool.  for each angle i assume similar
ifdetectedaruco = fixme;  //ie (big_32_Byte_Value & 001) >>1)  i assume something like this but not sure how to convert the value from pi to here.
ifaccuratereading = fixme; // Could be replaced by just checking if the array is populated
*****ifaccurateangle = fixme; // For this demo, the same as ifaccuratereading
*****ifdistancefound = fixme; 
ifArucoLocalized = fixme; // Signals that the distance/angle have been determined
angletospin = fixme;
distancetogo = fixme;
*/

  switch state
  {
    case 0:
        //need to put code in here that tells robot to spin 360 degrees
        if(ifdetectedaruco = true){
          state = 1;
        }
        break;

        
    case 1:
        //Needs to immediatly set all motor voltage to 0 here.
        if(ifaccurateangle = true){
          state = 2;
        }
        break;

        
    case 2:
        //spins to accurate angle faceing the aruco
        // above commadns IFFACINGaruco = true if facing corresct angle.
        
        if(IFFACINGaruco = true){
          state = 3;
        }
        break;

    case 3:
      //note to Logan agiain sets all voltage and u valus to zero.
      if(ifdistancefound = true){
        state = 4:
      }
      break;

    case 4:
        //arduino runs to the correct distance and when at correct distance sends to pi IFFACINGaruco;
        state = 5;
       break;
       
    case 5:
        //set everything in the motor to 0 here. //also set everyhthing from state 0 to false here in the fsm in the PI. *note for building this out for the final demo simply add one more state that spins to right 90degrees then circles aruco and the 
  }


}
