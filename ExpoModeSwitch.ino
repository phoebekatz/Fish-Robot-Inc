/*
MEMS Senior Design Expo Code

Fish Robot Inc
authors: PEK, Noah Karow, Parra Wang
Apr 14, 2025

Uses a switch to change between different modes:
0. off
1. oscillating. velocity and force/amplitude of oscillation changes wrt to potentiometer changes
2. position 
*/

// ************************LIBRARIES*****************************************
// put libraries here

//*************************PINS**********************************************
const int interruptPin = 2; # digital pin 2
// put other pins here

//************************VARIABLES / OBJECTS *******************************

volatile int mode = 0; //indicates which mode the fish is operating
// but global variables and IMU object here


void setup() 
{

  // put your setup code here, to run once:

  
  attachInterrupt(digitalPinToInterrupt(interruptPin),modeChange,FALLING); // Will trigger when the signal drops from HIGH to LOW

}

void loop() 
{
  if (mode == 0)
  {
    // turn off motor drivers. PWM = 0
  }
  else if (mode == 1)
  {
    // put oscillating code here
    // potentiometer to control delay
    // potentiometer to control magnitude (force / amplitude)
  }
  else if (mode == 2)
  {
    // put force changing code here
  }
  else
  {
    mode = 0; // again, this code should never run. but in case it does, coils will stop receiving power
  }

}

void modeChange()
{
  delay(50); // debounce switch

  if (mode < 2)
  {
    mode ++;
  }
  if (mode == 2) 
  {
    mode = 0;
  }
  else // this should never have to run, but just in case, the actuator will turn off
  {
    mode = 0;
  }
  
}
