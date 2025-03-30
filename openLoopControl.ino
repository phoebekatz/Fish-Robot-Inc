/*
PEK, Noah Karow
Mar 28, 2025
Senior Design Proj: Fish Robot Inc

This code will allow us to do open loop control, as we test out accelerometer and motor drivers
Diff from other code: no potentiometers, or PID. I also got rid of the roll and pitch function
*/

// Libraries
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"

// object: accelerometer
LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

// ****************PINS**********************************************************
// motor pins are all digital. 
// driver 1 controls pins 1 & 4, driver 2 controls coils 2 & 3

const int forwardDriver1 = 3; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 4; // blue. driver 1
const int pwmDriver1 = 5; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error

const int forwardDriver2 = 11; //driver 2, turquoise 
const int reverseDriver2 = 12;// driver 2, blue
const int pwmDriver2 = 13; // driver 2, orange

void setup() 
{

  Serial.begin(115200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,changeXAngle,changeYAngle,changeZAngle,Time");
  delay(500); //Do I need this?
  
  // Accelerometer
  Wire.begin();
  delay(10);

  if( myIMU.begin(0x6A,Wire) )
  {
    Serial.println("Ready.");
  }
  else 
  { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
  {
    Serial.println("Loaded Settings.");
  }

  // ***********inputs and outputs********** 
  pinMode(pwmDriver1, OUTPUT);
  pinMode(forwardDriver1, OUTPUT);
  pinMode(reverseDriver1, OUTPUT);
  pinMode(pwmDriver2, OUTPUT);
  pinMode(forwardDriver2, OUTPUT);
  pinMode(reverseDriver2, OUTPUT);

}

void loop() 
{
  //******************Variables******************************************************
  
  int pidTerm = 255;// any number between -255 and 255
  int pidTerm_scaled = abs(pidTerm); // must be pos to go to pwm out

  float x = myIMU.readFloatAccelX(); 
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();

  float pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;

//***********************Motor Driver Control****************************************
  if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: Reverse
  {
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  } 
  else  // coils 1 & 4: Reverse. coils 2 & 3: Forward
  {
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
    
  }

  analogWrite(pwmDriver1, pidTerm_scaled); // pin: pwmDriver1. value: pidTerm_scaled
  analogWrite(pwmDriver2, pidTerm_scaled);

  //********************************Value Print-out**********************************
  //Serial.println( (String) "DATA,"+ changeXAngle + "," + changeYAngle + "," + changeZAngle + "," + millis());
  Serial.println(pitch, 5);

  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(x, 5);
  Serial.print(" Y = ");
  Serial.println(y, 5);
  Serial.print(" Z = ");
  Serial.println(z, 5);

  delay(10);
  
}
