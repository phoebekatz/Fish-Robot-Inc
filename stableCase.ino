/*
PEK, Noah Karow
Apr 2, 2025
Senior Design Proj: Fish Robot Inc

Stability in center
main change: directions of the motor driver are the same as each other so that the forces end up against each other or opposite each other
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

const int forwardDriver1 = 4; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 5; // blue. driver 1
const int pwmDriver1 = 3; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error

const int forwardDriver2 = 12; //driver 2, turquoise 
const int reverseDriver2 = 13;// driver 2, blue
const int pwmDriver2 = 11; // driver 2, orange

void setup() 
{

  Serial.begin(115200);
  delay(50); 
  
  // Accelerometer
  Wire.begin();
  delay(10);

  if( myIMU.begin(0x6B,Wire) )
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
  
  pinMode(pwmDriver1, OUTPUT);
  pinMode(forwardDriver1, OUTPUT);
  pinMode(reverseDriver1, OUTPUT);
  pinMode(pwmDriver2, OUTPUT);
  pinMode(forwardDriver2, OUTPUT);
  pinMode(reverseDriver2, OUTPUT);

  // PWM
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // COM: enables non-inverted PWM, WGM: selects fast pwm
  TCCR2B = _BV(CS22); // prescaler: 64 -> freq ~ 1kHz. **********CHANGE depending on inductance of coils
  OCR2A = 0; // initially off. motor driver 2
  OCR2B = 0; // initially off. motor driver 1
  Serial.println("PWM 1 = 0");
  Serial.println("PWM 2 = 0");

}

void loop() 
{
  float x = myIMU.readFloatAccelX(); 
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();

  float roll = (atan2(y , z) * 57.3) - 90;
  Serial.print("Roll initial is ");
  Serial.println(roll);

  //******************Variables******************************************************
  //int i=1;

  int time = 500;
  int pidTerm = -255;// any number between -255 and 255. This moves the fish to the right
  int pidTerm_scaled = abs(pidTerm); // must be pos to go to pwm out


//***********************Motor Driver Control****************************************
  if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: forward. 
  {
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
  } 
  else  // coils 1 & 4: Reverse. coils 2 & 3: reverse
  {
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
    
  }

  OCR2A = pidTerm_scaled; // motor driver 2 set to appropriate value
  OCR2B = pidTerm_scaled; // motor driver 1 set to appropriate value
  
  delay(time);
}
