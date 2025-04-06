/*
PEK, Noah Karow
Apr 5, 2025
Senior Design Proj: Fish Robot Inc

This code wiggles the fish
open loop

USING BNO055 (new sensor. uses euler angle x)
also prints out angle and position (simplified equation). 
has the updated driver direction settings, based on updated wiring
*/

// Libraries
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// object: 9 DOF sensor
Adafruit_BNO055 sensor = Adafruit_BNO055(55, 0x28, &Wire); //I2C, addr 0x28

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
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,changeXAngle,changeYAngle,changeZAngle,Time");
  while (!Serial) delay(10);  // wait for serial port to open
  
  // Accelerometer
  if (!sensor.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // inputs and outputs
  //pinMode(analogIn, INPUT); // blue potentiometer
  //pinMode(analogIn2, INPUT);  // orange potentiometer
  
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
  // imu::Vector<3> euler = sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  // double angle = euler.x();

  // if (angle > 300) // actuator has rotated counterclockwise
  // { 
  //   angle = angle - 360;
  // }
  // /* Display the floating point data */
  // //Serial.print("Initial angle: ");
  // Serial.println(angle);

  // float position = -0.1947*roll;
  // Serial.print("initial linear position is ");
  // Serial.println(position);
  
  //******************Variables******************************************************
  //int i=1;

  int time = 5000;
  int pidTerm = 75;// any number between -255 and 255
  int pidTerm_scaled = abs(pidTerm); // must be pos to go to pwm out


//***********************Motor Driver Control****************************************
  if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: forward
  {
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  } 
  else  // coils 1 & 4: Reverse. coils 2 & 3: reverse
  {
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
    
  }

  OCR2A = pidTerm_scaled; // motor driver 2 set to appropriate value
  OCR2B = pidTerm_scaled; // motor driver 1 set to appropriate value
  
  delay(time/2);

  imu::Vector<3> euler = sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  double angle = euler.x();

  if (angle > 300) // actuator has rotated counterclockwise
  { 
    angle = angle - 360;
  }
  /* Display the floating point data */
  //Serial.print("Initial angle: ");
  Serial.println(angle);

  delay(time/2);

//***********************Motor Driver Control****************************************

 pidTerm = -pidTerm;// any number between -255 and 255
 pidTerm_scaled = abs(pidTerm); // must be pos to go to pwm out

 if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: forward
  {
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
    
   digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  } 
  else  // coils 1 & 4: Reverse. coils 2 & 3: reverse
  {
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
    
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
    
  }

  OCR2A = pidTerm_scaled; // motor driver 2 set to appropriate value
  OCR2B = pidTerm_scaled; // motor driver 1 set to appropriate value

delay(time/2);
//*******************ANGLE**************************************
  euler = sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle = euler.x();

  if (angle > 300) // actuator has rotated counterclockwise
  { 
    angle = angle - 360;
  }
   
  /* Display the floating point data */
  //Serial.print("Final angle: ");
  Serial.println(angle);

  delay(time/2);
  // position = -0.1947*roll;
  // Serial.print("final linear position is ");
  // Serial.println(position);

  
  }
