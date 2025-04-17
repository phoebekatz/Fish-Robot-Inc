/*
MEMS Senior Design Expo Code

Fish Robot Inc
authors: PEK, Noah Karow, Parra Wang
Apr 14, 2025

Use switches to change between different modes:
0. off
1. oscillating. velocity and force/amplitude of oscillation changes wrt to potentiometer changes
2. position 
*/

// ************************LIBRARIES*****************************************
// put libraries here
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//*************************PINS**********************************************
const int interruptPin = 2; // digital pin 3

const int forwardDriver1 = 4; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 9; // blue. driver 1
const int pwmDriver1 = 11; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error

const int forwardDriver2 = 12; //driver 2, turquoise 
const int reverseDriver2 = 13;// driver 2, blue
const int pwmDriver2 = 3; // driver 2, orange

const int potPinblue = A0; // amplitude and position
const int potPinwhite = A1; // frequency and stiffness

const int blueLEDs = 7; // mode 2 indicators
const int whiteLEDs = 8; // mode 1 indicators

//************************VARIABLES / OBJECTS *******************************
#define BNO055_ADDRESS 0x28    // I2C address of IMU
#define SERIAL_BAUD_RATE 115200
#define LOOP_RATE_HZ 100       // Control loop frequency
#define LOOP_PERIOD_MS (1000 / LOOP_RATE_HZ)
double CONSTANT_FORCE=255;
double CONSTANT_FORCE_FACTOR=CONSTANT_FORCE/10;

volatile int mode = 0; //indicates which mode the fish is operating

double setpoint, input;

double Kp = 1;
double Ki = 0;
double Kd = 0;

float last_error;
float error;
float changeError;
float totalError;
float pidTerm;
float pidTerm_scaled;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|

double potValblue = 0;
double potValwhite = 0;

unsigned long lastTime = 0;

float sineValue;  // Current sine wave value

// Wave parameters
float frequency;  // Hz
float amplitude; // degrees

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);

// **************************** FUNCTIONS ********************************
void PIDcalculation(void);
void MD_one(int,int);
void MD_two(int,int);
void MD_control(float,float);
void MD_controlSine(float);

void setup() 
{
    Serial.begin(SERIAL_BAUD_RATE);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("starting");
   
  delay(500);



  // inputs and outputs
  pinMode(potPinblue, INPUT); // blue potentiometer
  pinMode(potPinwhite, INPUT);  // white potentiometer
  
  pinMode(pwmDriver1, OUTPUT);
  pinMode(forwardDriver1, OUTPUT);
  pinMode(reverseDriver1, OUTPUT);
  pinMode(pwmDriver2, OUTPUT);
  pinMode(forwardDriver2, OUTPUT);
  pinMode(reverseDriver2, OUTPUT);

  pinMode(blueLEDs, OUTPUT);
  pinMode(whiteLEDs, OUTPUT);

  pinMode(interruptPin, INPUT);

  digitalWrite(blueLEDs, HIGH);
  digitalWrite(whiteLEDs, HIGH);

  // PWM
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // COM: enables non-inverted PWM, WGM: selects fast pwm
  TCCR2B = _BV(CS22); // prescaler: 64 -> freq ~ 1kHz.
  OCR2A = 0; // initially off. motor driver 2
  OCR2B = 0; // initially off. motor driver 1

  attachInterrupt(digitalPinToInterrupt(interruptPin),modeChange,FALLING); // Will trigger when the signal drops from HIGH to LOW
  //attachInterrupt(digitalPinToInterrupt(interruptPinT),onAndOff,FALLING); // Will trigger when the signal drops from HIGH to LOW

}

void loop() 
{
  if (mode == 0)
  {
    OCR0A = 0; // initially off. motor driver 2
    OCR0B = 0; // initially off. motor driver 1
  }
  else if (mode == 1)
  {
    Serial.println("Mode 1");
    amplitude = map(analogRead(potPinblue), 0, 1023, 1, 10) ;// Scale to calibrated range

    frequency = map(analogRead(potPinwhite), 0, 1023, 1, 15) ;// Scale to calibrated range

    // Get value from table
    float t = millis();

    sineValue = amplitude* sin(frequency*t/1000);
 
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    double angle = euler.x();
    if (angle > 300) // actuator has rotated counterclockwise
    { 
      angle = angle - 360;
    }

    MD_controlSine(sineValue);


    Serial.print("Sine");
    Serial.print(sineValue);
    Serial.print("Ang:");
    Serial.print(angle,1);

  }
  else if (mode == 2)
  {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;
    
    // Skip iteration if time delta too small
    if (dt < 0.001) return;
    setpoint = map(
      analogRead(potPinblue), 
      0, 1023, 
      -10, 10) ;// Scale to calibrated range

    CONSTANT_FORCE= map(
      analogRead(potPinwhite), 
      0, 1023, 
      0, 255) ;// Scale to calibrated range

    Serial.print("Stiffness");
    Serial.println(CONSTANT_FORCE);
  
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    double angle = euler.x();

    if (angle > 300) // actuator has rotated counterclockwise
    { 
      angle = angle - 360;
    }
    input = angle;

    PIDcalculation();

    Serial.print("Error: ");
    Serial.print(error);

    MD_control(pidTerm_scaled,setpoint);

    Serial.print("Set:");
    Serial.print(setpoint);
    Serial.print("° Ang:");
    Serial.print(angle,1);
    Serial.print("° Vel:");

    // Maintain constant loop rate
    delay(max(0, LOOP_PERIOD_MS - (millis() - currentTime)));
  }
  else
  {
    mode = 0; // again, this code should never run. but in case it does, coils will stop receiving power
  }

}

void modeChange() //ISR 1
{
  delay(50); // debounce switch

  Serial.println("interrupted");

  if (mode == 0)
  {
    mode = 1; 
    
    digitalWrite(blueLEDs, HIGH); // blue turn off. active low
    digitalWrite(whiteLEDs, LOW); // orange turn on. active low
  }
  else if (mode == 1)
  {
    mode = 2;

    digitalWrite(blueLEDs, LOW); // blue turned on. active low
    digitalWrite(whiteLEDs, HIGH); // orange turn off. active low
  }
  else if (mode == 2) 
  {
    mode = 1;

    digitalWrite(blueLEDs, HIGH); // blue turn off. active low
    digitalWrite(whiteLEDs, LOW); // orange turn on. active low
  }
  else // this should never have to run, but just in case, the actuator will turn off
  {
    mode = 0;
  }
  
}

void PIDcalculation()
{
  error = setpoint - input;
  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;

}

void MD_one(int direction, int value) //coils 1 and 4
{

  if (direction == 1){ //go right
    digitalWrite(forwardDriver1, LOW); // coils 1 & 4
    digitalWrite(reverseDriver1, HIGH);  // coils 1 & 4
  }
  else{ //go left
    digitalWrite(forwardDriver1, HIGH); // coils 1 & 4
    digitalWrite(reverseDriver1, LOW);  // coils 1 & 4
  }
  OCR2B = value;
}

void MD_two(int direction, int value) //coils 2 and 3
{ 

  if (direction == 1){ //go right
    digitalWrite(forwardDriver2, HIGH); // coils 2 & 3
    digitalWrite(reverseDriver2, LOW);  // coils 2 & 3
  }
  else{ //go left
    digitalWrite(forwardDriver2, LOW); // coils 2 & 3
    digitalWrite(reverseDriver2, HIGH);  // coils 2 & 3
  }
  OCR2A = value;
}

void MD_control(float pidTerm,float direction)
{
 if (direction > 0) // coils 1 & 4: Forward. coils 2 & 3: Reverse
  {
    // Go right
    MD_one(1,CONSTANT_FORCE);
    MD_two(0,pidTerm);

  } 
  else if (direction ==0){
    // Go to the center
    MD_one(1,CONSTANT_FORCE);
    MD_two(0,CONSTANT_FORCE);
  }
  else  // coils 1 & 4: Reverse. coils 2 & 3: Forward
  {
    // Go left
    MD_one(1,pidTerm);
    MD_two(0,CONSTANT_FORCE);
  }
}

void MD_controlSine(float pidTerm)
{
  if (pidTerm > 0) // coils 1 & 4: Forward. coils 2 & 3: Reverse
  {
    // Go right
    MD_one(1,CONSTANT_FORCE-(CONSTANT_FORCE_FACTOR*pidTerm));
    MD_two(0,CONSTANT_FORCE);

  } 
  // else if (direction ==0){
  //   // Go to the center
  //   MD_one(1,CONSTANT_FORCE);
  //   MD_two(0,CONSTANT_FORCE);
  // }
  else  // coils 1 & 4: Reverse. coils 2 & 3: Forward
  {
    // Go left
    MD_one(1,CONSTANT_FORCE);
    MD_two(0,CONSTANT_FORCE+(CONSTANT_FORCE_FACTOR*pidTerm));
  }
}
