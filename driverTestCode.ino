// Noah Karow March 27, 2025
// 
// PEK got much of this from Noah's github. PEK changing motor driver code for directionality
/* Things PEK changed: organized some set up code,
 added all motor driver pins and their pin mode settings,
 uncommented the if/else logic around the accelerometer setup,
 removed "=0" from global variable initializations so vars would not be overwritten,
 added to pidTerm >/< 0 logic so all directions will change,

*/
//Sensor Input
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

// ****************PINS**********************************************************
// potentiometer pins
const int analogIn = A0; // potentiometer 1. blue
const int analogIn2 = A1; // Potentiometer 2. white

// motor pins. these are all digital. driver 1 controls pins 1 & 4, driver 2 controls coils 2 & 3
const int forwardDriver1 = 2; //turquoise. DIR pin to control the direction of the motor (clockwise/counter-clockwise)
const int reverseDriver1 = 3; // blue. driver 1
const int pwmDriver1 = 4; // orange. driver 1, this is the PWM pin for the motor for how much we move it to correct for its error

const int forwardDriver2 = 5; //driver 2, turquoise 
const int reverseDriver2 = 6;// driver 2, blue
const int pwmDriver2 = 7; // driver 2, orange


// ****************VARIABLES**********************************************************// VARIABLES
float ref_pos = 1;
double A_r1 = 1;
double A_r2 = 1;

//PID
double setpoint, input;
// double count = 0; //set the counts of the encoder
// double angle = 0;//set the angles
// boolean A,B;
// byte state, statep;

double Kp = 1;
double Ki = 0;
double Kd = 0;

float last_error;
float error;
float changeError;
float totalError;
float pidTerm;
float pidTerm_scaled;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
long prev_time;

void Angle2Position()
{

}

void PIDcalculation(){
  //angle = (0.9 * count); //count to angle conversion
  error = setpoint - input;
  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_error = error;
}

void setup() {

  Serial.begin(115200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,changeXAngle,changeYAngle,changeZAngle,Time");
  delay(500); //Do I need this?
  
  // LSM6DSO
  Wire.begin();
  delay(10);

  if( myIMU.begin(0x6A,Wire) )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

  // inputs and outputs
  pinMode(analogIn, INPUT); // blue potentiometer
  pinMode(analogIn2, INPUT);  // orange potentiometer
  
  pinMode(pwmDriver1, OUTPUT);
  pinMode(forwardDriver1, OUTPUT);
  pinMode(reverseDriver1, OUTPUT);
  pinMode(pwmDriver2, OUTPUT);
  pinMode(forwardDriver2, OUTPUT);
  pinMode(reverseDriver2, OUTPUT);

}

void loop() {
  A_r1 = (analogRead(analogIn))/1024; // calculating amplitude of position (cm). *****Needs to change based on actual max/min 
  A_r2 = analogRead(analogIn2)/2048; // calculating freq (Hz) of travelling wave **** could change but probs not necessary
  ref_pos = A_r1 * sin(2*M_PI*A_r2*millis()); //ref sine wave
  setpoint = ref_pos;

  //LSM6DSO
  //Get all parameters
  long prev_time = millis();
  float xAccel1 = myIMU.readFloatAccelX(); //these were the numbers that were showing up when I was attached to the wrong communication ports
  float yAccel1 = myIMU.readFloatAccelY();
  float zAccel1 = myIMU.readFloatAccelZ();

  float xGyro1 = myIMU.readFloatGyroX();
  float yGyro1 = myIMU.readFloatGyroY();
  float zGyro1 = myIMU.readFloatGyroZ();

  delay(10);

  float xAccel2 = myIMU.readFloatAccelX(); 
  float yAccel2 = myIMU.readFloatAccelY();
  float zAccel2 = myIMU.readFloatAccelZ();

  float xGyro2 = myIMU.readFloatGyroX();
  float yGyro2 = myIMU.readFloatGyroY();
  float zGyro2 = myIMU.readFloatGyroZ();

  // change in values
  float changeXAccel = xAccel2 - xAccel1;
  float changeYAccel = yAccel2 - yAccel1;
  float changeZAccel = zAccel2 - zAccel1;

  float changeXGyro = xGyro2 - xGyro1;
  float changeYGyro = yGyro2 - yGyro1;
  float changeZGyro = zGyro2 - zGyro1;

  //change in angular position
  long time = millis()-prev_time;
  float changeXAngle = changeXGyro * time;
  float changeYAngle = changeYGyro * time;
  float changeZAngle = changeZGyro * time;
  input = changeXAngle;

  PIDcalculation();// find PID value
  
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

  analogWrite(pwmDriver1, pidTerm_scaled);
  analogWrite(pwmDriver2, pidTerm_scaled);

  Serial.println( (String) "DATA,"+ changeXAngle + "," + changeYAngle + "," + changeZAngle + "," + millis());


  // delay(100);

  //print values

  // Serial.print("\nAccelerometer:\n");
  // Serial.print(" X = ");
  // Serial.println(myIMU.readFloatAccelX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(myIMU.readFloatAccelY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(myIMU.readFloatAccelZ(), 3);

  // Serial.print("\nGyroscope:\n");
  // Serial.print(" X = ");
  // Serial.println(myIMU.readFloatGyroX(), 3);
  // Serial.print(" Y = ");
  // Serial.println(myIMU.readFloatGyroY(), 3);
  // Serial.print(" Z = ");
  // Serial.println(myIMU.readFloatGyroZ(), 3);

  // Serial.print("\nThermometer:\n");
  // Serial.print(" Degrees F = ");
  // Serial.println(myIMU.readTempF(), 3);
  
  delay(100);
  



}