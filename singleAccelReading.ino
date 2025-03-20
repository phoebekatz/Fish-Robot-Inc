/******************************************************************************
Single accelerometer (LSM6DSO breakout) readings
PEK
March 5, 2025
based on Sparkfun example code 

Reads acceleration and gyroscope data in twice, finds the difference, and prints it.
******************************************************************************/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

void setup() {


  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);
  if( myIMU.begin(0x6B, Wire) ) //default address (107 = 0x6b)
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

}


void loop()
{
  // Get all parameters
  float xAccel1 = myIMU.readFloatAccelX() - 1.994; //these were the numbers that were showing up when I was attached to the wrong communication ports
  float yAccel1 = myIMU.readFloatAccelY() - 1.994;
  float zAccel1 = myIMU.readFloatAccelZ() - 1.994;

  float xGyro1 = myIMU.readFloatGyroX() - 286.073;
  float yGyro1 = myIMU.readFloatGyroY()- 286.073;
  float zGyro1 = myIMU.readFloatGyroZ()- 286.073;

  delay(1000);

  float xAccel2 = myIMU.readFloatAccelX()- 1.994; 
  float yAccel2 = myIMU.readFloatAccelY()- 1.994;
  float zAccel2 = myIMU.readFloatAccelZ()- 1.994;

  float xGyro2 = myIMU.readFloatGyroX()- 286.073;
  float yGyro2 = myIMU.readFloatGyroY()- 286.073;
  float zGyro2 = myIMU.readFloatGyroZ()- 286.073;

  // change in values
  float changeXAccel = xAccel2 - xAccel1;
  float changeYAccel = yAccel2 - yAccel1;
  float changeZAccel = zAccel2 - zAccel1;

  float changeXGyro = xGyro2 - xGyro1;
  float changeYGyro = yGyro2 - yGyro1;
  float changeZGyro = zGyro2 - zGyro1;  

  
  //print changes
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(changeXAccel, 10); //3 is the number of decimal places
  Serial.print(" Y = ");
  Serial.println(changeYAccel, 10);
  Serial.print(" Z = ");
  Serial.println(changeZAccel, 10);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(changeXGyro,10);
  Serial.print(" Y = ");
  Serial.println(changeYGyro,10);
  Serial.print(" Z = ");
  Serial.println(changeZGyro,10);
  
  delay(5000);
}
