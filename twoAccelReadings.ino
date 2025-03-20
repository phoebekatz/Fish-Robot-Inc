/*
Two Accelerometer readings
PEK
March 5, 2025
based on Sparkfun example code (LSM6DSO breakout)

Sets up two accelerometer
Reads acceleration and gyroscope data
Prints it
*/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"

LSM6DSO firstIMU; //addr 0x6B
LSM6DSO secondIMU; //addr 0x6A


void setup() 
{

  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);

  //first IMU set-up
  if( firstIMU.begin(0x6B, Wire) ) //default I2C address (107 = 0x6b)
  {
    Serial.println("One Ready.");
  }
  else 
  { 
    Serial.println("Could not connect to first IMU.");
    Serial.println("Freezing");
  }
  delay (100);

  if( firstIMU.initialize(BASIC_SETTINGS) )
  {
    Serial.println("Loaded Settings for first IMU.");
  }
  delay (10);

  //SECOND IMU SET-UP
   if( secondIMU.begin(0x6A, Wire) ) //alternate I2C address 0x6a
  {
    Serial.println("Two Ready.");
  }
  else 
  { 
    Serial.println("Could not connect to second IMU.");
    Serial.println("Freezing");
  }
  
  if( secondIMU.initialize(BASIC_SETTINGS) )
  {
    Serial.println("Loaded Settings for second IMU.");
  }
}

void loop() 
{
  //first IMU data
  float xAccel1 = firstIMU.readFloatAccelX(); 
  float yAccel1 = firstIMU.readFloatAccelY();
  float zAccel1 = firstIMU.readFloatAccelZ();

  float xGyro1 = firstIMU.readFloatGyroX();
  float yGyro1 = firstIMU.readFloatGyroY();
  float zGyro1 = firstIMU.readFloatGyroZ();

  // second IMU data
  float xAccel2 = secondIMU.readFloatAccelX(); 
  float yAccel2 = secondIMU.readFloatAccelY();
  float zAccel2 = secondIMU.readFloatAccelZ();

  float xGyro2 = secondIMU.readFloatGyroX();
  float yGyro2 = secondIMU.readFloatGyroY();
  float zGyro2 = secondIMU.readFloatGyroZ();

  //print data
  Serial.print("\n1st Accelerometer:\n");
  Serial.print(" X = ");
  Serial.println(xAccel1, 5); //5 is the number of decimal places
  Serial.print(" Y = ");
  Serial.println(yAccel1, 5);
  Serial.print(" Z = ");
  Serial.println(zAccel1, 5);

  Serial.print("\n2nd Accelerometer:\n");
  Serial.print(" X = ");
  Serial.println(xAccel2, 5); //5 is the number of decimal places
  Serial.print(" Y = ");
  Serial.println(yAccel2, 5);
  Serial.print(" Z = ");
  Serial.println(zAccel2, 5);

  Serial.print("\n1st Gyroscope:\n");
  Serial.print(" X = ");
  Serial.println(xGyro1,10);
  Serial.print(" Y = ");
  Serial.println(yGyro1,10);
  Serial.print(" Z = ");
  Serial.println(zGyro1,10);

  Serial.print("\n2nd Gyroscope:\n");
  Serial.print(" X = ");
  Serial.println(xGyro2,10);
  Serial.print(" Y = ");
  Serial.println(yGyro2,10);
  Serial.print(" Z = ");
  Serial.println(zGyro2,10);

  delay(10000); // wait one sec
}
