/*
PEK
Fish Robot Inc
March 5, 2025
Magnetometer (MLX 90393) basic code
*/

#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 magnetometer = Adafruit_MLX90393();

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) 
  {
    delay(100);
  }

  Serial.println("Starting magnetomeasurements ...");
 if (magnetometer.begin_I2C(0x18,&Wire) )
  {
    Serial.println("Found a MLX90393 sensor");
  }
  //gain set to default (1x)
  //resolution set to default (maximum is automatic)
  magnetometer.setOversampling(MLX90393_OSR_3); //set oversampling
  magnetometer.setFilter(MLX90393_FILTER_5); //set digital filtering
}

void loop() 
{
  sensors_event_t data;
  magnetometer.getEvent(&data); // stored in uTesla

  Serial.print("X: "); Serial.print(data.magnetic.x); Serial.println(" uT");
  Serial.print("Y: "); Serial.print(data.magnetic.y); Serial.println(" uT");
  Serial.print("Z: "); Serial.print(data.magnetic.z); Serial.println(" uT");
 
  delay(1000);

}
