//#include <Wire.h>
//#include <LSM6.h>
#include "imu.h"

//LSM6 imu;

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(100);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("***RESET***");

  Imu::initialiseIMU();
  Imu::getImu()->calibrateGx();
  //Wire.begin();
  //
  //  if (!imu.init())
  //  {
  //    Serial.println("Failed to detect and initialize IMU!");
  //    while (1);
  //  }
  //  imu.enableDefault();
}

void loop()
{
  //Serial.print("A: ");
  //  Serial.print(Imu::getImu()->getAx());
  //  Serial.print(" ");
  //  Serial.print(Imu::getImu()->getAy());
  //  Serial.print(" ");
  //  Serial.print(Imu::getImu()->getAz());
  //  imu.read();
  //
  ////  Serial.print("A: ");
  ////  Serial.print(imu.a.x);
  ////  Serial.print(" ");
  ////  Serial.print(imu.a.y);
  ////  Serial.print(" ");
  ////  Serial.print(imu.a.z);
  //  Serial.print("\t G:  ");
  //  Serial.print(imu.g.x);
  //  Serial.print(" ");
  //  Serial.print(imu.g.y);
  //  Serial.print(" ");
  //  Serial.println(imu.g.z);
  //  Imu::getImu()->readAllAxis();
  Serial.println(Imu::getImu()->getGx());

  delay(100);
}
