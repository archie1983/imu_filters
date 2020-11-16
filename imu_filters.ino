#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
  imu.read();

  Serial.print("A: ");
  Serial.print(imu.a.x);
  Serial.print(" ");
  Serial.print(imu.a.y);
  Serial.print(" ");
  Serial.print(imu.a.z);
  Serial.print("\t G:  ");
  Serial.print(imu.g.x);
  Serial.print(" ");
  Serial.print(imu.g.y);
  Serial.print(" ");
  Serial.println(imu.g.z);

  delay(100);
}
