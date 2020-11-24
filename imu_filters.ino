//#include <Wire.h>
//#include <LSM6.h>
#include "imu.h"

//LSM6 imu;

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

float acceleration;
float vel = 0;
float pos = 0;
float time_difference = 100;

void setup()
{
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  Serial.begin(9600);
  Serial.setTimeout(100);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("***RESET***");

  Imu::initialiseIMU();

  digitalWrite( L_DIR_PIN, HIGH );
  digitalWrite( R_DIR_PIN, HIGH );

  analogWrite( L_PWM_PIN, 64 );
  analogWrite( R_PWM_PIN, 64 );

  
}

void loop()
{
  //  Serial.print(Imu::getImu()->getAxEmaFiltered());
  //  Serial.print(", ");
  //  Serial.println(Imu::getImu()->getAx());

  //  Imu::getImu()->getAx();
  //  Serial.print(Imu::getImu()->getCurrentPosX());
  //  Serial.print(", ");
  //  Serial.print(Imu::getImu()->getCurrentSpeedX() * 1000);
  //  Serial.print(", ");
  //  Serial.println(Imu::getImu()->getCurrentAccelerationX() * 1000);

  //  acceleration = (Imu::getImu()->getAx() / 1000) * 9.80665;
  //  vel = vel + (time_difference / 1000) * acceleration;
  //  pos = pos + (time_difference / 1000) * vel;

  //  if (pos > 0.1) {
  //    while (1) {
  //      delay(500);
  //    }
  //  }

  Imu::getImu() -> calcHeading();


  //  delay(time_difference);
}
