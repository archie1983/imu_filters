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
unsigned long previous_time;
float heading;
float previous_GyroZ_dps;

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
  Imu::getImu()->calibrateGx();
  Imu::getImu()->calibrateGz();

  digitalWrite( L_DIR_PIN, HIGH );
  digitalWrite( R_DIR_PIN, HIGH );

  analogWrite( L_PWM_PIN, 64 );
  analogWrite( R_PWM_PIN, 64 );

  // Initialise global variables
  previous_GyroZ_dps = 0;
  heading = 0;
  previous_time = millis();
}

void loop()
{
  //Serial.println(Imu::getImu()->getGx());

  unsigned long time_now = millis();
  unsigned long delta_t = time_now - previous_time;

  if (delta_t > 100) {

    float GyroZ = Imu::getImu() -> getGz();
    //Serial.print(GyroZ);
    // Serial.print(",");
    // Convert into deg/s
    float GyroZ_dps = GyroZ / 1000;
 //Serial.print(GyroZ_dps);
 //Serial.print(",");
//    float difference_rateOfchange_Z = previous_GyroZ_dps - GyroZ_dps;
//    Serial.print(difference_rateOfchange_Z);
//    Serial.print(",");
    heading += GyroZ_dps * ((float)delta_t / 1000);
    Serial.println(heading);

    //previous_GyroZ_dps = GyroZ_dps;
    previous_time = time_now;
  }



  //  acceleration = (Imu::getImu()->readAx() / 1000) * 9.80665;
  //  vel = vel + (time_difference / 1000) * acceleration;
  //  pos = pos + (time_difference / 1000) * vel;
  //
  //  if (pos > 0.1) {
  //    while (1) {
  //      delay(500);
  //    }
  //  }
  //
  //  delay(time_difference);
}
