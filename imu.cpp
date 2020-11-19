#include "imu.h"
#include <Wire.h>

volatile bool Imu::led_state = false;
Imu* Imu::imu = NULL; //# for now set this to NULL. We'll initialise it when initialiseIMU() is called.

/**
 * Private constructor because we only have 1 IMU which doesn't change.
 */
Imu::Imu(AccFullScaleSelection afss, AccAntiAliasFilter aaaf, AccSampleRate asr, GyroFullScaleSelection gfss, GyroSampleRate gsr) {  
  imuHardware = new LSM6();
  Wire.begin();

  if (!imuHardware->init())
  {
    //Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }

  imuHardware->enableDefault();
  reconfigureAcc(afss, aaaf, asr);
  reconfigureGyro(gfss, gsr);
}

/**
 * Changes the configuration of the IMU.
 */
void Imu::reconfigureAcc(Imu::AccFullScaleSelection afss, Imu::AccAntiAliasFilter aaaf, Imu::AccSampleRate asr) {
  imuHardware->writeReg(LSM6::CTRL1_XL, (asr << 4) | (afss << 2) | (aaaf));

  switch (afss) {
    case AFS_2:
      acc_sensitivity_conversion_factor = 0.061;
      break;
    case AFS_4:
      acc_sensitivity_conversion_factor = 0.122;
      break;
    case AFS_8:
      acc_sensitivity_conversion_factor = 0.244;
      break;
    case AFS_16:
      acc_sensitivity_conversion_factor = 0.488;
      break;
  }
}

/**
 * Changes the configuration of the Gyroscope.
 */
void Imu::reconfigureGyro(GyroFullScaleSelection gfss, GyroSampleRate gsr) {
  imuHardware->writeReg(LSM6::CTRL2_G, (gsr << 4) | (gfss << 1) | 0x0);

  switch (gfss) {
    case GFS_125:
      gyro_sensitivity_conversion_factor = 4.375;
      break;
    case GFS_250:
      gyro_sensitivity_conversion_factor = 8.75;
      break;
    case GFS_500:
      gyro_sensitivity_conversion_factor = 17.5;
      break;
    case GFS_1000:
      gyro_sensitivity_conversion_factor = 35;
      break;
    case GFS_2000:
      gyro_sensitivity_conversion_factor = 70;
      break;
  }
}

/**
 * Static getter for this class
 */
Imu* Imu::getImu() {
  return imu;
}

/**
 * Returns Accelerometer's X axis value converted to mg.
 */
float Imu::getAx() {
  return imuHardware->a.x * acc_sensitivity_conversion_factor;
}

/**
 * Returns Accelerometer's Y axis value converted to mg.
 */
float Imu::getAy() {
  return imuHardware->a.y * acc_sensitivity_conversion_factor;
}

/**
 * Returns Accelerometer's Z axis value converted to mg.
 */
float Imu::getAz() {
return imuHardware->a.z * acc_sensitivity_conversion_factor;
}

/**
 * Returns Gyroscope's X axis value converted to mdps.
 */
float Imu::getGx() {
  return imuHardware->g.x * gyro_sensitivity_conversion_factor;
}

/**
 * Returns Gyroscope's Y axis value converted to mdps.
 */
float Imu::getGy() {
  return imuHardware->g.y * gyro_sensitivity_conversion_factor;
}

/**
 * Returns Gyroscope's Z axis value converted to mdps.
 */
float Imu::getGz() {
  return imuHardware->g.z * gyro_sensitivity_conversion_factor;
}

/**
 * Returns Accelerometer's X axis value raw.
 */
float Imu::getAxRaw() {
  return imuHardware->a.x;
}

/**
 * Returns Accelerometer's Y axis value raw.
 */
float Imu::getAyRaw() {
  return imuHardware->a.y;
}

/**
 * Returns Accelerometer's Z axis value raw.
 */
float Imu::getAzRaw() {
return imuHardware->a.z;
}

/**
 * Returns Gyroscope's X axis value raw.
 */
float Imu::getGxRaw() {
  return imuHardware->g.x;
}

/**
 * Returns Gyroscope's Y axis value raw.
 */
float Imu::getGyRaw() {
  return imuHardware->g.y;
}

/**
 * Returns Gyroscope's Z axis value raw.
 */
float Imu::getGzRaw() {
  return imuHardware->g.z;
}

/**
 * Take in a fresh reading for each initialised sensor.
 */
static void Imu::readAllAxis() {
  toggle_led();
  if (imuHardware != NULL) {
    imuHardware->read();
    Serial.print(getAx());
    Serial.print(", ");
    Serial.print(getAy());
    Serial.print(", ");
    Serial.println(getAz());
//    Serial.print(", ");
//    Serial.print(getGx());
//    Serial.print(", ");
//    Serial.print(getGy());
//    Serial.print(", ");
//    Serial.println(getGz());
  }
}

void Imu::toggle_led() {
  digitalWrite(YELLOW_LED, led_state);
  led_state = !led_state;
}

/**
 * Call this from the setup loop to initialise the IMU sensors.
 */
void Imu::initialiseIMU() {
  pinMode(YELLOW_LED, OUTPUT);
  //toggle_led();

  Imu::imu = new Imu(Imu::AccFullScaleSelection::AFS_4, 
                          Imu::AccAntiAliasFilter::AA_50, 
                          Imu::AccSampleRate::ASR_125, 
                          Imu::GyroFullScaleSelection::GFS_2000, 
                          Imu::GyroSampleRate::GSR_104);
}
