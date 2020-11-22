#include "imu.h"
#include <Wire.h>

bool Imu::led_state = false;
Imu* Imu::imu = NULL; //# for now set this to NULL. We'll initialise it when initialiseIMU() is called.

/**
   Private constructor because we only have 1 IMU which doesn't change.
*/
Imu::Imu(AccFullScaleSelection afss, AccAntiAliasFilter aaaf, AccSampleRate asr, GyroFullScaleSelection gfss, GyroSampleRate gsr) {
  imuHardware = new LSM6();
  Wire.begin();

  if (!imuHardware->init())
  {
    //Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }

  acc_refresh_time_us = US_IN_1_S;
  gyro_refresh_time_us = US_IN_1_S;

  imuHardware->enableDefault();
  reconfigureAcc(afss, aaaf, asr);
  reconfigureGyro(gfss, gsr);

  /**
   * Calibrate
   */
  calibrateAllReadings();

  /**
   * Initialising EMA values with current readings for IMU sensor outputs.
   * 
   * We want raw values for accelerometer outputs and converted values for
   * gyroscope, because accelerometer converstion factors are all between
   * 0 and 1 and gyroscope conversion factors are all greater than 1.
   * 
   * We're trying to apply EMA on the greatest value- whether that's raw or
   * converter. That should give better effect of EMA.
   */
  time_since_last_read = 0;
  prev_Ax_ema_val = getAxRaw() - aXZero;
  prev_Ay_ema_val = getAyRaw() - aYZero;
  prev_Az_ema_val = getAzRaw() - aZZero;
  prev_Gx_ema_val = getGx();
  prev_Gy_ema_val = getGy();
  prev_Gz_ema_val = getGz();

  /**
   * At the beginning we're going to be at point (0, 0).
   */
  posX = 0.;
  posY = 0.;

  /**
   * And speed and acceleration is 0.
   */
  curAcceleration_X = 0.;
  curAcceleration_Y = 0.;
  curSpeed_X = 0.;
  curSpeed_Y = 0.;

  /**
   * And motor is not running.
   */
  motor_is_running = false;
}

/**
   Changes the configuration of the IMU.
*/
void Imu::reconfigureAcc(Imu::AccFullScaleSelection afss, Imu::AccAntiAliasFilter aaaf, Imu::AccSampleRate asr) {
  imuHardware->writeReg(LSM6::CTRL1_XL, (asr << 4) | (afss << 2) | (aaaf));

  /**
   * Choose accelerometer conversion factor
   */
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

  /**
   * Choose refresh time.
   */
  acc_refresh_time_us = getAccRefreshRate(asr);
}

/**
   Changes the configuration of the Gyroscope.
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

  /**
   * Choose refresh time.
   */
  gyro_refresh_time_us = getGyroRefreshRate(gsr);
}

/**
 * Returns how much time we can allow between measurement updates.
 */
unsigned int Imu::getAccRefreshRate(AccSampleRate asr) {
  unsigned long tmp_refresh_time_us = 0;
  switch (asr) {
    case ASR_OFF:
      tmp_refresh_time_us = US_IN_1_S;
      break;
    case ASR_125:
      tmp_refresh_time_us = US_IN_1_S / 12.5; //# for 12.5Hz we need to divide 1s worth of us into 12.5 chunks.
      break;
    case ASR_26:
      tmp_refresh_time_us = US_IN_1_S / 26.0; //# for 26Hz we need to divide 1s worth of us into 26 chunks.
      break;
    case ASR_52:
      tmp_refresh_time_us = US_IN_1_S / 52.0; //# for 52Hz we need to divide 1s worth of us into 52 chunks.
      break;
    case ASR_104:
      tmp_refresh_time_us = US_IN_1_S / 104.0; //# for 104Hz we need to divide 1s worth of us into 104 chunks.
      break;
    case ASR_208:
      tmp_refresh_time_us = US_IN_1_S / 208.0; //# for 208Hz we need to divide 1s worth of us into 208 chunks.
      break;
    case ASR_416:
      tmp_refresh_time_us = US_IN_1_S / 416.0; //# for 416Hz we need to divide 1s worth of us into 416 chunks.
      break;
    case ASR_833:
      tmp_refresh_time_us = US_IN_1_S / 833.0; //# for 833Hz we need to divide 1s worth of us into 833 chunks.
      break;
    case ASR_166k:
      tmp_refresh_time_us = US_IN_1_S / 1660.0; //# for 1.66kHz we need to divide 1s worth of us into 1660 chunks.
      break;
    case ASR_333k:
      tmp_refresh_time_us = US_IN_1_S / 3330.0; //# for 3.33Hz we need to divide 1s worth of us into 3330 chunks.
      break;
    case ASR_666k:
      tmp_refresh_time_us = US_IN_1_S / 6660.0; //# for 6.66kHz we need to divide 1s worth of us into 6660 chunks.
      break;
  }
//  Serial.print("ASR: ");
//  Serial.println(asr);
//  Serial.print("T: ");
//  Serial.print(tmp_refresh_time_us);
  return tmp_refresh_time_us;
}

/**
 * Returns how much time we can allow between measurement updates.
 */
unsigned int Imu::getGyroRefreshRate(GyroSampleRate gsr) {
  unsigned int tmp_refresh_time_us = 0;
  switch (gsr) {
    case GSR_OFF:
      tmp_refresh_time_us = US_IN_1_S;
      break;
    case GSR_125:
      tmp_refresh_time_us = US_IN_1_S / 12.5; //# for 12.5Hz we need to divide 1s worth of us into 12.5 chunks.
      break;
    case GSR_26:
      tmp_refresh_time_us = US_IN_1_S / 26.0; //# for 26Hz we need to divide 1s worth of us into 26 chunks.
      break;
    case GSR_52:
      tmp_refresh_time_us = US_IN_1_S / 52.0; //# for 52Hz we need to divide 1s worth of us into 52 chunks.
      break;
    case GSR_104:
      tmp_refresh_time_us = US_IN_1_S / 104.0; //# for 104Hz we need to divide 1s worth of us into 104 chunks.
      break;
    case GSR_208:
      tmp_refresh_time_us = US_IN_1_S / 208.0; //# for 208Hz we need to divide 1s worth of us into 208 chunks.
      break;
    case GSR_416:
      tmp_refresh_time_us = US_IN_1_S / 416.0; //# for 416Hz we need to divide 1s worth of us into 416 chunks.
      break;
    case GSR_833:
      tmp_refresh_time_us = US_IN_1_S / 833.0; //# for 833Hz we need to divide 1s worth of us into 833 chunks.
      break;
    case GSR_166k:
      tmp_refresh_time_us = US_IN_1_S / 1660.0; //# for 1.66kHz we need to divide 1s worth of us into 1660 chunks.
      break;
  }
  return tmp_refresh_time_us;
}

/**
   Static getter for this class
*/
Imu* Imu::getImu() {
  return imu;
}

/**
   Returns Accelerometer's X axis value converted to mg.
*/
float Imu::getAx() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();
    return (getAxRaw() - aXZero)  * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's Y axis value converted to mg.
*/
float Imu::getAy() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();
    return (getAyRaw() - aYZero) * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's Z axis value converted to mg.
*/
float Imu::getAz() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();
    return (getAzRaw() - aZZero) * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's X axis value converted to mdps.
*/
float Imu::getGx() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();
    return getGxRaw() * gyro_sensitivity_conversion_factor - gXZero;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's Y axis value converted to mdps.
*/
float Imu::getGy() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();
    return getGyRaw() * gyro_sensitivity_conversion_factor - gYZero;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's Z axis value converted to mdps.
*/
float Imu::getGz() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();
    return getGzRaw() * gyro_sensitivity_conversion_factor - gZZero;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's X axis value raw.
*/
float Imu::getAxRaw() {
  return imuHardware->a.x;
}

/**
   Returns Accelerometer's Y axis value raw.
*/
float Imu::getAyRaw() {
  return imuHardware->a.y;
}

/**
   Returns Accelerometer's Z axis value raw.
*/
float Imu::getAzRaw() {
  return imuHardware->a.z;
}

/**
   Returns Gyroscope's X axis value raw.
*/
float Imu::getGxRaw() {
  return imuHardware->g.x;
}

/**
   Returns Gyroscope's Y axis value raw.
*/
float Imu::getGyRaw() {
  return imuHardware->g.y;
}

/**
   Returns Gyroscope's Z axis value raw.
*/
float Imu::getGzRaw() {
  return imuHardware->g.z;
}

/**
   Returns Accelerometer's X axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAxEmaFiltered() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();

    float current_EMA_val = ((getAxRaw() - aXZero) * ACC_ALPHA_4_EMA) + (1 - ACC_ALPHA_4_EMA) * prev_Ax_ema_val;
    prev_Ax_ema_val = current_EMA_val;
    
    return current_EMA_val * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's Y axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAyEmaFiltered() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();

    float current_EMA_val = ((getAyRaw() - aYZero) * ACC_ALPHA_4_EMA) + (1 - ACC_ALPHA_4_EMA) * prev_Ay_ema_val;
    prev_Ay_ema_val = current_EMA_val;
    
    return current_EMA_val * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's Z axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAzEmaFiltered() {
  if (imuHardware != NULL) {
    readSensorIfNeeded();

    float current_EMA_val = ((getAzRaw() - aZZero)  * ACC_ALPHA_4_EMA) + (1 - ACC_ALPHA_4_EMA) * prev_Az_ema_val;
    prev_Az_ema_val = current_EMA_val;
    
    return current_EMA_val * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's X axis value converted to mdps, EMA filtered.
*/
float Imu::getGxEmaFiltered() {
  float current_EMA_val = (getGx() * GYRO_ALPHA_4_EMA) + (1 - GYRO_ALPHA_4_EMA) * prev_Gx_ema_val;
  prev_Gx_ema_val = current_EMA_val;
  return current_EMA_val;
}

/**
   Returns Gyroscope's Y axis value converted to mdps, EMA filtered.
*/
float Imu::getGyEmaFiltered() {
  float current_EMA_val = (getGy() * GYRO_ALPHA_4_EMA) + (1 - GYRO_ALPHA_4_EMA) * prev_Gy_ema_val;
  prev_Gy_ema_val = current_EMA_val;
  return current_EMA_val;
}

/**
   Returns Gyroscope's Z axis value converted to mdps, EMA filtered.
*/
float Imu::getGzEmaFiltered() {
  float current_EMA_val = (getGz() * GYRO_ALPHA_4_EMA) + (1 - GYRO_ALPHA_4_EMA) * prev_Gz_ema_val;
  prev_Gz_ema_val = current_EMA_val;
  return current_EMA_val;
}

/**
 * Reads the sensor via I2C bus if the time since last read has been long enough.
 */
void Imu::readSensorIfNeeded() {
  /**
   * If our hardware is not NULL and either gyro or accelerometer is due for a read, then read.
   */
  if (imuHardware != NULL && (micros() - time_since_last_read > acc_refresh_time_us || micros() - time_since_last_read > gyro_refresh_time_us)) {
    toggle_led();
    imuHardware->read();
    time_since_last_read = micros();
  }
}

/**
 * Take in a fresh reading for each initialised sensor.
 */
static void Imu::readAllAxis() {
  toggle_led();
  Serial.print(getAx());
  Serial.print(", ");
  Serial.print(getAy());
  Serial.print(", ");
  Serial.println(getAz());
  Serial.print(", ");
  Serial.print(getGx());
  Serial.print(", ");
  Serial.print(getGy());
  Serial.print(", ");
  Serial.println(getGz());
}

void Imu::toggle_led() {
  digitalWrite(YELLOW_LED, led_state);
  led_state = !led_state;
}

void Imu::calibrateAllReadings() {
  long long totalAx = 0.;
  long long totalAy = 0.;
  long long totalAz = 0.;
  long long totalGx = 0.;
  long long totalGy = 0.;
  long long totalGz = 0.;

  aXZero = 0;
  aYZero = 0;
  aZZero = 0;
  gXZero = 0;
  gYZero = 0;
  gZZero = 0;

//Serial.println(gyro_refresh_time_us);
//Serial.println(acc_refresh_time_us);
//Serial.println(US_IN_1_S / 12.5);
  
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imuHardware->read();
//    imuHardware->g.x * gyro_sensitivity_conversion_factor;
    delay(max(gyro_refresh_time_us, acc_refresh_time_us) / 1000.0);
  }
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imuHardware->read();
    totalAx += imuHardware->a.x;
    totalAy += imuHardware->a.y;
    totalAz += imuHardware->a.z;
    totalGx += imuHardware->g.x * gyro_sensitivity_conversion_factor;
    totalGy += imuHardware->g.y * gyro_sensitivity_conversion_factor;
    totalGz += imuHardware->g.z * gyro_sensitivity_conversion_factor;
    delay(max(gyro_refresh_time_us, acc_refresh_time_us) / 1000.0);
  }
  gXZero = totalGx / CALIBRATION_ITERATIONS;
  gYZero = totalGy / CALIBRATION_ITERATIONS;
  gZZero = totalGz / CALIBRATION_ITERATIONS;
  aXZero = totalAx / CALIBRATION_ITERATIONS;
  aYZero = totalAy / CALIBRATION_ITERATIONS;
  aZZero = totalAz / CALIBRATION_ITERATIONS;
}

/**
   Call this from the setup loop to initialise the IMU sensors.
*/
void Imu::initialiseIMU() {
  if (imu == NULL) {
    pinMode(YELLOW_LED, OUTPUT);
    //toggle_led();
    imu = new Imu(Imu::AccFullScaleSelection::AFS_8,
                  Imu::AccAntiAliasFilter::AA_50,
                  Imu::AccSampleRate::ASR_125,
                  Imu::GyroFullScaleSelection::GFS_2000,
                  Imu::GyroSampleRate::GSR_104);
  }
}


/**
 * A method to tell IMU class when the motor starts running and when it ends.
 */
void Imu::setMotorRunning(boolean motor_running) {
  motor_is_running = motor_running;

  /**
   * If we're stopping the motor, then nullify the speed and acceleration too.
   */
  if (!motor_running) {
    float curAcceleration_X;
    float curAcceleration_Y;
    float curSpeed_X;
    float curSpeed_Y;
  }
}
