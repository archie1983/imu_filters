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
  delay(10);
  reconfigureAcc(afss, aaaf, asr);
  delay(10);
  reconfigureGyro(gfss, gsr);

  // Wait for IMU readings to stabilize.
  delay(1000);

  time_since_last_read = 0;

  /**
     Calibrate
  */
  calibrateAllReadings();

  /**
     Initialising EMA values with current readings for IMU sensor outputs.
  */
  initialiseEmaValues();

  /**
     At the beginning we're going to be at point (0, 0).
  */
  posX = 0.;
  posY = 0.;

  /**
     And speed and acceleration is 0.
  */
  curAcceleration_X = 0.;
  curAcceleration_Y = 0.;
  curSpeed_X = 0.;
  curSpeed_Y = 0.;

  /**
     And motor is not running.
  */
  motor_is_running = false;

  heading = 0;
  previous_time = millis();
}

/**
 * Set the EMA values to the current readings so that if we're starting EMA values again,
 * we're not influenced by previous run.
 */
void Imu::initialiseEmaValues() {
  /**
     Initialising EMA values with current readings for IMU sensor outputs.

     We want raw values for accelerometer outputs and converted values for
     gyroscope, because accelerometer converstion factors are all between
     0 and 1 and gyroscope conversion factors are all greater than 1.

     We're trying to apply EMA on the greatest value- whether that's raw or
     converter. That should give better effect of EMA.
  */
//  prev_Ax_ema_val = getAxRaw() - aXZero;
//  prev_Ay_ema_val = getAyRaw() - aYZero;
//  prev_Az_ema_val = getAzRaw() - aZZero;
//  prev_Gx_ema_val = getGx();
//  prev_Gy_ema_val = getGy();
//  prev_Gz_ema_val = getGz();
  prev_Ax_ema_val = aXZero;
  prev_Ay_ema_val = aYZero;
  prev_Az_ema_val = aZZero;
  prev_Gx_ema_val = gXZero;
  prev_Gy_ema_val = gYZero;
  prev_Gz_ema_val = gZZero;
}

/**
   Changes the configuration of the IMU.
*/
void Imu::reconfigureAcc(Imu::AccFullScaleSelection afss, Imu::AccAntiAliasFilter aaaf, Imu::AccSampleRate asr) {
  imuHardware->writeReg(LSM6::CTRL1_XL, (asr << 4) | (afss << 2) | (aaaf));

  /**
     Choose accelerometer conversion factor
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
     Choose refresh time.
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
     Choose refresh time.
  */
  gyro_refresh_time_us = getGyroRefreshRate(gsr);
}

/**
   Returns how much time we can allow between measurement updates.
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
   Returns how much time we can allow between measurement updates.
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
float Imu::getAx(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }
    return (getAxRaw() - aXZero)  * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's Y axis value converted to mg.
*/
float Imu::getAy(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }
    return (getAyRaw() - aYZero) * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's Z axis value converted to mg.
*/
float Imu::getAz(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }
    return (getAzRaw() - aZZero) * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's X axis value converted to mdps.
*/
float Imu::getGx(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }
    return getGxRaw() * gyro_sensitivity_conversion_factor - gXZero;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's Y axis value converted to mdps.
*/
float Imu::getGy(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }
    return getGyRaw() * gyro_sensitivity_conversion_factor - gYZero;
  } else {
    return 0.;
  }
}

/**
   Returns Gyroscope's Z axis value converted to mdps.
*/
float Imu::getGz(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }
    return getGzRaw() * gyro_sensitivity_conversion_factor - gZZero;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's X axis value converted to mg.
*/
float Imu::getAx() {
  return getAx(true);
}

/**
   Returns Accelerometer's Y axis value converted to mg.
*/
float Imu::getAy() {
  return getAy(true);
}

/**
   Returns Accelerometer's Z axis value converted to mg.
*/
float Imu::getAz() {
  return getAz(true);
}

/**
   Returns Gyroscope's X axis value converted to mdps.
*/
float Imu::getGx() {
  return getGx(true);
}

/**
   Returns Gyroscope's Y axis value converted to mdps.
*/
float Imu::getGy() {
  return getGy(true);
}

/**
   Returns Gyroscope's Z axis value converted to mdps.
*/
float Imu::getGz() {
  return getGz(true);
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
float Imu::getAxEmaFiltered(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }

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
float Imu::getAyEmaFiltered(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }

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
float Imu::getAzEmaFiltered(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }

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
float Imu::getGxEmaFiltered(bool readHW) {
  float current_EMA_val = (getGx(readHW) * GYRO_ALPHA_4_EMA) + (1 - GYRO_ALPHA_4_EMA) * prev_Gx_ema_val;
  prev_Gx_ema_val = current_EMA_val;
  return current_EMA_val;
}

/**
   Returns Gyroscope's Y axis value converted to mdps, EMA filtered.
*/
float Imu::getGyEmaFiltered(bool readHW) {
  float current_EMA_val = (getGy(readHW) * GYRO_ALPHA_4_EMA) + (1 - GYRO_ALPHA_4_EMA) * prev_Gy_ema_val;
  prev_Gy_ema_val = current_EMA_val;
  return current_EMA_val;
}

/**
   Returns Gyroscope's Z axis value converted to mdps, EMA filtered.
*/
float Imu::getGzEmaFiltered(bool readHW) {
  float current_EMA_val = (getGz(readHW) * GYRO_ALPHA_4_EMA) + (1 - GYRO_ALPHA_4_EMA) * prev_Gz_ema_val;
  prev_Gz_ema_val = current_EMA_val;
  return current_EMA_val;
}

/**
   Returns Accelerometer's X axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAxEmaFiltered() {
  return getAxEmaFiltered(true);
}

/**
   Returns Accelerometer's Y axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAyEmaFiltered() {
  return getAyEmaFiltered(true);
}

/**
   Returns Accelerometer's Z axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAzEmaFiltered() {
  return getAzEmaFiltered(true);
}

/**
   Returns Gyroscope's X axis value converted to mdps, EMA filtered.
*/
float Imu::getGxEmaFiltered() {
  return getGxEmaFiltered(true);
}

/**
   Returns Gyroscope's Y axis value converted to mdps, EMA filtered.
*/
float Imu::getGyEmaFiltered() {
  return getGyEmaFiltered(true);
}

/**
   Returns Gyroscope's Z axis value converted to mdps, EMA filtered.
*/
float Imu::getGzEmaFiltered() {
  return getGzEmaFiltered(true);
}

/**
   Reads the sensor via I2C bus if the time since last read has been long enough.
*/
void Imu::readSensorIfNeeded() {
  /**
     If our hardware is not NULL and either gyro or accelerometer is due for a read, then read.
  */
  long time_diff = micros() - time_since_last_read;
  if (imuHardware != NULL && (time_diff > acc_refresh_time_us || time_diff > gyro_refresh_time_us)) {
    toggle_led();
    imuHardware->read();

    /**
       If we're reading in a new batch of values, then we may want to update our pose based on that
    */
    updatePosition(time_diff);

    time_since_last_read = micros();
  }
}

float prevAx = 0;
float curAx = 0;
void Imu::updatePosition(float time_diff) {
  if (motor_is_running) {
    /**
      WARNING Doing getAx(true) or getAx(true) here or getAxEmaFiltered(true) is risky, because if we're slow enough, we could
      enter eternal recursion. So pass false into those functions.
    */
    curAx = getAx(false);
    curAcceleration_X = (((curAx + prevAx) / 2) / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2

    prevAx = curAx;

    /**
       Dropping noise
    */
    if (abs(curAcceleration_X) < 0.05) {
      curAcceleration_X = 0;
    }

    curSpeed_X = curSpeed_X + (time_diff / US_IN_1_S) * curAcceleration_X; //# converting acceleration to the speed change in m/s and adding to the speed
//
//    if (abs(curSpeed_X) < 0.01) {
//      curSpeed_X = 0;
//    }

    posX = posX + (time_diff / US_IN_1_S) * curSpeed_X; //# converting position to m

    curAcceleration_Y = (getAyEmaFiltered(false) / 1000.0) * GRAVITY_CONSTANT;
    curSpeed_Y = curSpeed_Y + (time_diff / US_IN_1_S) * curAcceleration_Y;
    posY = posY + (time_diff / US_IN_1_S) * curSpeed_Y;
  }
}

float Imu::getFilteredAx() {
  /**
      WARNING Doing getAx(true) or getAx(true) here or getAxEmaFiltered(true) is risky, because if we're slow enough, we could
      enter eternal recursion. So pass false into those functions.
  */
  float curAcceleration_X = (getAxEmaFiltered(false) / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2

  /**
     Dropping noise
  */
  if (abs(curAcceleration_X) < 0.05) {
    curAcceleration_X = 0;
  }

  return curAcceleration_X;
}

/**
   Current immediate values for speed and acceleration
*/
float Imu::getCurrentSpeedX() {
  return curSpeed_X;
}

float Imu::getCurrentSpeedY() {
  return curSpeed_Y;
}

float Imu::getCurrentAccelerationX() {
  return curAcceleration_X;
}

float Imu::getCurrentAccelerationY() {
  return curAcceleration_Y;
}

float Imu::getCurrentPosX() {
  return posX;
}

float Imu::getCurrentPosY() {
  return posY;
}

/**
   Take in a fresh reading for each initialised sensor.
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
  //gXZero already multiplied by sensitivity
  gXZero = totalGx / CALIBRATION_ITERATIONS;
  gYZero = totalGy / CALIBRATION_ITERATIONS;
  gZZero = totalGz / CALIBRATION_ITERATIONS;
  aXZero = totalAx / CALIBRATION_ITERATIONS;
  aYZero = totalAy / CALIBRATION_ITERATIONS;
  aZZero = totalAz / CALIBRATION_ITERATIONS;
}

float Imu::calcHeading() {
  unsigned long time_now = millis();
  unsigned long delta_t = time_now - previous_time;

  //integrate every 100ms
  if (delta_t > 100) {

    float GyroZ = Imu::getImu() -> getGz();   // GyroZ in milli degree per second

    float GyroZ_dps = GyroZ / 1000;   //    Convert into deg / s
    heading += GyroZ_dps * ((float)delta_t / 1000); // convert delta_t from milli seconds to seconds
//    Serial.println(heading);
    
    previous_time = time_now;
  }

  return heading;
}

/**
   Call this from the setup loop to initialise the IMU sensors.
*/
void Imu::initialiseIMU() {
  if (imu == NULL) {
    pinMode(YELLOW_LED, OUTPUT);
    //toggle_led();
    imu = new Imu(Imu::AccFullScaleSelection::AFS_2,
                  Imu::AccAntiAliasFilter::AA_50,
                  Imu::AccSampleRate::ASR_104,
                  Imu::GyroFullScaleSelection::GFS_2000,
                  Imu::GyroSampleRate::GSR_104);
  }
}

/**
   A method to tell IMU class when the motor starts running and when it ends.
*/
void Imu::setMotorRunning(boolean motor_running) {
  motor_is_running = motor_running;

  /**
   * We don't want previous EMA values now.
   */
  initialiseEmaValues();
  time_since_last_read = micros() - max(gyro_refresh_time_us, acc_refresh_time_us);

  /**
     If we're stopping the motor, then nullify the speed and acceleration too.
  */
  if (!motor_running) {
    curAcceleration_X = 0.;
    curAcceleration_Y = 0.;
    curSpeed_X = 0.;
    curSpeed_Y = 0.;
  }
}

void Imu::setZeroPos() {
  curAcceleration_X = 0.;
  curAcceleration_Y = 0.;
  curSpeed_X = 0.;
  curSpeed_Y = 0.;  

  posX = 0.;
  posY = 0.;
}
