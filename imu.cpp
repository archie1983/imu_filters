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

  //imuHardware->enableDefault();
  reconfigureAcc(afss, aaaf, asr);
  reconfigureGyro(gfss, gsr);

  // No idea why this is needed, but it's what they do in the library code when configuring the defaults after both Acc and Gyro registers have been written.
  // IF_INC = 1 (automatically increment register address)
  imuHardware->writeReg(LSM6::CTRL3_C, 0x04);

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
   * G-H filter to filter IMU values. G = 0.5; H = 0.1; Alpha = 0.2 but will not be used here.
   */
  imu_acc_filter_gh = new Gh_filter_c(0.5, 0.1, 0.2);

  /**
   * Kalman filter to filter IMU values.
   */
  imu_acc_filter_Kalman = new TrivialKalmanFilter<float>(DT_COVARIANCE_RK, DT_COVARIANCE_QK);

  /**
   * This is position as calculated using acc values with no filtering.
   * At the beginning we're going to be at point 0.
   */
  posX_nf = 0.;

  /**
   * And speed and acceleration is 0.
   * This is acceleration and speed as calculated using no filtering on acc values.
   */
  curAcceleration_X_nf = 0.;
  curSpeed_X_nf = 0.;

  /**
   * This is position as calculated using G-H filtered acc values.
   * At the beginning we're going to be at point 0.
   */
  posX_gh = 0.;

  /**
   * And speed and acceleration is 0.
   * This is acceleration and speed as calculated using G-H filtered acc values.
   */
  curAcceleration_X_gh = 0.;
  curSpeed_X_gh = 0.;

  /**
   * This is position as calculated using Kalman filtered acc values.
   * At the beginning we're going to be at point 0.
   */
  posX_k = 0.;

  /**
   * And speed and acceleration is 0.
   * This is acceleration and speed as calculated using Kalman filtered acc values.
   */
  curAcceleration_X_k = 0.;
  curSpeed_X_k = 0.;

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

     We want raw values for accelerometer outputs because accelerometer converstion 
     factors are all between 0 and 1.

     We're trying to apply EMA on the greatest value- whether that's raw or
     converted. That should give better effect of EMA.
  */
//  prev_Ax_ema_val = getAxRaw() - aXZero;
  prev_Ax_ema_val = aXZero;
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
    return getAxRawCompensated() * acc_sensitivity_conversion_factor;
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
   Returns Accelerometer's X axis value raw.
*/
float Imu::getAxRaw() {
  return imuHardware->a.x;
}

/**
   Returns Accelerometer's X axis value raw.
*/
float Imu::getAxRawCompensated() {
  float ax = imuHardware->a.x;
//  return ax;
  if (ax > aXZero_max) {
    return ax - aXZero_max - aXZero;
  } else if (ax < aXZero_min) {
    return ax - aXZero_min - aXZero;
  } else {
    return 0;
  }
}

/**
 * Returns the maximum calibration value seen.
 */
float Imu::getAxZero_max() {
  return aXZero_max;
}

/**
 * Returns the minimum calibration value seen.
 */
float Imu::getAxZero_min() {
  return aXZero_min;
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

    float current_EMA_val = getAxRawCompensated() * ACC_ALPHA_4_EMA + (1 - ACC_ALPHA_4_EMA) * prev_Ax_ema_val;
    prev_Ax_ema_val = current_EMA_val;

    return current_EMA_val * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's X axis value converted to mg with EMA filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAxEmaFiltered() {
  return getAxEmaFiltered(true);
}

/**
   Returns Accelerometer's X axis value converted to mg with Kalman filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAxKalmanFiltered(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }

    return imu_acc_filter_Kalman->update(getAxRawCompensated()) * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Returns Accelerometer's X axis value converted to mg with G-h filtering applied on the raw value
   before multiplying with the conversion factor.
*/
float Imu::getAxGhFiltered(bool readHW) {
  if (imuHardware != NULL) {
    if (readHW) {
      readSensorIfNeeded();
    }

    return imu_acc_filter_gh->apply_filter(getAxRawCompensated()) * acc_sensitivity_conversion_factor;
  } else {
    return 0.;
  }
}

/**
   Reads the sensor via I2C bus if the time since last read has been long enough.
*/
void Imu::readSensorIfNeeded() {
  /**
     If our hardware is not NULL and either gyro or accelerometer is due for a read, then read.
  */
  long time_now = micros();
  long time_diff = time_now - time_since_last_read;
  if (imuHardware != NULL && (time_diff > acc_refresh_time_us || time_diff > gyro_refresh_time_us)) {
//    toggle_led();
//    long now_t = micros();
//    Serial.print("B: ");
//    Serial.print(now_t);
    imuHardware->readAcc();
//    Serial.print(" A: ");
//    Serial.print(micros() - now_t);
//    Serial.print(" # ");
//    Serial.println(imuHardware->a.x);

    /**
       If we're reading in a new batch of values, then we may want to update our pose based on that
    */
    updatePosition(time_diff);

    time_since_last_read = time_now;
  }
}

/**
 * Returns number of microsedonds after which a new value will be available. We want to
 * update IMU position after this many us have elapsed.
 */
unsigned int Imu::getAccRefreshTime() {
  return acc_refresh_time_us;
}

float prevAx = 0;
float curAx = 0;
void Imu::updatePosition(float time_diff) {
  if (motor_is_running) {
    /**
      WARNING Doing getAx(true) or getAx(true) here or getAxEmaFiltered(true) is risky, because if we're slow enough, we could
      enter eternal recursion. So pass false into those functions.
    */
    //curAx = getAx(false);
    //curAcceleration_X = (((curAx + prevAx) / 2) / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2
    //prevAx = curAx;

    curAx = getAxKalmanFiltered(false);
    curAcceleration_X_k = (curAx / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2
    curSpeed_X_k = curSpeed_X_k + (time_diff / US_IN_1_S) * curAcceleration_X_k; //# converting acceleration to the speed change in m/s and adding to the speed
    posX_k = posX_k + (time_diff / US_IN_1_S) * curSpeed_X_k; //# converting position to m

    curAx = getAxGhFiltered(false);
    curAcceleration_X_gh = (curAx / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2
    curSpeed_X_gh = curSpeed_X_gh + (time_diff / US_IN_1_S) * curAcceleration_X_gh; //# converting acceleration to the speed change in m/s and adding to the speed
    posX_gh = posX_gh + (time_diff / US_IN_1_S) * curSpeed_X_gh; //# converting position to m

    curAx = getAx(false);
    curAcceleration_X_nf = (curAx / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2
    curSpeed_X_nf = curSpeed_X_nf + (time_diff / US_IN_1_S) * curAcceleration_X_nf; //# converting acceleration to the speed change in m/s and adding to the speed
    posX_nf = posX_nf + (time_diff / US_IN_1_S) * curSpeed_X_nf; //# converting position to m
  }
}

/**
   Current immediate values for speed and acceleration
*/
float Imu::getCurrentSpeedX(EstimateType et) {
  switch (et) {
    case NO_FILTERED:
      return curSpeed_X_nf;
      break;
    case GH_FILTERED:
      return curSpeed_X_gh;
      break;
    case KALMAN_FILTERED:
      return curSpeed_X_k;
      break;
  }
}

float Imu::getCurrentAccelerationX(EstimateType et) {
  switch (et) {
    case NO_FILTERED:
      return curAcceleration_X_nf;
      break;
    case GH_FILTERED:
      return curAcceleration_X_gh;
      break;
    case KALMAN_FILTERED:
      return curAcceleration_X_k;
      break;
  }
}

float Imu::getCurrentPosX(EstimateType et) {
  switch (et) {
    case NO_FILTERED:
      return posX_nf;
      break;
    case GH_FILTERED:
      return posX_gh;
      break;
    case KALMAN_FILTERED:
      return posX_k;
      break;
  }
}

float Imu::getCurrentPosXmm(EstimateType et) {
  switch (et) {
    case NO_FILTERED:
      return posX_nf * 1000;
      break;
    case GH_FILTERED:
      return posX_gh * 1000;
      break;
    case KALMAN_FILTERED:
      return posX_k * 1000;
      break;
  }
}

void Imu::toggle_led() {
  digitalWrite(YELLOW_LED, led_state);
  led_state = !led_state;
}

void Imu::calibrateAllReadings() {
  long long totalAx = 0.;

  aXZero = 0;

  aXZero_min = 10000;
  aXZero_max = -10000;

  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imuHardware->readAcc();
    
    /**
     * getting minimum value seen during calibration
     */
    if (aXZero_min > imuHardware->a.x) {
      aXZero_min = imuHardware->a.x;
    }

    /**
     * getting max value seen during calibration
     */
    if (aXZero_max < imuHardware->a.x) {
      aXZero_max = imuHardware->a.x;
    }
    
    delay((max(gyro_refresh_time_us, acc_refresh_time_us) / 1000.0) + 2);
  }
  Serial.print(aXZero_min);
  Serial.print(" ,");
  Serial.println(aXZero_max);
  /**
   * Finding the middle value between the highest and lowest seen
   */
  aXZero = (aXZero_max + aXZero_min) / 2;

  /**
   * Now find the mean value for all readings above axZero and below axZero.
   */
  aXZero_max = 0.;
  aXZero_min = 0.;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imuHardware->readAcc();
    totalAx += imuHardware->a.x;
    
    /**
     * getting minimum value seen during calibration
     */
    if (imuHardware->a.x > aXZero) {
      aXZero_max += imuHardware->a.x;
    }

    /**
     * getting max value seen during calibration
     */
    if (imuHardware->a.x < aXZero) {
      aXZero_min += imuHardware->a.x;
    }
    
    delay((max(gyro_refresh_time_us, acc_refresh_time_us) / 1000.0) + 2);
  }
  
  aXZero_max = aXZero_max / CALIBRATION_ITERATIONS;
  aXZero_min = aXZero_min / CALIBRATION_ITERATIONS;


  aXZero = 0; //# switch off the previous style calibration
  aXZero_min = -617; // -315; //# overriding with experimental value
  aXZero_max = 900; //1474; // 315; //# overriding with experimental value
}

/**
   Call this from the setup loop to initialise the IMU sensors.
*/
void Imu::initialiseIMU() {
  if (imu == NULL) {
    pinMode(YELLOW_LED, OUTPUT);
    //toggle_led();
    imu = new Imu(Imu::AccFullScaleSelection::AFS_2,
                  Imu::AccAntiAliasFilter::AA_200,
                  Imu::AccSampleRate::ASR_416,
                  Imu::GyroFullScaleSelection::GFS_2000,
                  Imu::GyroSampleRate::GSR_416);
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
    curAcceleration_X_nf = 0.;
    curAcceleration_X_gh = 0.;
    curAcceleration_X_k = 0.;
    
    curSpeed_X_nf = 0.;
    curSpeed_X_gh = 0.;
    curSpeed_X_k = 0.;
  }
}

void Imu::setZeroPos(bool recalib) {
  curAcceleration_X_nf = 0.;
  curAcceleration_X_gh = 0.;
  curAcceleration_X_k = 0.;
  
  curSpeed_X_nf = 0.;
  curSpeed_X_gh = 0.;
  curSpeed_X_k = 0.;

  posX_nf = 0.;
  posX_gh = 0.;
  posX_k = 0.;

  imu_acc_filter_gh->init_params();
  //imu_acc_filter_Kalman->reset();

  /*
   * And re-calibrate it too if needed
   */
  if (recalib) {
    calibrateAllReadings();
  }
}
