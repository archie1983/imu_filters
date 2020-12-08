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
   * We'll be using this to filter acc values.
   */
  gh_filter = new Gh_filter_c(0.5, 0.1, 0.2);

  /**
     At the beginning we're going to be at point (0, 0).
  */
  posX = 0.;

  /**
     And speed and acceleration is 0.
  */
  curAcceleration_X = 0.;
  curSpeed_X = 0.;

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
    //curAx = getFilteredAx();
    curAx = getAx(false);
    //curAcceleration_X = (((curAx + prevAx) / 2) / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2
    curAcceleration_X = (curAx / 1000.0) * GRAVITY_CONSTANT; //# converting mg to m/s^2

    prevAx = curAx;

//    /**
//       Dropping noise
//    */
//    if (abs(curAcceleration_X) < 0.05) {
//      curAcceleration_X = 0;
//    }

    curSpeed_X = curSpeed_X + (time_diff / US_IN_1_S) * curAcceleration_X; //# converting acceleration to the speed change in m/s and adding to the speed
//
//    if (abs(curSpeed_X) < 0.01) {
//      curSpeed_X = 0;
//    }

    posX = posX + (time_diff / US_IN_1_S) * curSpeed_X; //# converting position to m
  }
}

/**
 * Returns an Acc X axis reading filtered with some filter (for now- with G-H filter).
 */
float Imu::getFilteredAx() {
  /**
      WARNING Doing getAx(true) or getAx(true) here or getAxEmaFiltered(true) is risky, because if we're slow enough, we could
      enter eternal recursion. So pass false into those functions.
  */
  float gh_filtered_value = gh_filter->apply_filter(getAxRawCompensated());
  float curAcceleration_X = (gh_filtered_value * acc_sensitivity_conversion_factor); //# converting to mg

  return curAcceleration_X;
}

/**
   Current immediate values for speed and acceleration
*/
float Imu::getCurrentSpeedX() {
  return curSpeed_X;
}

float Imu::getCurrentAccelerationX() {
  return curAcceleration_X;
}

float Imu::getCurrentPosX() {
  return posX;
}

float Imu::getCurrentPosXmm() {
  return posX * 1000;
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
  aXZero_min = -315; //# overriding with experimental value
  aXZero_max = 315; //# overriding with experimental value
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
    curAcceleration_X = 0.;
    curSpeed_X = 0.;
  }
}

void Imu::setZeroPos(bool recalib) {
  curAcceleration_X = 0.;
  curSpeed_X = 0.;

  posX = 0.;

  gh_filter->init_params();

  /*
   * And re-calibrate it too if needed
   */
  if (recalib) {
    calibrateAllReadings();
  }
}
