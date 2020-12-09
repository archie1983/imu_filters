#ifndef _IMU_
#define _IMU_

#include <Arduino.h>
#include <LSM6.h>
#include "pin_names_and_constants.h"
#include "gh_filter.h"
#include <TrivialKalmanFilter.h>

#define DT_COVARIANCE_RK 4.7e-3 // Estimation of the noise covariances (process) for Kalman filter
#define DT_COVARIANCE_QK 1e-5   // Estimation of the noise covariances (observation) for Kalman filter

/**
   Class for IMU
*/
class Imu {
  public:
    /**
       Linear accelerometer sensitivity modes (full scale selection).
    */
    enum AccFullScaleSelection {
      AFS_2 = 0b00, //# full scale is +-2g
      AFS_4 = 0b10, //# full scale is +-4g
      AFS_8 = 0b11, //# full scale is +-8g
      AFS_16 = 0b01 //# full scale is +-16g
    };

    /**
       Anti-aliasing filter bandwidth selection.
    */
    enum AccAntiAliasFilter {
      AA_400 = 0b00, //# Anti-aliasing filter bandwidth = 400Hz
      AA_200 = 0b01, //# Anti-aliasing filter bandwidth = 200Hz
      AA_100 = 0b10, //# Anti-aliasing filter bandwidth = 100Hz
      AA_50 = 0b11 //# Anti-aliasing filter bandwidth = 50Hz
    };

    /**
       Sample rate.
    */
    enum AccSampleRate {
      ASR_OFF = 0b0000, //# Sensor is off
      ASR_125 = 0b0001, //# Output data rate is 12.5Hz
      ASR_26 = 0b0010, //# Output data rate is 26Hz
      ASR_52 = 0b0011, //# Output data rate is 52Hz
      ASR_104 = 0b0100, //# Output data rate is 104Hz
      ASR_208 = 0b0101, //# Output data rate is 208Hz
      ASR_416 = 0b0110, //# Output data rate is 416Hz
      ASR_833 = 0b0111, //# Output data rate is 833Hz
      ASR_166k = 0b1000, //# Output data rate is 1.66kHz
      ASR_333k = 0b1001, //# Output data rate is 3.33kHz
      ASR_666k = 0b1010 //# Output data rate is 6.66kHz
    };

    /**
       Gyroscope sensitivity modes (full scale selection).
    */
    enum GyroFullScaleSelection {
      GFS_125 =  0b001,     //# full rotation speed is 125 degrees per second
      GFS_250 =  0b000,  //# full rotation speed is 250 degrees per second
      GFS_500 =  0b010,  //# full rotation speed is 500 degrees per second
      GFS_1000 = 0b100, //# full rotation speed is 1000 degrees per second
      GFS_2000 = 0b110  //# full rotation speed is 2000 degrees per second
    };

    /**
       Gyroscope Sample rate.
    */
    enum GyroSampleRate {
      GSR_OFF = 0b0000, //# Sensor is off
      GSR_125 = 0b0001, //# Output data rate is 12.5Hz
      GSR_26 = 0b0010, //# Output data rate is 26Hz
      GSR_52 = 0b0011, //# Output data rate is 52Hz
      GSR_104 = 0b0100, //# Output data rate is 104Hz
      GSR_208 = 0b0101, //# Output data rate is 208Hz
      GSR_416 = 0b0110, //# Output data rate is 416Hz
      GSR_833 = 0b0111, //# Output data rate is 833Hz
      GSR_166k = 0b1000 //# Output data rate is 1.66kHz
    };

    /**
     * Estimate types for acceleration, speed and position.
     */
    enum EstimateType {
      NO_FILTERED, //# With no filter applied to the acc
      GH_FILTERED, //# With G-H filter applied to the acc
      KALMAN_FILTERED //# With Kalman filter applied to the acc
    };

    /**
       Changes the configuration of the Accelerometer.
    */
    void reconfigureAcc(AccFullScaleSelection afss, AccAntiAliasFilter aaaf, AccSampleRate asr);

    /**
       Changes the configuration of the Gyroscope.
    */
    void reconfigureGyro(GyroFullScaleSelection gfss, GyroSampleRate gsr);

    /**
       Returns how much time we can allow between measurement updates for accelerometer.
    */
    unsigned int getAccRefreshRate(AccSampleRate asr);

    /**
       Returns how much time we can allow between measurement updates for gyro.
    */
    unsigned int getGyroRefreshRate(GyroSampleRate gsr);

    /**
       Returns Accelerometer's X axis value converted to mg. This function guarantees that hardware will be read.
    */
    float getAx();

    /**
       Returns Accelerometer's X axis value converted to mg with or without reading the hardware.
    */
    float getAx(bool readHW);

    /**
       Returns Accelerometer's X axis value raw.
    */
    float getAxRaw();

    float getAxRawCompensated();

    /**
       Returns Accelerometer's X axis value EMA filtered. This function guarantees that hardware will be read.
    */
    float getAxEmaFiltered();

    /**
       Returns Accelerometer's X axis value EMA filtered with or without reading the hardware.
    */
    float getAxEmaFiltered(bool readHW);

    /**
       Returns Accelerometer's X axis value converted to mg with Kalman filtering applied on the raw value
       before multiplying with the conversion factor.
    */
    float Imu::getAxKalmanFiltered(bool readHW);

    /**
       Returns Accelerometer's X axis value converted to mg with G-h filtering applied on the raw value
       before multiplying with the conversion factor.
    */
    float Imu::getAxGhFiltered(bool readHW);

    /**
       Variables required for applying EMA to the IMU outputs.
    */
    float prev_Ax_ema_val;

    /**
       Static getter for this class
    */
    static Imu* getImu();

    /**
       Call this from the setup loop to initialise the IMU sensors.
    */
    static void initialiseIMU();

    /**
       Calibrate all axis and all gyroscope velocities.
    */
    void calibrateAllReadings();

    /**
       A method to tell IMU class when the motor starts running and when it ends.
    */
    void setMotorRunning(boolean motor_running);

    /**
       Current immediate values for speed and acceleration
    */
    float getCurrentSpeedX(EstimateType et);
    float getCurrentAccelerationX(EstimateType et);
    float getCurrentPosX(EstimateType et);
    float getCurrentPosXmm(EstimateType et);

    /**
     * Returns the maximum calibration value seen.
     */
    float getAxZero_max();

    /**
     * Returns the minimum calibration value seen.
     */
    float getAxZero_min();

    /**
     * Sets pose to (0, 0)
     */
    void setZeroPos(bool recalib);

    /**
     * Returns number of microsedonds after which a new value will be available. We want to
     * update IMU position after this many us have elapsed.
     */
    unsigned int getAccRefreshTime();

  private:
    /**
       Private constructor because we only have 1 IMU which doesn't change.
    */
    Imu(AccFullScaleSelection afss, AccAntiAliasFilter aaaf, AccSampleRate asr, GyroFullScaleSelection gfss, GyroSampleRate gsr);

    /**
       Reference to the singleton imu object.
    */
    static Imu* imu;

    /**
     * We'll be using this to filter acc values.
     */
    Gh_filter_c* imu_acc_filter_gh;

    /**
     * Kalman filter to filter IMU values.
     */
    TrivialKalmanFilter<float> * imu_acc_filter_Kalman;

    /**
       Reference to the hardware.
    */
    LSM6* imuHardware;

    /**
       Accelerometer sensitivity factor - number that we need to multiply the result with to get mg values.
    */
    float acc_sensitivity_conversion_factor;

    /**
       Gyro sensitivity factor - number that we need to multiply the result with to get mdps values.
    */
    float gyro_sensitivity_conversion_factor;

    static bool led_state;
    static void toggle_led();

    /**
       How many microseconds should we aim to get between reads for accelerometer.
    */
    unsigned int acc_refresh_time_us;

    /**
       How many microseconds should we aim to get between reads for gyroscope.
    */
    unsigned int gyro_refresh_time_us;

    /**
       Time in microseconds since our last read.
    */
    long time_since_last_read;
    /**
       Reads the sensor via I2C bus if the time since last read has been long enough.
    */
    void readSensorIfNeeded();

    /**
     * Set the EMA values to the current readings so that if we're starting EMA values again,
     * we're not influenced by previous run.
     */
    void initialiseEmaValues();

    /**
       Variables needed to store calibration value for axis and gyro speeds.
    */
    float aXZero;

    /**
     * When calibrating X axis, we'll take the minimum value that we've seen while stationary and the maximum.
     * Then if we see anything between these values, we'll ignore it.
     */
    float aXZero_min;
    float aXZero_max;

    /**
     * Position of Romi according to what we can figure out from the accelerometer.
     */
    float posX_nf; //# From acc values with no filter
    float posX_gh; //# From acc values filtered with G-H filter
    float posX_k; //# From acc values filtered with Kalman filter

    /**
     * Immediate acceleration and speed for X and Y axis.
     */
    float curAcceleration_X_nf; //# From acc values with no filter
    float curAcceleration_X_gh; //# From acc values filtered with G-H filter
    float curAcceleration_X_k; //# From acc values filtered with Kalman filter

    /**
     * Speed estimates
     */
    float curSpeed_X_nf; //# From acc values with no filter
    float curSpeed_X_gh; //# From acc values filtered with G-H filter
    float curSpeed_X_k; //# From acc values filtered with Kalman filter

    /**
       We will want to accumulate position only when the motor is running. In reality we probably should
       acuumulate it all the time, but given the noisiness of the sensor and it generally not being
       that accurate, this will help us to ignore random spikes.
    */
    boolean motor_is_running;

    /**
       Function that we'll use to update the pose based on accelerometer data.
    */
    void updatePosition(float time_diff);

    unsigned long previous_time;
    float heading;
};

#endif
