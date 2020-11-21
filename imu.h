#ifndef _IMU_
#define _IMU_

#include <Arduino.h>
#include <LSM6.h>
#include "pin_names_and_constants.h"

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
     * Changes the configuration of the Accelerometer.
     */
    void reconfigureAcc(AccFullScaleSelection afss, AccAntiAliasFilter aaaf, AccSampleRate asr);

    /**
     * Changes the configuration of the Gyroscope.
     */
    void reconfigureGyro(GyroFullScaleSelection gfss, GyroSampleRate gsr);

    /**
     * Returns how much time we can allow between measurement updates for accelerometer.
     */
    unsigned int getAccRefreshRate(AccSampleRate asr);

    /**
     * Returns how much time we can allow between measurement updates for gyro.
     */
    unsigned int getGyroRefreshRate(GyroSampleRate gsr);

    /**
       Returns Accelerometer's X axis value converted to mg.
    */
    float getAx();

    /**
       Returns Accelerometer's Y axis value converted to mg.
    */
    float getAy();

    /**
       Returns Accelerometer's Z axis value converted to mg.
    */
    float getAz();

    /**
       Returns Gyroscope's X axis value converted to mdps.
    */
    float getGx();

    /**
       Returns Gyroscope's Y axis value converted to mdps.
    */
    float getGy();

    /**
       Returns Gyroscope's Z axis value converted to mdps.
    */
    float getGz();

    /**
       Returns Accelerometer's X axis value raw.
    */
    float getAxRaw();

    /**
       Returns Accelerometer's Y axis value raw.
    */
    float getAyRaw();

    /**
       Returns Accelerometer's Z axis value raw.
    */
    float getAzRaw();

    /**
       Returns Gyroscope's X axis value raw.
    */
    float getGxRaw();

    /**
       Returns Gyroscope's Y axis value raw.
    */
    float getGyRaw();

    /**
       Returns Gyroscope's Z axis value raw.
    */
    float getGzRaw();

    /**
       Returns Accelerometer's X axis value EMA filtered.
    */
    float getAxEmaFiltered();

    /**
       Returns Accelerometer's Y axis value EMA filtered.
    */
    float getAyEmaFiltered();

    /**
       Returns Accelerometer's Z axis value EMA filtered.
    */
    float getAzEmaFiltered();

    /**
       Returns Gyroscope's X axis value EMA filtered.
    */
    float getGxEmaFiltered();

    /**
       Returns Gyroscope's Y axis value EMA filtered.
    */
    float getGyEmaFiltered();

    /**
       Returns Gyroscope's Z axis value EMA filtered.
    */
    float getGzEmaFiltered();

    /**
     * Variables required for applying EMA to the IMU outputs.
     */
    float prev_Ax_ema_val;
    float prev_Ay_ema_val;
    float prev_Az_ema_val;
    float prev_Gx_ema_val;
    float prev_Gy_ema_val;
    float prev_Gz_ema_val;

    /**
       Static getter for this class
    */
    static Imu* getImu();

    /**
       Call this from the setup loop to initialise the IMU sensors.
    */
    static void initialiseIMU();

    /**
       Take in a fresh reading for each initialised sensor.
    */
    void readAllAxis();

    /**
       Take in a fresh reading for each initialised sensor.
    */
    void calibrateGx();

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
     * How many microseconds should we aim to get between reads for accelerometer.
     */
    unsigned int acc_refresh_time_us;

    /**
     * How many microseconds should we aim to get between reads for gyroscope.
     */
    unsigned int gyro_refresh_time_us;

    /**
     * Time in microseconds since our last read.
     */
    long time_since_last_read;
    /**
     * Reads the sensor via I2C bus if the time since last read has been long enough.
     */
    void readSensorIfNeeded();

    float totalGx;
    float gXZero;
};

#endif
