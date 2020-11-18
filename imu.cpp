#include "imu.h"
#include <Wire.h>

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
  //imuHardware.enableDefault();
  reconfigureAcc(afss, aaaf, asr);
  reconfigureGyro(gfss, gsr);
}

/**
 * Changes the configuration of the IMU.
 */
void Imu::reconfigureAcc(Imu::AccFullScaleSelection fss, Imu::AccAntiAliasFilter aaf, Imu::AccSampleRate sr) {
  imuHardware->writeReg(LSM6::CTRL1_XL, (sr << 4) | (fss << 2) | (aaf));

  switch (fss) {
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
void Imu::reconfigureGyro(GyroFullScaleSelection fss, GyroSampleRate sr) {
  imuHardware->writeReg(LSM6::CTRL2_G, (sr << 4) | (fss << 1) | 0x0);

  switch (fss) {
    case GFS_125:
      gyro_sensitivity_conversion_factor = 4.375;
      break;
    case GFS_250:
      acc_sensitivity_conversion_factor = 8.75;
      break;
    case GFS_500:
      acc_sensitivity_conversion_factor = 17.5;
      break;
    case GFS_1000:
      acc_sensitivity_conversion_factor = 35;
      break;
    case GFS_2000:
      acc_sensitivity_conversion_factor = 70;
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
static void Imu::updateAllInitialisedSensors() {
  imuHardware->read();
}

/**
 * A public static function to initialise the timer. Turns out that if we do this inside 
 * the constructor of LineSensor, then our configuration gets overwritten later, because
 * our instance is constructed before Arduino has initialised its stuff. So we have no
 * choice but to call this function in the setup section of the main code.
 */
static void Imu::reInitTimer(long freq) {
  initialiseTimer3(freq);
}

/**
 * Routine to setupt timer3 to run 
 * 
 * @desired_frequency - the desired frequency for the timer to trigger the ISR at.
 */
void Imu::initialiseTimer3(long desired_frequency) {
  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  /**
   * So we have 8 options for the pre-scaler and we have a 16bit counter, so up to 65536.
   * 
   * 000: timer stopped
   * 001: no pre-scaling, so running at 16MHz
   * 010: F / 8 pre-scaler, so running at 2MHz
   * 011: F / 64. so running at 250KHz
   * 100: F / 256, so running at 62.5KHz
   * 101: F/ 1024, so running at 16KHz
   * 110: External clock, falling edge
   * 111: External clock, rising edge.
   * 
   * Let's divide the pre-scaled frequency with the desired frequency and we'll get the counter value
   * that we need to count up to, then go through all the pre-scaler frequencies and see which of the 
   * divisions is the cleanest (the least value after the decimal point).
   */
  unsigned long pre_scaled_frequencies[] = {16000000, 2000000, 250000, 62500, 16000};
  unsigned short cnt = 0;
  
  /**
   * Here we'll keep the closest frequency we've gotten so far while checking them.
   */
  double best_candidate_frequency = 0.0;
  unsigned short best_candidate_pre_scaler = 0;
  unsigned int best_candidate_counter = 0;
  double current_candidate_frequency = 0.0;
  unsigned int current_candidate_counter = 0;

  for (cnt = 0; cnt < sizeof(pre_scaled_frequencies) / sizeof(long); cnt++) {
    /**
     * So if we divide the pre-scaled frequency with the desired frequency, we'll get the counter value,
     * but it of course has to be no more than 65536 and it has to be a whole number.
     */
    current_candidate_counter = round((double)pre_scaled_frequencies[cnt] / (double)desired_frequency);
    current_candidate_frequency = (double)pre_scaled_frequencies[cnt] / current_candidate_counter;

    //# Minimising the deviation from the desired frequency accross different pre-scale values
    if (abs(current_candidate_frequency - desired_frequency) < abs(best_candidate_frequency - desired_frequency) && current_candidate_counter <= 65536) {
      best_candidate_frequency = current_candidate_frequency;
      best_candidate_pre_scaler = cnt;
      best_candidate_counter = current_candidate_counter;
    }
  }

//  Serial.print("Sensor update frequency: desired: ");
//  Serial.print(desired_frequency);
//  Serial.print(" best found: ");
//  Serial.print(best_candidate_frequency);
//  Serial.print(" prescaler: ");
//  Serial.print(pre_scaled_frequencies[best_candidate_pre_scaler]);
//  Serial.print(" counter: ");
//  Serial.println(best_candidate_counter);

  /*
   * Now let's load the registers with the found values.
   */
  if (best_candidate_frequency > 0) {
    switch (best_candidate_pre_scaler) {
      case 0:
        TCCR3B = TCCR3B | (1 << CS30); //# no prescaling
        break;
      case 1:
        TCCR3B = TCCR3B | (1 << CS31); //# /8 prescaling
        break;
      case 2:
        TCCR3B = TCCR3B | (1 << CS31) | (1 << CS30); //# /64 prescaling
        break;
      case 3:
        TCCR3B = TCCR3B | (1 << CS32); //# /256 prescaling
        break;
      case 4:
        TCCR3B = TCCR3B | (1 << CS32) | (1 << CS30); //# /1024 prescaling
        break;
    }

    OCR3A = best_candidate_counter;
    Imu::current_measurement_frequency = best_candidate_frequency;
  }
   
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
}

/**
 * We'll use timer3 to read the sensors.
 * 
 * The name TIMER3_COMPA_vect is a special flag to the 
 * compiler.  It automatically associates with Timer3 in
 * CTC mode.
 */
ISR( TIMER3_COMPA_vect ) {
  /*
   * Let's update all initialised sensors.
   */
  Imu::getImu()->updateAllInitialisedSensors();
  //Serial.print( "\n" );
}

Imu* Imu::imu = new Imu(Imu::AccFullScaleSelection::AFS_2, 
                        Imu::AccAntiAliasFilter::AA_400, 
                        Imu::AccSampleRate::ASR_666k, 
                        Imu::GyroFullScaleSelection::GFS_250, 
                        Imu::GyroSampleRate::GSR_166k);
