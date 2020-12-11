#ifndef _PIN_NAMES_
#define _PIN_NAMES_

/**
 * This class will serve as a central point for all pin definitions
 * and constants.
 */

/**
 * Amount of microseconds in a second.
 */
#define US_IN_1_S 1000000.0
#define US_IN_1_MS 1000.0
#define CALIBRATION_ITERATIONS 1000 //# 700

/**
 * Alpha factors for EMA filtering acceleromter and gyroscope values.
 */
#define ACC_ALPHA_4_EMA 0.2
#define GYRO_ALPHA_4_EMA 0.5

/**
 * Gravity constant
 */
#define GRAVITY_CONSTANT 9.80665

/**
   LED
*/
#define YELLOW_LED 13

/**
 * Time constraints for IMU and others
 */
#define TIME_TO_UPDATE_FILTERED_POSE 50000 //# We'll update our filter every this many microseconds
#define TIME_TO_ESTIMATE_ACC_FROM_ENCODERS 50000 //# We'll estimate acceleration from encoders every this many microseconds
#define TIME_TO_PRINT_POSE 100000 //# We'll print our poses every this many microseconds
#endif
