#include "imu.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
#include "timer3.h"
#include "gh_filter.h"
#include <TrivialKalmanFilter.h>

#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define M0_DIR          16  // Motor Pins.
#define M0_PWM          10
#define M1_DIR          15
#define M1_PWM          9

#define BUZZER_PIN      6   // To make the annoying beeping
#define DEBUG_LED      13

#define STRAIGHT_FWD_SPEED       30

Motor_c       L_Motor( M0_PWM, M0_DIR);                       // To set left motor power.
Motor_c       R_Motor( M1_PWM, M1_DIR);                       // To set right motor power.
Kinematics_c  RomiPose;

/**
   Position filter G = 0.5; H = 0.1; Alpha = 0.2 in favour of second argument, which gets 0.8 weight when doing apply_filter(float arg1, float arg2).
*/
Gh_filter_c position_filter(0.5, 0.1, 0.2);

/**
   Fused Acceleration filter. G = 0.5; H = 0.1; Alpha = 0.2 in favour of second argument, which gets 0.8 weight when doing apply_filter(float arg1, float arg2).
*/
Gh_filter_c acc_filter(0.5, 0.1, 0.2);

/**
   IMU Acceleration filter. G = 0.5; H = 0.1; Alpha = 0.2 but will not be used here.
*/
Gh_filter_c imu_acc_filter(0.5, 0.1, 0.2);

/**
   Kalman filter to filter IMU values.
*/
#define DT_COVARIANCE_RK 4.7e-3 // Estimation of the noise covariances (observation)
#define DT_COVARIANCE_QK 1e-5   // Estimation of the noise covariances (process)

TrivialKalmanFilter<float> imu_acc_filter_k(DT_COVARIANCE_RK, DT_COVARIANCE_QK);

// Global variables.
unsigned long update_ts;   // Used for timing/flow control for main loop()
unsigned long behaviour_ts;// Use to track how long a behaviour has run.

// Flags used for a basic finite state machine
#define STATE_DRIVE_STRAIGHT  1
#define STATE_IDLE            2    // used when doing nothing
#define STATE_BRAKING         3
int STATE = STATE_IDLE;  // System starts by being idle.

/**
   The value resulting from fusing the IMU acceleration and Kinematics calculated acceleration
   with a G-H filter.
*/
float fused_acc = 0.;
float fused_speed = 0.; //# speed inferred from fused_acc.

float cur_ghfilterPos = 0; //# G-H filter fused estimates of position that come from IMU and from encoder counts - for the current iteration of the movement.
float total_ghfilterPos = 0; //# G-H filter fused estimates of position that come from IMU and from encoder counts - for the total movement.

float cur_fused_pos = 0; //# Position estimate from values that are acquired using a G-H filter to fuse IMU accelerations and acceleration estimates from encoder counts - for the current iteration of the movement.
float total_fused_pos = 0; //# Position estimate from values that are acquired using a G-H filter to fuse IMU accelerations and acceleration estimates from encoder counts - for the total movement.

float cur_imu_pos_nf = 0; //# Position estimate from IMU values (no filtering) - for the current iteration of the movement.
float total_imu_pos_nf = 0; //# Position estimate from IMU values (no filtering) - for the total movement.

float cur_imu_pos_gh = 0; //# Position estimate from IMU values (G-H filtered) - for the current iteration of the movement.
float total_imu_pos_gh = 0; //# Position estimate from IMU values (G-H filtered) - for the total movement.

float cur_imu_pos_kf = 0; //# Position estimate from IMU values (Kalman filtered) - for the current iteration of the movement.
float total_imu_pos_kf = 0; //# Position estimate from IMU values (Kalman filtered) - for the total movement.

/**
   Variables to ensure timed execution of various processes
*/
unsigned long last_imu_read_time = 0;
unsigned long update_time_for_imu = 0;

unsigned long last_filter_update_time = 0;
unsigned long last_pose_print_time = 0;

unsigned long time_now = 0;

float filtered_imu_acc = 0.; //# IMU acc filtered with G-H filter.
float filtered_imu_acc_k = 0.; //# IMU acc filtered with Kalman filter

float last_stop_distance = 0.; //# Where did we stop last time when we completed the last movement.
byte movements_left_to_do = NUMBER_OF_MOVEMENTS; //# How many movements have we left to do to cover the overall distance?

/*
   Note, this blocks the flow/timing of your code.  Use sparingly.
   Audio indicator to notify boot up sequence
*/
void beep(int count) {
  for (int i = 0; i < count; i++) {
    analogWrite(6, 80);
    delay(50);
    analogWrite(6, 0);
    delay(50);
  }
}

void stop_motors(bool notifyIMU) {
  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );
  if (notifyIMU) Imu::getImu()->setMotorRunning(false);
}

/**
   Drops all speeds, accelerations and positions to 0.

   recalibrate_acc: a flag of whether we want to re-calibrate IMU or not
*/
void zero_pose(bool recalibrate_acc) {
  stop_fused_motion();
  
  total_ghfilterPos = 0;
  total_fused_pos = 0;
  total_imu_pos_nf = 0;
  total_imu_pos_gh = 0;
  total_imu_pos_kf = 0;

  // X = 0, Y = 0, Theta = 0
  RomiPose.setPose( 0, 0, 0 );
  Imu::getImu()->setZeroPos(recalibrate_acc);
  position_filter.init_params();
}

/**
   Zeroes the fused speed and acceleration when the motion has stopped.
*/
void stop_fused_motion() {
  Imu::getImu()->setZeroAccAndSpeeds();

  fused_acc = 0.;
  fused_speed = 0.; //# speed inferred from fused_acc.

  /**
   * Positions that were achieved during the last movement iteration.
   */
  cur_ghfilterPos = 0;
  cur_fused_pos = 0;
  cur_imu_pos_nf = 0;
  cur_imu_pos_gh = 0;
  cur_imu_pos_kf = 0;
  
  filtered_imu_acc = 0.; //# IMU acc filtered with G-H filter.
  filtered_imu_acc_k = 0.; //# IMU acc filtered with Kalman filter

  imu_acc_filter_k.reset();
  imu_acc_filter.init_params();
  acc_filter.init_params();
  position_filter.init_params();
}

void setup()
{
  Serial.begin(9600);
  
  // Print a debug, so we can see a reset on monitor.
  if ( SERIAL_ACTIVE ) Serial.println("***RESET***");

  // Wait for serial to connect
  //delay(1500);
  Imu::initialiseIMU(); //# initialisation time of IMU should be enough wait time for serial to connect.

  beep(3);

  // Make sure motors are off so the robot
  // stays still.
  stop_motors(true);

  // Begin tracking encoder changes.
  setupEncoder0();
  setupEncoder1();

  // Using Timer3 to calcuate wheel speed
  // in the background at 100hz.
  setupTimer3();

  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0
  zero_pose(false);

  // Set initial value for our system
  // time stamps.
  behaviour_ts = millis();
  update_ts = millis();

  // Set initial state, could be any.
  STATE = STATE_IDLE;

  stop_motors(true);   //not notify IMU

  update_time_for_imu = Imu::getImu()->getAccRefreshTime();
}

/**
   We'll be switching which data we feed to the sensor fusion algorithm. On one cycle we'll
   be feeding in IMU data and on next it should be Kinematics pose data.
*/
bool kinematics_or_imu_data = false;

void loop()
{
  if (STATE == STATE_DRIVE_STRAIGHT && ( //# If we're going straight AND
    RomiPose.getPoseXmm() >= OVERALL_DISTANCE_TO_ACHIEVE || //# Have covered all the required distance altogether OR
    RomiPose.getPoseXmm() - last_stop_distance >= OVERALL_DISTANCE_TO_ACHIEVE / NUMBER_OF_MOVEMENTS //# Have covered the required distance in the current movement
    )) { //# Then we want to start braking
    stop_motors(false);   //not notify IMU
    changeState(STATE_BRAKING);

    if (RomiPose.getPoseXmm() >= OVERALL_DISTANCE_TO_ACHIEVE) {
      Serial.println("FULLSTOP");
    } else {
      Serial.println("TEMPSTOP");
    }
  } else if (STATE == STATE_BRAKING && millis() - behaviour_ts > 50) { //# If we've been braking for more than 50ms, then we'll assum that we've stopped now.
    changeState(STATE_IDLE);

    movements_left_to_do--;
    last_stop_distance = RomiPose.getPoseXmm();

    total_ghfilterPos += cur_ghfilterPos; //# G-H filter fused estimates of position that come from IMU and from encoder counts - for the total movement.
    total_fused_pos += cur_fused_pos * 1000; //# Position estimate from values that are acquired using a G-H filter to fuse IMU accelerations and acceleration estimates from encoder counts - for the total movement.
    total_imu_pos_nf += cur_imu_pos_nf; //# Position estimate from IMU values (no filtering) - for the total movement.
    total_imu_pos_gh += cur_imu_pos_gh; //# Position estimate from IMU values (G-H filtered) - for the total movement.
    total_imu_pos_kf += cur_imu_pos_kf; //# Position estimate from IMU values (Kalman filtered) - for the total movement.

    stop_fused_motion();

    if (movements_left_to_do > 0) {
      /**
       * Starting the next iteration.
       */
      changeState(STATE_DRIVE_STRAIGHT);
      driveStraight(true);
    } else {
      /**
       * Stopping for good
       */
      stop_motors(true);   // notify IMU
      last_stop_distance = 0;
      movements_left_to_do = NUMBER_OF_MOVEMENTS;
    }
  }

  /**
     Below this line we'll be branching based on time constraints
  */
  time_now = micros();

  /**
     Here we update IMU pose, acceleration and velocity values when needed
  */
  if (time_now - last_imu_read_time > update_time_for_imu + US_IN_1_MS) {
    Imu::getImu()->getAx(); //getAx is called to request acceleration from IMU and do all acc, speed and distance calculations

    float cur_ax = Imu::getImu()->getCurrentAccelerationX(Imu::EstimateType::KALMAN_FILTERED);
    filtered_imu_acc = imu_acc_filter.apply_filter(cur_ax);
    filtered_imu_acc_k = imu_acc_filter_k.update(cur_ax);
    //    /**
    //     * Do the Kinematics update too and also the fused distance calculations.
    //     */
    //    RomiPose.update(e0_count, e1_count);
    //    ghfilterPos = position_filter.apply_filter(Imu::getImu()->getCurrentPosXmm(), RomiPose.getPoseXmm());
    //
    //    /**
    //     * Now that we've updated both Kinematics and IMU, let's try to fuse their accelerations.
    //     */
    //    infer_position_from_fused_acc(time_now - last_imu_read_time);

    last_imu_read_time = time_now;
  }

  /**
     Here we update Kinematics pose, acceleration and velocity values when needed
  */
  if (time_now - last_filter_update_time > TIME_TO_ESTIMATE_ACC_FROM_ENCODERS) {
    RomiPose.update(e0_count, e1_count);
    cur_ghfilterPos = position_filter.apply_filter(Imu::getImu()->getCurrentPosXmm(Imu::EstimateType::NO_FILTERED), RomiPose.getPoseXmm()); //# G-H filter fused estimates of position that come from IMU and from encoder counts - for the current iteration of the movement.

    /**
       Now that we've updated both Kinematics and IMU, let's try to fuse their accelerations.
    */
    infer_position_from_fused_acc(time_now - last_filter_update_time);
    last_filter_update_time = time_now;
  }

  if ((STATE != STATE_IDLE && time_now - last_pose_print_time > TIME_TO_PRINT_POSE) ||
      time_now - last_pose_print_time > TIME_TO_PRINT_POSE * 10) {
    //    Serial.print(ghfilterPos);
    //    Serial.print(",");
    //    Serial.print(Imu::getImu()->getCurrentPosXmm());  //prints distance in m
    //    Serial.print(", ");
    //    Serial.println(RomiPose.getPoseXmm());  //prints distance in m

    //  Serial.print(", ");
    //Serial.print(Imu::getImu()->getAxEmaFiltered(false));
    //    Serial.print(Imu::getImu()->getAxRaw());
    //    Serial.print(", ");
    //    Serial.print(Imu::getImu()->getFilteredAx());
    //    Serial.print(", ");
    //    Serial.print(Imu::getImu()->getAxRawCompensated());
    //    Serial.print(", ");
    
    //cur_ghfilterPos = ...; //# G-H filter fused estimates of position that come from IMU and from encoder counts - for the current iteration of the movement.
    //cur_fused_pos = ...; //# Position estimate from values that are acquired using a G-H filter to fuse IMU accelerations and acceleration estimates from encoder counts - for the current iteration of the movement.
    cur_imu_pos_nf = Imu::getImu()->getCurrentPosXmm(Imu::EstimateType::NO_FILTERED); //# Position estimate from IMU values (no filtering) - for the current iteration of the movement.
    cur_imu_pos_gh = Imu::getImu()->getCurrentPosXmm(Imu::EstimateType::GH_FILTERED); //# Position estimate from IMU values (G-H filtered) - for the current iteration of the movement.
    cur_imu_pos_kf = Imu::getImu()->getCurrentPosXmm(Imu::EstimateType::KALMAN_FILTERED); //# Position estimate from IMU values (Kalman filtered) - for the current iteration of the movement.

    Serial.print(total_ghfilterPos + cur_ghfilterPos);  // G-H filter of possition reported by acc and encoder
    Serial.print(", ");
    Serial.print(total_fused_pos + cur_fused_pos * 1000);   // G-H filter of acceleration reported by acc and encoder and then cal position
    Serial.print(", ");
    Serial.print(total_imu_pos_nf + cur_imu_pos_nf);  // RAW IMU reported positions with no filter
    Serial.print(", ");
    Serial.print(total_imu_pos_gh + cur_imu_pos_gh);  // G-H filter only on acc and then calc position
    Serial.print(", ");
    Serial.print(total_imu_pos_kf + cur_imu_pos_kf);  // Kalman fiilter only on acc and then calc position
    Serial.print(", ");
    Serial.print(RomiPose.getTravelledDistance_mm());  // Encoder count reported distance
    Serial.print(", ");
    Serial.print(RomiPose.getCurAcceleration(), 6);
    Serial.print(", ");
    Serial.println(Imu::getImu()->getCurrentAccelerationX(Imu::EstimateType::NO_FILTERED), 6);
    
    //    Serial.print(", ");
    //    Serial.print(Imu::getImu()->getCurrentSpeedX(), 6);
    //    Serial.print(", ");
    //    Serial.print(RomiPose.getCurVelocity(), 6);
    //    Serial.print(", ");
    //    Serial.print(fused_acc);
    //    Serial.print(", ");
    //    Serial.print(filtered_imu_acc, 6);
    //    Serial.print(", ");
    //    Serial.print(filtered_imu_acc_k, 6);
    //    Serial.print(", ");
    //    Serial.println(RomiPose.getCurAcceleration(), 6);
    //filtered_imu_acc

    last_pose_print_time = time_now;

    act_on_commands();
  }
  //delay(5);
}

/**
   Infers speed and position from the fused accelerations of Kinematics and IMU.

   time_diff: the time in us that was spent with the acceleration values that we get from kinematics and IMU.
*/
void infer_position_from_fused_acc(long time_diff) {
  fused_acc = acc_filter.apply_filter(filtered_imu_acc, RomiPose.getCurAcceleration());
  fused_speed = fused_speed + (time_diff / US_IN_1_S) * fused_acc; //# converting acceleration to the speed change in m/s and adding to the speed
  cur_fused_pos += (time_diff / US_IN_1_S) * fused_speed; //# converting position to m
}

boolean driving_direction = true; //# TRUE for going forward and FALSE for going back.
/**
   Reads a command from the Serial connection and acts on it
*/
void act_on_commands() {
  //This line checks whether there is anything to read
  if ( Serial.available() ) {
    String in_cmd = Serial.readString();

    if (in_cmd.indexOf("run") > -1) { //# Go straight
      Serial.println("START");
      driving_direction = true;
      changeState(STATE_DRIVE_STRAIGHT);
      driveStraight(true);
    } else if (in_cmd.indexOf("back") > -1) { //# Go straight
      Serial.println("START");
      zero_pose(false);
      driving_direction = false;
      changeState(STATE_DRIVE_STRAIGHT);
      driveStraight(true);
    } else if (in_cmd.indexOf("zeroa") > -1) { //# Zero all and re-calibrate
      zero_pose(true);
      Serial.println("ZEROED ALL");
      beep(2);
      Serial.print("MIN: ");
      Serial.print(Imu::getImu()->getAxZero_min());
      Serial.print(", MAX: ");
      Serial.println(Imu::getImu()->getAxZero_max());
    } else if (in_cmd.indexOf("zerop") > -1) { //# Zero all but DO NOT re-calibrate
      zero_pose(false);
      Serial.println("ZEROED POS");
    } else if (in_cmd.indexOf("zerod") > -1) { //# Zero all and re-calibrate while driving. This is to understand noise level while the speed should be constant while driving.
      driveStraight(false);
      delay(1000); //# let it get up to speed
      zero_pose(true); //# start calibration
      stop_motors(false);
      beep(2);
      Serial.println("ZEROED ALL");
      beep(2);
      Serial.print("MIN: ");
      Serial.print(Imu::getImu()->getAxZero_min());
      Serial.print(", MAX: ");
      Serial.println(Imu::getImu()->getAxZero_max());
    } else if (in_cmd.indexOf("printcal") > -1) { //# Print calibration params
      Serial.print("MIN: ");
      Serial.print(Imu::getImu()->getAxZero_min());
      Serial.print(", MAX: ");
      Serial.println(Imu::getImu()->getAxZero_max());
    }
  }
}

/*
    We use a function to change behaviour because
    everytime we change behaviour we need to reset
    the PID controllers and also some variables.
    Therefore reducing repeated code reduces the
    chance of bugs.
*/
void changeState( int which_state ) {

  // Note, this is blocking!
  // But when we transition behaviour it
  // is useful to hear it.
  beep(1);

  // We update the update_ts general time
  update_ts = millis();

  // We use behaviour_ts timestamp to track
  // how long a behaviour has been operating.
  // Therefore, on change state we reset the
  // timestamp
  behaviour_ts = millis();

  // Switch the global flag to set the new
  // romi state
  STATE = which_state;
}

void driveStraight(bool notify_imu) {
  float fwd_bias = (driving_direction ? 1 : -1) * STRAIGHT_FWD_SPEED;

  if (notify_imu) {
    Imu::getImu()->setMotorRunning(true);
  }

  // Write power to motors.
  L_Motor.setPower(fwd_bias);
  R_Motor.setPower(fwd_bias);
}
