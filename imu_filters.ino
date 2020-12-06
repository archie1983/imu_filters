#include "imu.h"
#include "LineSensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
#include "pid.h"
#include "timer3.h"
#include "utils.h"
#include "gh_filter.h"

#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define M0_DIR          16  // Motor Pins.
#define M0_PWM          10
#define M1_DIR          15
#define M1_PWM          9

#define L_SENSE_L       A4  // Line sensor pins
#define L_SENSE_C       A3
#define L_SENSE_R       A2

#define BUZZER_PIN      6   // To make the annoying beeping
#define DEBUG_LED      13

// Behaviour parameters
#define LINE_THRESHOLD        280.00  // note! calibrate for your surface.
#define STRAIGHT_FWD_SPEED       30
#define LINE_FOLLOW_SPEED       4.0

// Speed controller for motors.
// Using same gains for left and right motors.
// You may need to recalibrate these.
//#define SPD_PGAIN      1.8
//#define SPD_IGAIN      0.1
//#define SPD_DGAIN     -1.5
#define SPD_PGAIN      0.8
#define SPD_IGAIN      0.004
#define SPD_DGAIN      -6.5

// PID controller gains for heading feedback
// You may need to recalibrate these.
#define H_PGAIN    0.8
#define H_IGAIN    0.001
#define H_DGAIN   -2.5

// Clas instances.
LineSensor_c  LineSensor( L_SENSE_L, L_SENSE_C, L_SENSE_R );  // Class to handle all 3 line sensors.
Motor_c       L_Motor( M0_PWM, M0_DIR);                       // To set left motor power.
Motor_c       R_Motor( M1_PWM, M1_DIR);                       // To set right motor power.
PID_c         L_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, left.
PID_c         R_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, right.
PID_c         H_PID( H_PGAIN, H_IGAIN, H_DGAIN );             // Position control, angle.
Kinematics_c  RomiPose;
Gh_filter_c gh_filter(0.5, 0.1);

// Global variables.
unsigned long update_ts;   // Used for timing/flow control for main loop()
unsigned long behaviour_ts;// Use to track how long a behaviour has run.

float line_confidence = 1.0;  // Line following updates this to indicate how
// well it is achieving line following.

float random_walk_turn = 0;   // our random walk behaviour needs a global variable


// Flags used for a basic finite state machine
#define STATE_FOLLOW_LINE     0
#define STATE_DRIVE_STRAIGHT  1
#define STATE_TURN_TO_90      2
#define STATE_TURN_TO_120     3
#define STATE_RANDOM_WALK     4
#define STATE_MAX_STATES      5    // used when selecting a random state
#define STATE_IDLE            6    // used when doing nothing
#define STATE_BRAKING         7
int STATE = STATE_IDLE;  // System starts by being idle.
float ghfilterPos = 0;

/**
 * Variables to ensure timed execution of various processes
 */
unsigned long last_imu_read_time = 0;
unsigned long update_time_for_imu = 0;

unsigned long last_filter_update_time = 0;
unsigned long last_pose_print_time = 0;

unsigned long time_now = 0;

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

void setup()
{
  Serial.begin(9600);

  // Wait for serial to connect
  //delay(1500);
  Imu::initialiseIMU(); //# initialisation time of IMU should be enough wait time for serial to connect.

  beep(3);

  // Print a debug, so we can see a reset on monitor.
  if ( SERIAL_ACTIVE ) Serial.println("***RESET***");

  // Make sure motors are off so the robot
  // stays still.
  stop_motors(true);

  // Begin tracking encoder changes.
  setupEncoder0();
  setupEncoder1();

  // Line sensor.
  LineSensor.calibrate();

  // Using Timer3 to calcuate wheel speed
  // in the background at 100hz.
  setupTimer3();

  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0
  RomiPose.setPose( 0, 0, 0 );

  // Set initial value for our system
  // time stamps.
  behaviour_ts = millis();
  update_ts = millis();

  // Set initial state, could be any.
  STATE = STATE_IDLE;

  stop_motors(true);   //not notify IMU

  // Reset PID
  L_PID.reset();
  R_PID.reset();
  H_PID.reset();

  update_time_for_imu = Imu::getImu()->getAccRefreshTime();
}

/**
 * We'll be switching which data we feed to the sensor fusion algorithm. On one cycle we'll
 * be feeding in IMU data and on next it should be Kinematics pose data.
 */
bool kinematics_or_imu_data = false;

void loop()
{
  /**
     If we've been going straight for over a second, then stop
  */
  if (STATE == STATE_DRIVE_STRAIGHT && RomiPose.getPoseXmm() >= 500) {//abs(ghfilterPos) >= 0.5) {
    stop_motors(false);   //not notify IMU
    changeState(STATE_BRAKING);
    Serial.println("STOP");
  } else if (STATE == STATE_BRAKING && millis() - behaviour_ts > 150) {
    changeState(STATE_IDLE);
    stop_motors(true);   // notify IMU
  }

  /**
   * Below this line we'll be branching based on time constraints
   */
  time_now = micros();  

  if (time_now - last_imu_read_time > update_time_for_imu) {
    Imu::getImu()->getAx(); //getAx is called to request acceleration from IMU
    last_imu_read_time = time_now;
  }

  if (time_now - last_filter_update_time > TIME_TO_UPDATE_FILTERED_POSE) {
    RomiPose.update(e0_count, e1_count);
    ghfilterPos = gh_filter.apply_filter(Imu::getImu()->getCurrentPosXmm(), RomiPose.getPoseXmm());
    last_filter_update_time = time_now;

//  if (kinematics_or_imu_data) {
//    ghfilterPos = gh_filter.apply_filter(RomiPose.getPoseXmm());
//  } else {
//    ghfilterPos = gh_filter.apply_filter(Imu::getImu()->getCurrentPosXmm());
//  }
//
//  kinematics_or_imu_data = !kinematics_or_imu_data;
    
  }

  if ((STATE != STATE_IDLE && time_now - last_pose_print_time > TIME_TO_PRINT_POSE) ||
      time_now - last_pose_print_time > TIME_TO_PRINT_POSE * 1) {
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
    Serial.print(ghfilterPos);
    Serial.print(", ");
    Serial.print(Imu::getImu()->getCurrentPosXmm());  //prints distance in m
    Serial.print(", ");
    Serial.print(RomiPose.getPoseXmm());  //prints distance in m
    Serial.print(", ");
    Serial.print(Imu::getImu()->getCurrentSpeedX());
    Serial.print(", ");  
    Serial.println(Imu::getImu()->getCurrentAccelerationX());
    
    last_pose_print_time = time_now;

    act_on_commands();
  }
  //delay(5);
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
      driveStraight();
    } else if (in_cmd.indexOf("back") > -1) { //# Go straight
      Serial.println("START");
      driving_direction = false;
      changeState(STATE_DRIVE_STRAIGHT);
      driveStraight();
    } else if (in_cmd.indexOf("zeroa") > -1) { //# Zero all and re-calibrate
      RomiPose.setPose( 0, 0, 0 );
      Imu::getImu()->setZeroPos(true);
      Serial.println("ZEROED ALL");
    } else if (in_cmd.indexOf("zerop") > -1) { //# Zero all but DO NOT re-calibrate
      RomiPose.setPose( 0, 0, 0 );
      Imu::getImu()->setZeroPos(false);
      Serial.println("ZEROED POS");
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

  // Assume we have found the line...
  line_confidence = 1.0;

  // Clear any old accumulated error in the
  // PID controllers
  L_PID.reset();
  R_PID.reset();
  H_PID.reset();

  // Switch the global flag to set the new
  // romi state
  STATE = which_state;
}

/* Works very similar to turnToDemandTheta() function.
   Attempts to minimise difference between theta_demand
   and the Romi Theta, but includes a forward bias when
   operating the motors.  Therefore, it will turn to face
   the theta_demand value and also drive fowards, in a
   straight line once romiPose.theta matches demand_theta.
*/
//void driveStraight( float theta_demand ) {
//
//  // First, make sure the demand is reasonably
//  // sensible.  Wrap value within 0:TWO_PI
//  while ( theta_demand < 0 )      theta_demand += TWO_PI;
//  while ( theta_demand > TWO_PI ) theta_demand -= TWO_PI;
//
//  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
//  // Some crazy atan2 magic.
//  // Treats the difference in angle as cartesian x,y components.
//  // Cos and Sin are effectively wrapping the values between -1, +1, with a 90 degree phase.
//  // So we can pass in values larger/smaller than 0:TWO_PI fine.
//  // atan2 returns -PI/+PI, giving us an indication of direction to turn.
//  // Between -PI/+PI also means we avoid an extreme demand sent to the heading PID, e.g.
//  // if we were to send PI*4...
//
//  // We want to keep the difference in the Romi theta
//  // between time steps to 0.
//  float diff = atan2( sin( ( theta_demand - RomiPose.theta) ), cos( (theta_demand - RomiPose.theta) ) );
//
//  // Demand 0, change in theta is measurement.
//  float bearing = H_PID.update( 0, diff );
//
//  // Foward speed.
//  float fwd_bias = (driving_direction ? 1 : -1) * STRAIGHT_FWD_SPEED;
//
//  // PID speed control.
//  float l_pwr = L_PID.update( (fwd_bias - bearing), l_speed_t3 );
//  float r_pwr = R_PID.update( (fwd_bias + bearing), r_speed_t3 );
//
//  Imu::getImu()->setMotorRunning(true);
//
//  // Write power to motors.
//  L_Motor.setPower(l_pwr);
//  R_Motor.setPower(r_pwr);
//} // end of behaviour

void driveStraight() {
  float fwd_bias = (driving_direction ? 1 : -1) * STRAIGHT_FWD_SPEED;

  Imu::getImu()->setMotorRunning(true);

  // Write power to motors.
  L_Motor.setPower(fwd_bias);
  R_Motor.setPower(fwd_bias);
}

/*
   This function attempts to minimise the difference between
   a demanded angle and the Romi current angle (theta).
   Whilst non-zero, it causes the robot to turn on the spot.
   Note, very large demands may cause unwanted behaviour...
*/
void turnToDemandTheta( float demand_angle) {

  // First, make sure the demand is reasonably
  // sensible.  Wrap value within 0:TWO_PI
  while ( demand_angle < 0 ) demand_angle += TWO_PI;
  while ( demand_angle > TWO_PI ) demand_angle -= TWO_PI;


  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  // Some crazy atan2 magic.
  // Treats the difference in angle as cartesian x,y components.
  // Cos and Sin are effectively wrapping the values between -1, +1, with a 90 degree phase.
  // So we can pass in values larger/smaller than 0:TWO_PI fine.
  // atan2 returns -PI/+PI, giving us an indication of direction to turn.
  // Between -PI/+PI also means we avoid an extreme demand sent to the heading PID, e.g.
  // if we were to send PI*4...
  float diff = atan2( sin( ( demand_angle - RomiPose.theta) ), cos( (demand_angle - RomiPose.theta) ) );
  //Serial.print( abs(diff), 4 );

  // If we have got the Romi theta to roughly match
  // the demand (by getting the difference to 0(ish)
  // We transition out of this behaviour.
  if ( abs(diff) <= 0.03 ) {

    // Turn is complete.
    // Set motor power to off
    L_Motor.setPower(0);
    R_Motor.setPower(0);

    // We have 4 states #defined at the top.
    // We typecast random(), which will round down.
    // Therefore, currently randomly sets a new state
    // between 0 and 3
    int new_state = (int)random( 0, STATE_MAX_STATES );

    // We use a function to change state
    // to reset system variables in the same
    // way every time.
    changeState( new_state );

  } else {    // else, turning behaviour

    // Measurement is the different in angle, demand is 0
    // bearing steers us to minimise difference toward 0
    float bearing = H_PID.update( 0, diff );

    // Append to motor speed control
    float l_pwr = L_PID.update( ( 0 - bearing ), l_speed_t3 );
    float r_pwr = R_PID.update( ( 0 + bearing ), r_speed_t3 );

    // Set motor power.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  } // end of abs(diff)<0.03 if()

}// end of behaviour
