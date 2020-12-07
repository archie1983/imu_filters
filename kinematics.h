#ifndef _Kinematics
#define _Kinematics_h


class Kinematics_c {
  public:
  
    //Public variables and methods go here
    float x;
    float y; 
    float theta;
    float last_theta;
    long last_e0;
    long last_e1;

    /**
     * Estimated acceleration and velocity. This is just forward acceleration and velocity, not for
     * X and Y axis individually.
     */
    float cur_acceleration;
    float cur_velocity;
    float common_distance; //# a combination of X and Y pose values to produce just a common travel distance. Should be equivalent to sqrt(x * x + y * y)

    /**
     * Variables to keep track of time- when it was updated compared to last time so that we can figure out velocity and acceleration.
     */
    long last_update_time;
    long current_update_time;

    /* 
     *  Function prototypes 
     */
    Kinematics_c::Kinematics_c();
    void update( long e0, long e1 );
    void setPose( float _x, float _y, float _theta );
    float getPoseX();
    float getPoseXmm();
    float getTravelledDistance_mm();
    float getCurAcceleration();
    float getCurVelocity();

  private:

    float wheel_sep         = (143 / 2);
    float wheel_radius      = 32.5; //With rubber tyres: 35, without: 32.5 - all in mm
    float rad_per_enc_step  = (TWO_PI / 1440.0);
    float mm_per_enc_step   = 0.15;

}; // End of class definition.


/*
 * Constructor, zeros variables.
 */
Kinematics_c::Kinematics_c() {
  setPose(0, 0, 0);
  last_theta = 0;
  last_e0 = 0;
  last_e1 = 0;
} // end of constructor.


/*
 * Allows the kinematics to be set to an intial value
 * or reset.
 */
void Kinematics_c::setPose( float _x, float _y, float _theta ) {
  x     = _x;
  y     = _y;
  theta = _theta;
  common_distance = sqrt(_x * _x + _y * _y);
  cur_acceleration = 0.;
  cur_velocity = 0.;
  last_update_time = micros();
}

/*
 * Update for the kinematics.
 */
void Kinematics_c::update( long e0, long e1 ) {

  // This class stores the last received
  // encoder counts to calculate the difference
  // in encoder counts for itself.
  long delta_e0 = e0 - last_e0;
  long delta_e1 = e1 - last_e1;
  last_e0 = e0;
  last_e1 = e1;

  // Rotation for each wheel, using
  // the number of radians per encoder count
  // We're only keeping track of distance travelled
  // (a cartesian coordinate position), not speed, 
  // so we don't need to factor elapsed time here.
  // However, knowing how long a time elapsed between
  // update() might help to estimate error in the 
  // approximation (?)
  float av0;    // av, angular velocity
  av0 = rad_per_enc_step;
  av0 = av0 * (float)delta_e0;

  float av1;
  av1 = rad_per_enc_step;
  av1 = av1 * (float)delta_e1;


  // Kinematics without ICC projection
  // Some error is going to accumulate.
  // But with a quick enough update, its a pretty good
  // straight-line approximation for curves.
  float new_x = (( av0 * wheel_radius ) + ( av1 * wheel_radius )) / 2;
  
  float new_theta = (( av0 * wheel_radius) - (av1 * wheel_radius ) ) / (2 * wheel_sep );
  //float new_theta = ( av0 - av1 ) / (2*wheel_sep);
  
  
  // record current theta as 'old' so that we can
  // keep track of angular change else where in 
  // the program.
  last_theta = theta;
  

  // Update global theta.
  theta = theta + new_theta;

  // Lets wrap theta to keep it between 0 and TWO_PI
  // Not strictly necessary, but predictable at least.
  while( theta < 0 ) theta += TWO_PI;
  while( theta > TWO_PI ) theta -= TWO_PI;

  // Integrate this movement step by rotating
  // the x contribution by theta, therefore "sharing" it
  // between x and y in the global reference frame.
  x = x + (new_x * cos( theta ) );
  y = y + (new_x * sin( theta ) );
  
  common_distance += new_x;
  /**
   * Now we'll calculate current velocity and acceleration based on
   * what encoder count increases we've seen since last time.
   */
  current_update_time = micros();
  float prev_velocity = cur_velocity;
  cur_velocity = new_x / ((float(current_update_time - last_update_time) / US_IN_1_S) * 1000.0); //# converting to m/s
  cur_acceleration = (cur_velocity - prev_velocity) / (float(current_update_time - last_update_time) / US_IN_1_S);
  last_update_time = current_update_time;
  
} // End of update()

float Kinematics_c::getPoseX(){
  return x/1000.0;
}

float Kinematics_c::getPoseXmm(){
  return x;
}

float Kinematics_c::getTravelledDistance_mm(){
  return common_distance;
}

float Kinematics_c::getCurAcceleration() {
  return cur_acceleration;
}

float Kinematics_c::getCurVelocity() {
  return cur_velocity;
}

#endif
