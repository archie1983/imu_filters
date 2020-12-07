#include "gh_filter.h"

/**
 * g_in is how much we trust the new position update (G parameter of the G-H filter)
 * h_in is how much we trust the new velocity update (H parameter of the G-H filter)
 * alpha_in is how we split measurement weight between sensors. Alpha is how much weight we give to the first parameter in apply_filter(float arg1, float arg2)
 */
Gh_filter_c::Gh_filter_c(float g_in, float h_in, float alpha_in) {
  // arbiteraly defined g, h & alpha correction gains
  // 0 < g < 1 && 0 < h <= 2 && (0 < 4 - 2g - h)
  g = g_in; //# how much we trust the new position update //# 0.5
  h = h_in; //# how much we trust the new velocity update //# 0.1
  alpha = alpha_in; //# how we split measurement weight between sensors. Alpha is how much weight we give to the first parameter in apply_filter(float arg1, float arg2)

  // set initial value of state estimates
  init_params();
}

/**
 * Drops all working parameters to their initial values.
 */
void Gh_filter_c::init_params() {
  // set initial value of state estimates
  current_position = 0.;
  previous_position = 0.;
  velocity = 0.;

  residual = 0.;

  previous_time = micros();
  current_time = 0;
}

/**
 * Receive both inputs- from IMU and from encoders, then condition them and apply
 * the filter.
 */
float Gh_filter_c::apply_filter(float new_measurement_acc, float new_measurement_enc) {

  // complementary filter used to calculate current velocity from sensors
  float new_measurement = alpha * new_measurement_acc + (1 - alpha) * new_measurement_enc;

  return apply_filter(new_measurement);
}

/**
 * Receive just one input- either from the IMU or encoders and then apply the filter
 * just on that input.
 */
float Gh_filter_c::apply_filter(float new_measurement) {
  current_time = micros();
  
  // predict current position based on previous data
  current_position = previous_position + (current_time - previous_time) * velocity; 

  // calculating error between real and prediction position
  residual = new_measurement - current_position;

  /*Update state equations for position and velocity*/
  // update current position based on percision of the sensors
  // g-h filter out any errors arising from sensor measurments
  current_position += g * residual;
  velocity += (h * residual) / (current_time - previous_time);

  // reinitialise values
  previous_position = current_position;
  previous_time = current_time;
  
  return current_position;
}
