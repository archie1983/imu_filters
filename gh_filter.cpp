#include "gh_filter.h"

Gh_filter_c::Gh_filter_c() {

  previous_time = micros();
  current_time = 0;
  // arbiteraly defined g, h & alpha correction gains
  // 0 < g < 1 && 0 < h <= 2 && (0 < 4 - 2g - h)
  g = 0.5;
  h = 0.1;
  alpha = 0.5;  // spliting measurment weight equaly between sensors

  // set initial value of state estimates
  current_position = 0.;
  previous_position = 0.;
  velocity = 0.;

  residual = 0.;
}

float Gh_filter_c::apply_filter(float new_measurement_acc, float new_measurement_enc) {

  // complementary filter used to calculate current velocity from sensors
  float new_measurement = alpha * new_measurement_acc + (1 - alpha) * new_measurement_enc;

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
