#include "gh_filter.h"

Gh_filter_c::Gh_filter_c() {

  previous_time = micros();
  current_time = 0;
  g = 0.5;
  h = 0.1;

  current_position = 0.;
  previous_position = 0.;
  estimated_position = 0.;

  current_velocity = 0.;
  previous_velocity = 0.;
  estimated_velocity = 0.;

  residual = 0.;
  
}

float Gh_filter_c::apply_filter(float new_measurement) {
  
current_time = micros();



  current_position = previous_position + (current_time - previous_time)*current_velocity;

  current_velocity = previous_velocity;

  residual = new_measurement - current_position;

  current_position += g*residual;

  current_velocity += h*residual/(current_time - previous_time);

  previous_position = current_position;

  previous_velocity = current_velocity;
  
  return current_position;
  }
