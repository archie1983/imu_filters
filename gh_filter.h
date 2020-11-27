#ifndef _GH_FILTER_
#define _GH_FILTER_
#include <Arduino.h>

class Gh_filter_c {

  public:

    Gh_filter_c();

    float apply_filter(float new_measurement);


  private:

    unsigned long current_time;
    unsigned long previous_time;
    float g;
    float h;

    float current_position;
    float previous_position;
    float estimated_position;

    float current_velocity;
    float previous_velocity;
    float estimated_velocity;

    float residual;



};

#endif
