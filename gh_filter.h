#ifndef _GH_FILTER_
#define _GH_FILTER_
#include <Arduino.h>

class Gh_filter_c {

  public:

    Gh_filter_c();

    float apply_filter(float new_measurement_acc, float new_measurement_enc);

  private:

    float alpha; //parameter needed in complimentary filter
    
    unsigned long current_time;
    unsigned long previous_time;
    float g;
    float h;

    float current_position;
    float previous_position;

    float velocity;

    float residual;
};

#endif
