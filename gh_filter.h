#ifndef _GH_FILTER_
#define _GH_FILTER_
#include <Arduino.h>

class Gh_filter_c {

  public:

    Gh_filter_c();

    /**
     * Receive both inputs- from IMU and from encoders, then condition them and apply
     * the filter.
     */
    float apply_filter(float new_measurement_acc, float new_measurement_enc);

    /**
     * Receive just one input- either from the IMU or encoders and then apply the filter
     * just on that input.
     */
    float apply_filter(float new_measurement);

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
