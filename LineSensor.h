#ifndef _LineSensor_h
#define _LineSensor_h
class LineSensor_c {
  
  public:

    // Constructor, takes the 3 pins being used for
    // the line sensor (eg. A4, A3, A2)
    LineSensor_c(int p0, int p1, int p2 ) {
      setPins( p0, p1, p2 );
      setGains( -0.01, 0.01 );
    };

    // Save pins to the class variables so we can 
    // re-use them later for the analogRead operations.
    void setPins( int p0, int p1, int p2 ) {
      pin0 = p0;
      pin1 = p1;
      pin2 = p2;
      pinMode( p0, INPUT);
      pinMode( p1, INPUT);
      pinMode( p2, INPUT);
    }

    // Simple routine to work out the mean reading
    // from each of the IR sensors, and then we save
    // this so we can subtract it from later readings
    // to remove the offset. 
    // Note: Assumes calibration happens on the non-line
    // surface (not black).
    void calibrate() {
      bias0 = 0;
      bias1 = 0;
      bias2 = 0;
      int n_samples = 20;
      for( int i = 0; i < n_samples; i++ ) {
         bias0 += analogRead( pin0 );
         bias1 += analogRead( pin1 );
         bias2 += analogRead( pin2 );
      }
      
      if( bias0 > 0 ) bias0 /= n_samples;
      if( bias1 > 0 ) bias1 /= n_samples;
      if( bias2 > 0 ) bias2 /= n_samples;
      
    }


    // This version of the weighted line sensing
    // uses some gains to scale the measurement value.
    void setGains( float k0, float k1 ) {
        gain_left = k0;
        gain_right = k1;
    }


    // If any of the sensors are above a threshold value,
    // returns true (sensor is on the line). Otherwise
    // false.  
    // Note: user must specify a reliable threshold value
    // from experimenting with their Romi and environment.
    boolean onLine( float threshold ) {
        
        // If any of these are true, return true.
        if( getCalibrated( LEFT )   > threshold ) return true;
        
        if( getCalibrated( CENTRE ) > threshold ) return true;
        
        if( getCalibrated( RIGHT )  > threshold ) return true;

       // Else, return false.
       return false;
        
    
    }

    // Useful debug function to print out the raw
    // sensor values (e.g., with the offset present)
    void printRawSensorReadings() {
      Serial.print( analogRead( pin0 ) );
      Serial.print(",");
      Serial.print( analogRead( pin1 ) );
      Serial.print(",");
      Serial.print( analogRead( pin2 ) );
      Serial.print("\n");
    }

    // Does a sensor read for left/centre/right, 
    // and removes the mean (offset).  Note, 
    // LEFT/CENTRE/RIGHT are defined as private
    // variables in this class.  
    float getCalibrated( int which ) {
      
      float sensor_read;
      
      if( which == LEFT ) {
            sensor_read = (float)analogRead( pin0 );
            return (sensor_read - bias0 );
            
      } else if( which == CENTRE) {
            sensor_read = (float)analogRead( pin1 );
            return (sensor_read - bias1 );
            
      } else if( which == RIGHT ) {
            sensor_read = (float)analogRead( pin2 );
            return (sensor_read - bias2 );
            
      } else {  // non-existent "which" value
            return -1; 
      }
     }


    // Weighted line sensing. 
    // Assumes that the gain values for left and right
    // are going to be opposite signs (e.g., -1 / +1)
    float getHeading( ) {

       float value_left   = getCalibrated( LEFT );
       float value_centre = getCalibrated( CENTRE );
       float value_right  = getCalibrated( RIGHT );
       
       float left, right;
       left = (value_left + value_centre );
       right = (value_centre + value_right);

       float feedback = (left * gain_left ) + (right * gain_right );
       
       return feedback;
    }


  private:
    int pin0;               // to store hardware pins in use.
    int pin1;
    int pin2;
    float bias0;            // offset measured from sensor
    float bias1;
    float bias2;
    float gain_left;       // for weighted line sensing
    float gain_right;
    const int LEFT    = 0; // used to make code more readable
    const int CENTRE  = 1;
    const int RIGHT   = 2;
};

#endif
