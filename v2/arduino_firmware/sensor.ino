#include "sensor.h"

// the angle last read by the sensor
double sensorAngle = 0;


void SENSORsetup() {
  DEBUGLN(F("SENSORsetup()"));

  analogReadResolution(SENSOR_BITS);

  // use i2c to adjust the sensor receiver half-gain.  See IPS2200 programming guide section 3.5.3
  
  // read the current sensor value
  SENSORread();
  steps = sensorAngle * STEPS_PER_DEGREE;
}


void SENSORread() {
  // if sensor max VDD is 5 and min is 0 then middle is 2.5 and Renesas says the max/min of the sensor value will be +/-1.25 (1/4 VDD)
  // get and make -1...1
  double c  = ((double)analogRead(PIN_IPS_COS ) - SENSOR_MIDDLE_VALUE) / SENSOR_RANGE_HALF;
  double s  = ((double)analogRead(PIN_IPS_SIN ) - SENSOR_MIDDLE_VALUE) / SENSOR_RANGE_HALF;
  double cn = ((double)analogRead(PIN_IPS_COSN) - SENSOR_MIDDLE_VALUE) / SENSOR_RANGE_HALF;
  double sn = ((double)analogRead(PIN_IPS_SINN) - SENSOR_MIDDLE_VALUE) / SENSOR_RANGE_HALF;
  // one = half minus negative half
  double sx = c-cn; 
  double sy = s-sn;
  // limit check
  sx = min(1.0,max(sx,-1.0));
  sy = min(1.0,max(sy,-1.0));

  // get sensor angle as a value from 0...1
  double sensorAngleUnit = (atan2(sy,sx)+PI) / (2.0*PI);
  
  #ifdef DEBUG_SENSOR
    // debug
    DEBUG(analogRead(PIN_IPS_COS ) );    DEBUG('\t');
    DEBUG(analogRead(PIN_IPS_SIN ) );    DEBUG('\t');
    DEBUG(c );    DEBUG('\t');
    DEBUG(s );    DEBUG('\t');
    DEBUG(cn);    DEBUG('\t');
    DEBUG(sn);    DEBUG('\t');
    DEBUG(sx);    DEBUG('\t');
    DEBUG(sy);    DEBUG('\t');
    DEBUGLN(sensorAngle);
  #endif

  // convert 0...1 -> 0...360
  // and store in global for later
  sensorAngle = 360.0 * sensorAngleUnit;
  
  // color wheel
  // convert 0...1 -> 0...255
  //wheel((byte)(int)(255.0 * sensorAngleUnit));
}