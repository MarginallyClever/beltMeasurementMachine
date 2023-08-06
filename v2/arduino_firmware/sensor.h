#pragma once

//----------------------------
// sensor readings

// define this value to report sensor readings over serial (if BUILD_SERIAL is enabled)
#define DEBUG_SENSOR

#define SENSOR_BITS 12  // 10 is the default in arduino.  12 will get 4096 positions.
#if SENSOR_BITS == 12
  #define SENSOR_FULL_VALUE (4096.0)
#else
  #define SENSOR_FULL_VALUE (2048.0)
#endif
#define SENSOR_MIDDLE_VALUE (SENSOR_FULL_VALUE*0.5)
#define SENSOR_RANGE_HALF (SENSOR_FULL_VALUE*0.5)

//----------------------------

extern double sensorAngle;

//----------------------------

extern void SENSORsetup();
extern void SENSORread();