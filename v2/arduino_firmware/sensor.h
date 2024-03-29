#pragma once
//-----------------------------------------------------------------------------
// sensor readings

#define SENSOR_BITS 12  // 10 is the default in arduino.  12 will get 4096 positions.
#if SENSOR_BITS == 12
  #define SENSOR_FULL_VALUE (4096.0)
#else
  #define SENSOR_FULL_VALUE (2048.0)
#endif
#define SENSOR_MIDDLE_VALUE (SENSOR_FULL_VALUE*0.5)
#define SENSOR_RANGE_HALF (SENSOR_FULL_VALUE*0.5)

//----------------------------

/**
 * Last sensor reading. Absolute value [0...1]
 */
extern float sensorAngleUnit;

//----------------------------

extern void SENSORsetup();
extern void SENSORread();