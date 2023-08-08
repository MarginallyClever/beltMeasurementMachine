#pragma once
//-----------------------------------------------------------------------------

// define this to use USB serial.
#define BUILD_SERIAL
// define this to use CANBus.
#define BUILD_CANBUS

// define this value to report sensor readings over serial (if BUILD_SERIAL is enabled)
//#define DEBUG_SENSOR

// define this value to report stepper calculations over serial (if BUILD_SERIAL is enabled)
//#define DEBUG_STEPPING

// uncomment to turn on serial debugging
//#define DEBUG_CAN
// uncomment to report CAN send fails over serial.
//#define CAN_REPORT_FAIL


#ifdef BUILD_CANBUS
  //#define CAN_SPEED       CAN_1000KBPS
  #define CAN_SPEED       CAN_500KBPS
  //#define CAN_SPEED       CAN_100KBPS
#endif

//-----------------------------------------------------------------------------

#ifdef BUILD_SERIAL
  #define DEBUG(x)        Serial.print(x)
  #define DEBUG2(x0,x1)   Serial.print(x0,x1)
  #define DEBUGLN(x)      Serial.println(x)
#else
  #define NOP __asm__("nop\n\t")
  #define DEBUG(x)        NOP
  #define DEBUG2(x0,x1)   NOP
  #define DEBUGLN(x)      NOP
#endif

#include "pins.h"
#include "serial.h"
#include "CANBus.h"
#include "memory.h"
#include "sensor.h"
#include "motor.h"
#include "led.h"
#include "CANOpen.h"
#include "application.h"
