#pragma once


//----------------------------
// Mutually exclusive features - With STM32F103FC8T6 you can have USB serial or CANBus but not both.
// If you try to use both then neither will work.

// define this to use USB serial.
#define BUILD_SERIAL
// define this to use CANBus.
//#define BUILD_CANBUS

#if defined(BUILD_SERIAL) && defined(BUILD_CANBUS)
#error define BUILD_SERIAL or BUILD_CANBUS.  Not both at once!
#endif
#if !defined(BUILD_SERIAL) && !defined(BUILD_CANBUS)
#error define BUILD_SERIAL or BUILD_CANBUS.  At least one!
#endif

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

#ifdef BUILD_CANBUS
  //#define CAN_SPEED       CAN_1000KBPS
  #define CAN_SPEED       CAN_500KBPS
  //#define CAN_SPEED       CAN_100KBPS
#endif

#include "pins.h"
#include "serial.h"
#include "CANBus.h"
#include "memory.h"
#include "sensor.h"
#include "motor.h"
#include "led.h"