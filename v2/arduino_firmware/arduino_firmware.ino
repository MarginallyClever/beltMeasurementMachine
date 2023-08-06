//-----------------------------------------------------------------------------
// Daisy Driver firmware
// 2022-12-21 dan@marginallyclever.com
//-----------------------------------------------------------------------------
#include "config.h"


void setup() {
  #ifdef BUILD_SERIAL
    // serial must be first for enumeration.
    SERIALsetup();
  #endif
  MEMORYsetup();
  #ifdef BUILD_CANBUS
    CANsetup();
  #endif
  LEDsetup();
  SENSORsetup();
  MOTORsetup();
}


void loop() {
  #ifdef BUILD_CANBUS
    CANstep();
  #endif
  SENSORread();
  MOTORstep();
}