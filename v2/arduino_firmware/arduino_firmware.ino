//-----------------------------------------------------------------------------
// Daisy Driver firmware
// 2022-12-21 dan@marginallyclever.com
//
// be sure to install "STM32 Boards" from Boards Manager.
// set tools > board to "Generic stm32f4 series"
// set tools > board part number to "Black F407VE"
// set tools > usb support to "CDC (generic 'Serial' supercedes U(S)ART)"
//
// Note:
// if you have a JTAG programmer connected PC->JTAG->PCB and disconnect the 
// PC -> JTAG, don't be surprised if it fails to boot.  You must disconnect
// JTAG -> PCB.  
//-----------------------------------------------------------------------------
#include "config.h"


void setup() {
  #ifdef BUILD_SERIAL
    // serial must be first for enumeration.
    SERIALsetup();
  #endif
  MEMORYsetup();
  #ifdef BUILD_CANBUS
    CANbus.setup();
    //CANOpen::setup();
  #endif
  LEDsetup();
  SENSORsetup();
  MOTORsetup();
  APPsetup();
}


void loop() {
  #ifdef BUILD_CANBUS
    if(CANbus.available()) {
      //CANOpen::receive();
    }
  #endif
  #ifdef BUILD_SERIAL
    SERIALupdate();
  #endif
  SENSORread();

  //CANopen::updateHeartbeat();
  APPupdate();

  LEDsetColor(
    cos(sensorAngleUnit*PI)*255.0f/PI,
    MOTORstate, // sin(sensorAngleUnit*PI)*255.0f/PI,
    CANstate
    );
}