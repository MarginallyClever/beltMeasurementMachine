#pragma once

//----------------------------
// define this value to report stepper calculations over serial (if BUILD_SERIAL is enabled)
//#define DEBUG_STEPPING

// motor gearbox parameters
#define STEPS_PER_DEGREE (105.0)
#define STEPS_PER_ROTATION (STEPS_PER_DEGREE*360)

// for TMC2130 StallGuard
#define STALL_VALUE 0 // [-64..63]

//----------------------------

extern int steps;

//----------------------------

extern void MOTORsetup();
extern void MOTORstep();
