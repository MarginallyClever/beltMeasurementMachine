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

extern void MOTORsetDirection(int direction);

// make one step in the current direction.  waits for delay_us/2 microseconds before and after each half of the step.
extern void MOTORmakeOneStep(int16_t delay_us);
