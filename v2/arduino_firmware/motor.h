#pragma once
//-----------------------------------------------------------------------------

//#define USE_SOFTWARE_SPI

// motor gearbox parameters
#define STEPS_PER_DEGREE (200.0/360.0)
#define STEPS_PER_ROTATION (STEPS_PER_DEGREE*360)

// for TMC2130 StallGuard
#define STALL_VALUE 0 // [-64..63]

//----------------------------

extern int32_t steps;

extern uint32_t stepDelay;

extern uint8_t MOTORstate;

//----------------------------

extern void MOTORsetup();
extern void MOTORstep();
extern void MOTORsetCompare(uint32_t overflow);

extern void MOTORinterruptOff();
extern void MOTORinterruptOn();

/**
 * Set the direction the motor will move each step.
 * @param direction positive for one direction.  0 or negative for the other.
 */
extern void MOTORsetDirection(int direction);

/**
 * Set the target position and activate the stepper interrupt.
 * @param angleUnit 0....1 represents a full rotation, where 1 and 0 are the same thing.
 */
extern void MOTORsetTargetPosition(float angleUnit);

/**
 * Set the motor interrupt delay, which controls the rate at which steps are taken.
 * @param degPerS degrees/second
 */
extern void MOTORsetTargetVelocity(float degPerS);

/**
 * Make one step in the current direction.
 */
extern void MOTORmakeOneStep();