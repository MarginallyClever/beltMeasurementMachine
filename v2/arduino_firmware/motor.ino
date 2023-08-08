#include "config.h"
//-----------------------------------------------------------------------------

#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <Wire.h>
#include <HardwareTimer.h>

//-----------------------------------------------------------------------------

#define R_SENSE 0.11f
#define RMS_CURRENT 570 // 0.4a motors / 0.707 peak-to-peak current for TMC2130

#define STEP_TIMER_IRQ_PRIO 2

#define STEPPER_TIMER_RATE 2000000 // 2 Mhz
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000)
#define HAL_TIMER_TYPE_MAX 0xFFFF  // maximum value for 16 bit timer.

//-----------------------------------------------------------------------------

HardwareTimer *timer = new HardwareTimer(TIM8);

// the stepper driver interface
#ifdef USE_SOFTWARE_SPI
TMC2130Stepper driver = TMC2130Stepper(PIN_SPI1_TMC_CS, R_SENSE, PIN_SPI1_MOSI, PIN_SPI1_MISO, PIN_SPI1_CLK); // Software SPI
#else
TMC2130Stepper driver = TMC2130Stepper(PIN_SPI1_TMC_CS, R_SENSE);  // hardware SPI
#endif


// the current motor position, in steps.
int32_t steps = 0;
int32_t targetSteps = 0;
uint8_t stepDir = 0;
uint32_t stepsDiffAbs = 0;
uint32_t stepDelay = 1000000;

/**
 * 0...360
 */
float targetPosition = 0;

uint8_t MOTORstate;

//-----------------------------------------------------------------------------

/**
 * prepare the TMC2130 driver
 * See also https://revspace.nl/TMC2130
 */
void MOTORsetup() {
  DEBUGLN(F("MOTORsetup()"));

  MOTORdriverSetup();
  MOTORinterruptSetup();
}

void MOTORinterruptSetup() {
  uint32_t clockFrequency = timer->getTimerClkFreq();
  uint32_t prescale = clockFrequency/(STEPPER_TIMER_RATE);
  timer->setPrescaleFactor(min((uint32_t)HAL_TIMER_TYPE_MAX,prescale));
  timer->setOverflow(clockFrequency / prescale, TICK_FORMAT);
  timer->setPreloadEnable(false);

  MOTORinterruptEnable();

  // Start the timer.
  timer->resume();
  timer->setInterruptPriority(STEP_TIMER_IRQ_PRIO, 0);
}

void MOTORdriverSetup() {
  pinMode(PIN_TMC_EN,OUTPUT);
  pinMode(PIN_TMC_DIR,OUTPUT);
  pinMode(PIN_TMC_STEP,OUTPUT);

  SPIsetup();

  driver.begin();
  driver.toff(5);           // enable StallGuard
  driver.rms_current(RMS_CURRENT);  // Set motor RMS current
  driver.microsteps(0);    // Set microsteps (0,2,4,8,16)

  // Toggle stealthChop
  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);

  // enable the driver
  digitalWrite(PIN_TMC_EN,LOW);
}

void MOTORinterruptEnable() {
  // Attach an interrupt on overflow (update) of the timer.
  if(!timer->hasInterrupt()) timer->attachInterrupt(MOTORstepInterrupt);
}

void MOTORinterruptDisable() {
  timer->detachInterrupt();
}


void MOTORinterruptOff() {
  __disable_irq();
}

void MOTORinterruptOn() {
  __enable_irq();
}


void MOTORsetCompare(const uint32_t overflow) {
  timer->setOverflow(overflow+1,TICK_FORMAT);
  if(overflow < timer->getCount()) {
    timer->refresh();
  }
}


inline uint32_t MOTORgetCount() {
  return timer->getCount();
}


void SPIsetup() {
  SPI.setMOSI(PIN_SPI1_MOSI);
  SPI.setMISO(PIN_SPI1_MISO);
  SPI.setSCLK(PIN_SPI1_CLK);
	SPI.begin();
	pinMode(PIN_SPI1_MISO, INPUT_PULLUP);
}

/*
// slow way for debugging
void MOTORstep() {
  double angleNow = steps / STEPS_PER_DEGREE;
  // get the difference between sensor and assumed motor position
  double diff = targetPosition - angleNow;
  if(abs(diff)<1.0) return;

  // set the direction
  double dir = diff>0 ? 1 : -1;
  driver.shaft(diff>0);
  
  #ifdef DEBUG_STEPPING
    DEBUG(sensorAngle);
    DEBUG("\t");
    DEBUG(angleNow);
    DEBUG("\t");
    DEBUG(diff);
    DEBUG("\t");
    DEBUGLN(dir);
  #endif

  // move the motor
  digitalWrite(PIN_TMC_STEP,HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(PIN_TMC_STEP,LOW);
  delayMicroseconds(stepDelay);

  // keep count
  steps+=dir;
  if(steps<0) steps += STEPS_PER_ROTATION;
  if(steps>=STEPS_PER_ROTATION) steps -= STEPS_PER_ROTATION;
}
*/

void MOTORstepInterrupt() {
  static uint32_t nextMainISR = 0;

  MOTORinterruptOff();
  MOTORsetCompare(HAL_TIMER_TYPE_MAX);

  uint8_t maxLoops = 10;
  uint32_t minTicks;
  uint32_t nextTick=0;
  
  do {
    MOTORinterruptOn();
    // STEP PHASE
    if(!nextMainISR && stepsDiffAbs>0) {
      // blink light
      MOTORstate = (!MOTORstate ? 255:0);

      MOTORmakeOneStep();
    }
    // BLOCK PHASE
    if(!nextMainISR) nextMainISR = stepDelay;

    // timing
    const uint32_t interval = min((uint32_t)HAL_TIMER_TYPE_MAX,nextMainISR);
    nextMainISR -= interval;
    nextTick += interval;

    MOTORinterruptOff();
    minTicks = MOTORgetCount() + (uint32_t)STEPPER_TIMER_TICKS_PER_US;
    if (!--maxLoops) nextTick = minTicks;
  } while(nextTick < minTicks);

  if(stepsDiffAbs==0) {
    MOTORinterruptDisable();
  } else {
    MOTORsetCompare(nextTick);
  }
  MOTORinterruptOn();
}


/**
 * @param angleUnit 0...1
 */
void MOTORsetTargetPosition(float angleUnit) {
  targetSteps = angleUnit * STEPS_PER_ROTATION;
  int32_t diff = targetSteps - steps;
  stepsDiffAbs = abs(diff);

  MOTORsetDirection(diff);
  MOTORinterruptEnable();
}

void MOTORsetDirection(int direction) {
  stepDir = direction>0 ? 1 : -1;
  digitalWrite(PIN_TMC_DIR,(direction>0)?HIGH:LOW);
}

void MOTORsetTargetVelocity(float degPerS) {
  stepDelay = STEPPER_TIMER_RATE / (degPerS * STEPS_PER_DEGREE);
}

void MOTORmakeOneStep() {
  // move the motor
  digitalWrite(PIN_TMC_STEP,HIGH);
  digitalWrite(PIN_TMC_STEP,LOW);
  // keep count
  stepsDiffAbs--;
  steps += stepDir;
  if(steps<0) steps += STEPS_PER_ROTATION;
  if(steps>=STEPS_PER_ROTATION) steps -= STEPS_PER_ROTATION;
}