#include "config.h"

#include <TMC2130Stepper.h>
#include <Wire.h>


// the stepper driver interface
TMC2130Stepper driver = TMC2130Stepper(PIN_TMC_EN, PIN_TMC_DIR, PIN_TMC_STEP, PIN_SPI_TMC_CS, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK);
// the current motor position, in steps.
int steps = 0;

//----------------------------

// prepare the TMC2130 driver
// See also https://revspace.nl/TMC2130
void MOTORsetup() {
  DEBUGLN(F("MOTORsetup()"));

  SPIsetup();

  driver.begin();
  driver.rms_current(600);  // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.stealthChop(1);  // Enable extremely quiet stepping
  driver.microsteps(0);

  // StallGuard magic
  driver.toff(3);
  driver.tbl(1);
  driver.hysteresis_start(4);
  driver.hysteresis_end(-2);
  driver.diag1_stall(1);
  driver.diag1_active_high(1);
  driver.coolstep_min_speed(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sg_stall_value(STALL_VALUE);

  // enable the driver
  digitalWrite(PIN_TMC_EN,LOW);
}


void SPIsetup() {
	SPI.begin();
	pinMode(PIN_SPI_MISO, INPUT_PULLUP);
}


void MOTORstep() {
  double angleNow = steps / STEPS_PER_DEGREE;
  // get the difference between sensor and assumed motor position
  double diff = sensorAngle - angleNow;
  if(abs(diff)<1.0) return;

  // set the direction
  double dir = diff>0 ? 1 : -1;
  driver.shaft_dir(diff>0?1:0);
  
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
  delayMicroseconds(1000);
  digitalWrite(PIN_TMC_STEP,LOW);
  delayMicroseconds(1000);

  // keep count
  steps+=dir;
  if(steps<0) steps += STEPS_PER_ROTATION;
  if(steps>=STEPS_PER_ROTATION) steps -= STEPS_PER_ROTATION;
}