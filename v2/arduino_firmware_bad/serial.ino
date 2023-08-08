#include "serial.h"

#ifdef BUILD_SERIAL

#define SERIAL_BUFFER_LEN (60)

char serialBuffer[SERIAL_BUFFER_LEN];
int8_t serialPos;

void SERIALsetup() {
  // must be first, prevents connected USB devices from enumerating this device before it is ready.
  pinMode(PIN_BOOT1,OUTPUT);
  digitalWrite(PIN_BOOT1,LOW);

  // now get ready
  Serial.begin(115200);

  while(!Serial.availableForWrite());
  // now tell whoever we connect to that we're ready to be enumerated.
  digitalWrite(PIN_BOOT1,HIGH);

  delay(2000);
  serialPos++;
  DEBUGLN("Hello, world.");
}

void SERIALread() {
  if(!Serial.available()) return;

  int8_t c = Serial.read();
  if(c=='\n') {
    serialBuffer[serialPos]=0;
    SERIALprocess();
  } else {
    if(serialPos<SERIAL_BUFFER_LEN) {
      serialBuffer[serialPos++]=c;
    }
  }
}

void SERIALprocess() {
  // read the digit and output that many meters.
  if(isdigit(serialBuffer[0])) {
    int mm = atoi(serialBuffer);
    stepThisFar(mm);
  }
}

// motor is 200 steps.  Pulley is 20 teeth of GT2, or 40mm pitch.
#define PULLEY_PITCH         (40.0) // mm
#define MOTOR_STEPS_PER_TURN (200.0)
#define MM_PER_STEP          (PULLEY_PITCH / MOTOR_STEPS_PER_TURN)
#define MM_PER_DEGREE        (PULLEY_PITCH / 360.0)

void stepThisFar(int mm) {
  int16_t stepTimeUs = 1000;
  double previousPosition = getCurrentSensorPosition();
  double sum = 0;
  while(sum-mm > MM_PER_STEP) {
    MOTORmakeOneStep(stepTimeUs);
    SENSORread();
    double degreeChange = sensorAngle - previousPosition;
    previousPosition = sensorAngle;
    double mmChange = MM_PER_DEGREE * degreeChange;
    sum += mmChange;
    Serial.print(sum);
    Serial.print('\t');
    Serial.print(mmChange);
    Serial.print('\t');
    Serial.print(degreeChange);
    Serial.print('\t');
    Serial.print(sensorAngle);
    Serial.print('\t');
    Serial.print(previousPosition);
    Serial.print('\n');
  }
  Serial.println("done");
}

double getCurrentSensorPosition() {
  SENSORread();
  return sensorAngle;
}
#endif