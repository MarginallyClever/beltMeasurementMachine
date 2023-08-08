#include "config.h"
//-----------------------------------------------------------------------------

char serialBufferIn[SERIAL_MAX_LEN];
int  serialReceived;

//-----------------------------------------------------------------------------

#ifdef BUILD_SERIAL

void SERIALsetup() {
  // must be first, prevents connected USB devices from enumerating this device before it is ready.
  //pinMode(PIN_BOOT1,OUTPUT);
  //digitalWrite(PIN_BOOT1,LOW);

  // now get ready
  Serial.begin(SERIAL_BAUD);

  while(!Serial.availableForWrite());
  // now tell whoever we connect to that we're ready to be enumerated.
  //digitalWrite(PIN_BOOT1,HIGH);

  delay(2000);

  serialReceived=0;

  DEBUGLN("Hello, world.");
}

void SERIALupdate() {
  if(Serial.available()) {
    char c = Serial.read();
    if(serialReceived < SERIAL_MAX_LEN-1) {
      serialBufferIn[serialReceived++]=c;
    }
    // new line, instruction done.
    if(c=='\n') {
      serialBufferIn[serialReceived]=0;
      SERIALparse();
      serialReceived=0;
    }
  }
}

int seen(char c) {
  for(int i=0;i<serialReceived;++i) {
    if(serialBufferIn[i]==c) {
      return i+1;
    }
  }
  return -1;
}

void SERIALparse() {
  int codePos = seen('G');
  if(codePos>=0) {
    int gcode = atoi(serialBufferIn+codePos);
    switch(gcode) {
      case 0:
        APPrapidMove();
        break;
      case 1:
        APPrapidMove();
        break;
      case 7:  // not normally used by Marlin
        APPmeasureBelt();
        break;
      default:
        break;
    }
    return;
  }
  
  codePos = seen('M');
  if(codePos>=0) {
    int mcode = atoi(serialBufferIn+codePos);
    switch(mcode) {
      case 114:
        APPreportAllMotorPositions();
        break;
      default:
        break;
    }
    return;
  }
}
#endif