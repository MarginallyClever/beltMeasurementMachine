#include "serial.h"

#ifdef BUILD_SERIAL
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
  DEBUGLN("Hello, world.");
}
#endif