#include <EEPROM.h>
#include "CANBus.h"

void MEMORYsetup() {
  if(!MEMORYload()) {
    MEMORYreset();
    MEMORYsave();
  }

  DEBUG(F("version="));
  DEBUGLN(FIRMWARE_VERSION);
  
}


bool MEMORYload() {
  DEBUG(F("EEPROM load..."));
  char storedVersion[4];
  int ePos=0;

  EEPROM.get(ePos,storedVersion);
  ePos+=sizeof(storedVersion);

  if(strncmp(FIRMWARE_VERSION,storedVersion,3)!=0) {
    if (storedVersion[3] != '\0') {
      storedVersion[0] = '?';
      storedVersion[1] = '\0';
    }
    DEBUG("EEPROM version mismatch.  Expected ");
    DEBUG(FIRMWARE_VERSION);
    DEBUG(", found ");
    DEBUGLN(storedVersion);
    return false;
  }

   EEPROM.get(ePos,CANBusAddress);
   ePos+=sizeof(CANBusAddress);

  return true;
}


void MEMORYsave() {
  DEBUG(F("EEPROM saving..."));
  int ePos=0;

  char storedVersion[4];
  strcpy(storedVersion,FIRMWARE_VERSION);
  EEPROM.put(ePos,storedVersion);
   ePos+=sizeof(storedVersion);

  EEPROM.put(ePos,CANBusAddress);
   ePos+=sizeof(CANBusAddress);
}


// load default values into memory
void MEMORYreset() {
  DEBUG(F("EEPROM reset..."));
  CANBusAddress = DEFAULT_CANBUS_ADDRESS;
}