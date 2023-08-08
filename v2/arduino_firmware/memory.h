#pragma once
//-----------------------------------------------------------------------------

// the expected version of the firmware.  must be updated any time a variable is added or removed from the persistent store.
#define FIRMWARE_VERSION        "V01"

// the default address for this device on the CANBus.
#define DEFAULT_CANBUS_ADDRESS  (255)

//-----------------------------------------------------------------------------

extern void MEMORYsetup();
extern void MEMORYsave();
extern bool MEMORYload();
extern void MEMORYreset();
