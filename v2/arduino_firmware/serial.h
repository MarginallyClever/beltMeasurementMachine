#pragma once
//-----------------------------------------------------------------------------

#define SERIAL_BAUD     115200

#define SERIAL_MAX_LEN  60

//-----------------------------------------------------------------------------

extern char serialBufferIn[SERIAL_MAX_LEN];
extern int  serialReceived;

//-----------------------------------------------------------------------------

#ifdef BUILD_SERIAL
extern void SERIALsetup();
extern void SERIALupdate();
#endif