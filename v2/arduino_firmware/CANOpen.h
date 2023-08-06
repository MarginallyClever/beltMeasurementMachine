#pragma once


#define CANOPEN_NMT       (0x000)
#define CANOPEN_HEARTBEAT (0x700)
//#define CANOPEN_SYNC      (0x000)
#define CANOPEN_TIMESTAMP (0x100)
#define CANOPEN_EMCY      (0x080)
//#define CANOPEN_SDO       (0x000)

#define STATE_BOOT             (0x00)
#define STATE_BOOTLOADER       (0x02)
#define STATE_COLLISION        (0x03)
#define STATE_STOPPED          (0x04)
#define STATE_OPERATIONAL      (0x05)
#define STATE_PREOPERATIONAL   (0x7F)

#define NMT_OPERATIONAL        (0x01)
#define NMT_STOPPED            (0x02)
#define NMT_PREOPERATIONAL     (0x80)
#define NMT_RESETNODE          (0x81)
#define NMT_RESETCOMMUNICATION (0x82)