#include "config.h"
#include "CANOpen.h"

class CANOpen {
  static uint8_t state;
  static uint16_t functionCode;
  static uint8_t nodeAddress;

  static void setup() {
    state = STATE_BOOT;
  }

  static void parseCAN(CAN_msg_t *msg) {
    parseFunctionCodeAndNodeAddress(msg);
    switch(functionCode) {
    case CANOPEN_NMT:       parseNMT(msg);        break;
    //case CANOPEN_HEARTBEAT: parseHEARTBEAT(msg);  break;    // servos do not receive heartbeat.
    //case CANOPEN_SYNC:      parseSYNC(msg);       break;
    case CANOPEN_TIMESTAMP: parseTIMESTAMP(msg);  break;
    case CANOPEN_EMCY:      parseEMCY(msg);       break;
    //case CANOPEN_SDO:       parseSDO(msg);        break;
    default: break;
    }
  }

  static void parseFunctionCodeAndNodeAddress(CAN_msg_t *msg) {
    uint16_t id = msg->id;
    functionCode = (id & 0b11110000000);
    nodeAddress  = (id & 0b00001111111);
  }

  static bool thisMessageIsForMe() {
    return nodeAddress == CANBusAddress;
  }


  static void parseNMT(CAN_msg_t *msg) {
    if(!(state & 0x80)) return;
    if(msg->len!=2) return;

    switch(msg->data[0]) {
      case NMT_OPERATIONAL:
        // TODO
        break;
      case NMT_STOPPED:
        // TODO
        break;
      case NMT_PREOPERATIONAL:
        // TODO
        break;
      case NMT_RESETNODE:
        // TODO
        break;
      case NMT_RESETCOMMUNICATION:
        // TODO
        break;
    }
  }


  static uint16_t generateID(uint16_t functionID,uint16_t nodeID) {
    return functionID | nodeID;
  }


  static void generateHEARTBEAT(CAN_msg_t *msg) {
    msg->id = generateID(CANOPEN_HEARTBEAT,CANBusAddress);
    msg->len = 1;
    msg->data[0] = 0;
  }


  static void parseSYNC(CAN_msg_t *msg) {
  }


  static void parseTIMESTAMP(CAN_msg_t *msg) {
  }


  static void parseEMCY(CAN_msg_t *msg) {
  }


  static void parseSDO(CAN_msg_t *msg) {
  }
};