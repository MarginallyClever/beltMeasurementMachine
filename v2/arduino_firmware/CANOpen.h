#pragma once
//-----------------------------------------------------------------------------

#define HEARTBEAT_INTERVAL_MS 1000

#define HEARTBEAT_PRODUCER_BASE_COB_ID  0x700
#define HEARTBEAT_CONSUMER_BASE_COB_ID  0x700

#define CAN_SET                   0x2012
#define CAN_CUSTOM_PARAMETER_READ 0x2013

#define CAN_SET_CURRENT       1
#define CAN_SET_BRAKE_CURRENT 2
#define CAN_SET_VELOCITY      3
#define CAN_SET_POSITION      4


#define CAN_CUSTOM_PARAMETER_READ_POSITION 5

//-----------------------------------------------------------------------------

struct CANOpen_msg_t {
    uint16_t COB_ID;
    uint8_t RTR;
    uint8_t length;
    uint8_t data[8];
};


enum OperationalState {
  RESET_NODE,
  RESET_COMMUNICATION,
  INITIALIZATION,
  PRE_OPERATIONAL,
  OPERATIONAL,
  STOPPED,
};


enum NMT_Command {
    NMT_START_REMOTE_NODE = 0x01,
    NMT_STOP_REMOTE_NODE = 0x02,
    NMT_ENTER_PREOPERATIONAL = 0x80,
    NMT_RESET_NODE = 0x81,
    NMT_RESET_COMMUNICATION = 0x82
};

//-----------------------------------------------------------------------------

class CANOpen {
public:
    OperationalState operationalState;
    uint32_t lastHeartbeatTime;

    CANOpen();
    void begin();
    bool send(CANOpen_msg_t *msg);
    void receive();
    void sendNMTMessage(NMT_Command command, uint8_t nodeID);

    void updateHeartbeat();

private:
    CAN_msg_t _canMsg;

    void _convertCANOpenToCAN(CANOpen_msg_t *openMsg, CAN_msg_t *canMsg);
    void _convertCANToCANOpen(CAN_msg_t *canMsg, CANOpen_msg_t *openMsg);

    void handleNMTMessage(CANOpen_msg_t *msg);

    void sendHeartbeatMessage(uint8_t nodeID, uint8_t state);
    void handleHeartbeatMessage(CANOpen_msg_t *msg);
};

//-----------------------------------------------------------------------------

extern CANOpen CANopen;