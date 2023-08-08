#include "config.h"
#include "CANOpen.h"
//-----------------------------------------------------------------------------
// https://www.ni.com/en-ca/shop/seamlessly-connect-to-third-party-devices-and-supervisory-system/the-basics-of-canopen.html
//-----------------------------------------------------------------------------

CANOpen CANopen;

//-----------------------------------------------------------------------------

CANOpen::CANOpen() {
  operationalState = OperationalState::RESET_NODE;
}

void CANOpen::begin() {
  operationalState = OperationalState::RESET_COMMUNICATION;

  // Initialize the CAN bus with the desired baud rate
  CANbus.setup();

  operationalState = OperationalState::INITIALIZATION;

  lastHeartbeatTime = millis();
  CANopen.sendHeartbeatMessage(CANbus.CANBusAddress, operationalState);

  operationalState = OperationalState::PRE_OPERATIONAL;
}

bool CANOpen::send(CANOpen_msg_t *msg) {
    _convertCANOpenToCAN(msg, &_canMsg);
    return CANbus.send(&_canMsg);
}

void CANOpen::receive() {
    CANOpen_msg_t _canOpenMsg;

    CANbus.receive(&_canMsg);
    _convertCANToCANOpen(&_canMsg, &_canOpenMsg);
    if (_canOpenMsg.COB_ID == 0x0000) {
      handleNMTMessage(&_canOpenMsg);
    } else if(_canOpenMsg.COB_ID == HEARTBEAT_CONSUMER_BASE_COB_ID) {
      handleHeartbeatMessage(&_canOpenMsg);
    }
}

void CANOpen::_convertCANOpenToCAN(CANOpen_msg_t *openMsg, CAN_msg_t *canMsg) {
    canMsg->id = openMsg->COB_ID;
    canMsg->type = openMsg->RTR;
    canMsg->len = openMsg->length;
    memcpy(canMsg->data, openMsg->data, openMsg->length);
}

void CANOpen::_convertCANToCANOpen(CAN_msg_t *canMsg, CANOpen_msg_t *openMsg) {
    openMsg->COB_ID = canMsg->id;
    openMsg->RTR = canMsg->type;
    openMsg->length = canMsg->len;
    memcpy(openMsg->data, canMsg->data, canMsg->len);
}

void CANOpen::sendNMTMessage(NMT_Command command, uint8_t nodeID) {
    CANOpen_msg_t nmtMsg;
    nmtMsg.COB_ID = 0x0000; // NMT COB-ID
    nmtMsg.RTR = 0;
    nmtMsg.length = 2;
    nmtMsg.data[0] = command;
    nmtMsg.data[1] = nodeID;

    send(&nmtMsg);
}

void CANOpen::handleNMTMessage(CANOpen_msg_t *msg) {
    NMT_Command command = static_cast<NMT_Command>(msg->data[0]);
    uint8_t nodeID = msg->data[1];
    if(nodeID != CANbus.CANBusAddress) return;

    // Take appropriate action based on the NMT command received
    switch (command) {
      case NMT_START_REMOTE_NODE:
        // Start the node with the given node ID
        operationalState = OperationalState::OPERATIONAL;
        break;
      case NMT_STOP_REMOTE_NODE:
        // Stop the node with the given node ID
        operationalState = OperationalState::STOPPED;
        break;
      case NMT_ENTER_PREOPERATIONAL:
        // Set the node to the pre-operational state
        operationalState = OperationalState::PRE_OPERATIONAL;
        break;
      case NMT_RESET_NODE:
        // Reset the node
        //pinMode(PIN_NRST,OUTPUT);
        //digitalWrite(PIN_NRST,HIGH);
        //digitalWrite(PIN_NRST,LOW);
        break;
      case NMT_RESET_COMMUNICATION:
        // Reset the communication for the node
        this->begin();
        break;
      default:
        // Handle unknown commands if necessary
        break;
    }
}

void CANOpen::sendHeartbeatMessage(uint8_t nodeID, uint8_t state) {
    CANOpen_msg_t hbMsg;
    hbMsg.COB_ID = HEARTBEAT_PRODUCER_BASE_COB_ID + nodeID; // Heartbeat producer COB-ID
    hbMsg.RTR = 0;
    hbMsg.length = 1;
    hbMsg.data[0] = state;

    send(&hbMsg);
}

void CANOpen::handleHeartbeatMessage(CANOpen_msg_t *msg) {
    uint8_t nodeID = msg->COB_ID & 0x7F;
    uint8_t state = msg->data[0];

    // Take appropriate action based on the received Heartbeat message
    // For example, you could update the node's state in a data structure or trigger an event
}  

void CANOpen::updateHeartbeat() {
  uint32_t currentTime = millis();
  // Check if it's time to send a Heartbeat message
  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
      this->sendHeartbeatMessage(CANbus.CANBusAddress, operationalState);
      lastHeartbeatTime = currentTime;
  }
}