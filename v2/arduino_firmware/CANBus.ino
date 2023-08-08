//-----------------------------------------------------------------------------
// CAN bus for Daisy Driver
//-----------------------------------------------------------------------------
#include "CANBus.h"
#include <assert.h>

//-----------------------------------------------------------------------------

#define CAN_SEND_DELAY 1000000

#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU

//-----------------------------------------------------------------------------

CAN_bit_timing_config_t can_configs[6] = {{2, 12, 56}, {2, 12, 28}, {2, 13, 21}, {2, 11, 12}, {2, 11, 6}, {1, 5, 6}};


CANBus CANbus;


uint8_t counter = 0;
uint8_t frameLength = 0;
uint32_t previousMillis = 0;  // stores last time output was updated
const long interval = 1000;  // transmission interval (milliseconds)

uint8_t canR=0;
uint8_t canG=0;
uint8_t canB=0;
uint8_t countS=0;
uint8_t countG=0;

//-----------------------------------------------------------------------------

/**
 * Print registers.
*/ 
void CANBus::printRegister(const char * buf, uint32_t reg) {
  #ifdef DEBUG_CAN
  DEBUG(buf);
  DEBUG2(reg, HEX);
  DEBUGLN();
  #endif
}

/**
 * Initializes the CAN GPIO registers.
 *
 * @params: addr    - Specified GPIO register address.
 * @params: index   - Specified GPIO index.
 * @params: speed   - Specified OSPEEDR register value.(Optional)
 *
 */
void CANBus::setGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t speed) {
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7) {
      _index4 = (index - 8) * 4;
      ofs = 1;
    }

    uint32_t mask;
    this->printRegister("GPIO_AFR(b)=", addr->AFR[1]);
    mask = 0xF << _index4;
    addr->AFR[ofs]  &= ~mask;         // Reset alternate function
    setting = 0x9;                    // AF9
    mask = setting << _index4;
    addr->AFR[ofs]  |= mask;          // Set alternate function
    this->printRegister("GPIO_AFR(a)=", addr->AFR[1]);

    this->printRegister("GPIO_MODER(b)=", addr->MODER);
    mask = 0x3 << _index2;
    addr->MODER   &= ~mask;           // Reset mode
    setting = 0x2;                    // Alternate function mode
    mask = setting << _index2;
    addr->MODER   |= mask;            // Set mode
    this->printRegister("GPIO_MODER(a)=", addr->MODER);

    this->printRegister("GPIO_OSPEEDR(b)=", addr->OSPEEDR);
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask;           // Reset speed
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask;            // Set speed
    this->printRegister("GPIO_OSPEEDR(a)=", addr->OSPEEDR);

    this->printRegister("GPIO_OTYPER(b)=", addr->OTYPER);
    mask = 0x1 << index;
    addr->OTYPER  &= ~mask;           // Reset Output push-pull
    this->printRegister("GPIO_OTYPER(a)=", addr->OTYPER);

    this->printRegister("GPIO_PUPDR(b)=", addr->PUPDR);
    mask = 0x3 << _index2;
    addr->PUPDR   &= ~mask;           // Reset port pull-up/pull-down
    this->printRegister("GPIO_PUPDR(a)=", addr->PUPDR);
}


void CANBus::readAddress() {
  pinMode(PIN_CAN_ADDR0,INPUT_PULLUP);
  pinMode(PIN_CAN_ADDR1,INPUT_PULLUP);
  pinMode(PIN_CAN_ADDR2,INPUT_PULLUP);
  pinMode(PIN_CAN_ADDR3,INPUT_PULLUP);
  pinMode(PIN_CAN_ADDR4,INPUT_PULLUP);
  pinMode(PIN_CAN_ADDR5,INPUT_PULLUP);

  this->CANBusAddress = 
      ((uint8_t)digitalRead(PIN_CAN_ADDR0) << 0)
    | ((uint8_t)digitalRead(PIN_CAN_ADDR1) << 1)
    | ((uint8_t)digitalRead(PIN_CAN_ADDR2) << 2)
    | ((uint8_t)digitalRead(PIN_CAN_ADDR3) << 3)
    | ((uint8_t)digitalRead(PIN_CAN_ADDR4) << 4)
    | ((uint8_t)digitalRead(PIN_CAN_ADDR5) << 5);
}



/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
void CANBus::setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
  if (index > 27) return;

  CAN1->FA1R &= ~(0x1UL<<index);               // Deactivate filter

  if (scale == 0) {
    CAN1->FS1R &= ~(0x1UL<<index);             // Set filter to Dual 16-bit scale configuration
  } else {
    CAN1->FS1R |= (0x1UL<<index);              // Set filter to single 32 bit configuration
  }
    if (mode == 0) {
    CAN1->FM1R &= ~(0x1UL<<index);             // Set filter to Mask mode
  } else {
    CAN1->FM1R |= (0x1UL<<index);              // Set filter to List mode
  }

  if (fifo == 0) {
    CAN1->FFA1R &= ~(0x1UL<<index);            // Set filter assigned to FIFO 0
  } else {
    CAN1->FFA1R |= (0x1UL<<index);             // Set filter assigned to FIFO 1
  }

  CAN1->sFilterRegister[index].FR1 = bank1;    // Set filter bank registers1
  CAN1->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

  CAN1->FA1R |= (0x1UL<<index);                // Activate filter

}

/**
 * Initializes the CAN controller with specified bit rate.
 *
 * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
 * @params: remap   - Select CAN port. 
 *                    =0:CAN1_RX mapped to PA11, CAN1_TX mapped to PA12
 *                       CAN2_RX mapped to PB5 , CAN2_TX mapped to PB6
 *                    =1:Not used
 *                    =2:CAN1_RX mapped to PB8,  CAN1_TX mapped to PB9 (not available on 36-pin package)
 *                       CAN2_RX mapped to PB12, CAN2_TX mapped to PB13
 *                    =3:CAN1_RX mapped to PD0,  CAN1_TX mapped to PD1 (available on 100-pin and 144-pin package)
 *                       CAN2_RX mapped to PB12, CAN2_TX mapped to PB13
 *
 */
bool CANBus::init(BITRATE bitrate, int remap) {
  // Reference manual
  // https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf

  RCC->APB1ENR |= 0x2000000UL;           // Enable CAN1 clock 
  RCC->APB1ENR |= 0x4000000UL;           // Enable CAN2 clock 
  
  if (remap == 0) {
    // CAN1
    RCC->AHB1ENR |= 0x1;                 // Enable GPIOA clock 
    this->setGpio(GPIOA, 11);               // Set PA11
    this->setGpio(GPIOA, 12);               // Set PA12
    
    // CAN2
    RCC->AHB1ENR |= 0x2;                 // Enable GPIOB clock 
    this->setGpio(GPIOB, 5);                // Set PB5
    this->setGpio(GPIOB, 6);                // Set PB6
  }

  if (remap == 2) {
    // CAN1
    RCC->AHB1ENR |= 0x2;                 // Enable GPIOB clock 
    this->setGpio(GPIOB, 8);                // Set PB8
    this->setGpio(GPIOB, 9);                // Set PB9

    // CAN2
    RCC->AHB1ENR |= 0x2;                 // Enable GPIOB clock 
    this->setGpio(GPIOB, 12);               // Set PB12
    this->setGpio(GPIOB, 13);               // Set PB13
  }
    
  if (remap == 3) {
    // CAN1
    RCC->AHB1ENR |= 0x8;                 // Enable GPIOD clock 
    this->setGpio(GPIOD, 0);                // Set PD0
    this->setGpio(GPIOD, 1);                // Set PD1

    // CAN2
    RCC->AHB1ENR |= 0x2;                 // Enable GPIOB clock 
    this->setGpio(GPIOB, 12);               // Set PB12
    this->setGpio(GPIOB, 13);               // Set PB13
  }

  CAN1->MCR |= 0x1UL;                    // Require CAN1 to Initialization mode 
  while (!(CAN1->MSR & 0x1UL));          // Wait for Initialization mode
  this->printRegister("CAN1->MCR=", CAN1->MCR);

  CAN2->MCR |= 0x1UL;                    // Require CAN2 to Initialization mode
  while (!(CAN2->MSR & 0x1UL));          // Wait for Initialization mode
  this->printRegister("CAN2->MCR=", CAN2->MCR);

  //CAN1->MCR = 0x51UL;                  // Hardware initialization(No automatic retransmission)
  CAN1->MCR = 0x41UL;                    // Hardware initialization(With automatic retransmission)

  //CAN2->MCR = 0x51UL;                  // Hardware initialization(No automatic retransmission)
  CAN2->MCR = 0x41UL;                    // Hardware initialization(With automatic retransmission)

  
  // Set bit rates 
  CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
  CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
  this->printRegister("CAN1->BTR=", CAN1->BTR);

  CAN2->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
  CAN2->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
  this->printRegister("CAN2->BTR=", CAN2->BTR);

  // Configure Filters to default values
  CAN1->FMR  |=   0x1UL;                 // Set to filter initialization mode
  CAN1->FMR  &= 0xFFFFC0FF;              // Clear CAN2 start bank
  this->printRegister("CAN1->FMR=", CAN1->FMR);

  // bxCAN has 28 filters.
  // These filters are used for both CAN1 and CAN2.
  // STM32F405 has CAN1 and CAN2, so CAN2 filters are offset by 14
  CAN1->FMR  |= 0xE00;                   // Start bank for the CAN2 interface

  // Set fileter 0
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  this->setFilter(0, 1, 0, 0, 0x0UL, 0x0UL); 

  // Set fileter 14
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  this->setFilter(14, 1, 0, 0, 0x0UL, 0x0UL); 

  CAN1->FMR   &= ~(0x1UL);               // Deactivate initialization mode

  uint16_t TimeoutMilliseconds = 1000;
  bool can2 = false;
  CAN2->MCR   &= ~(0x1UL);               // Require CAN2 to normal mode  

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN2->MSR & 0x1UL) == 0) {
      can2 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //DEBUG("can2=");
  //DEBUGLN(can2);
  if (can2) {
    DEBUGLN("CAN2 initialize ok");
  } else {
    DEBUGLN("CAN2 initialize fail!!");
  }

  bool can1 = false;
  CAN1->MCR   &= ~(0x1UL);               // Require CAN1 to normal mode 

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN1->MSR & 0x1UL) == 0) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //DEBUG("can1=");
  //DEBUGLN(can1);
  if (can1) {
    DEBUGLN("CAN1 initialize ok");
  } else {
    DEBUGLN("CAN1 initialize fail!!");
    return false;
  }
  return true;
}
 
/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions A valid CAN message is received
 * @param ch channel 1 or 2
 * @param CAN_rx_msg CAN message structure for reception
 */
void CANBus::receive(uint8_t ch, CAN_msg_t* CAN_rx_msg) {
  if(ch == 1) {
    uint32_t id = CAN1->sFIFOMailBox[0].RIR;
    if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
        CAN_rx_msg->format = STANDARD_FORMAT;;
        CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
    } 
    else {                               // Extended frame format
        CAN_rx_msg->format = EXTENDED_FORMAT;;
        CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
    }

    if ((id & STM32_CAN_RIR_RTR) == 0) {  // Data frame
        CAN_rx_msg->type = DATA_FRAME;
    }
    else {                                // Remote frame
        CAN_rx_msg->type = REMOTE_FRAME;
    }

    
    CAN_rx_msg->len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
    
    CAN_rx_msg->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
    CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
    CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
    CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
    CAN_rx_msg->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
    CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
    CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
    CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);
    
    CAN1->RF0R |= 0x20UL;
  } // end CAN1

  if(ch == 2) {
    uint32_t id = CAN2->sFIFOMailBox[0].RIR;
    if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
        CAN_rx_msg->format = STANDARD_FORMAT;;
        CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
    } 
    else {                               // Extended frame format
        CAN_rx_msg->format = EXTENDED_FORMAT;;
        CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
    }

    if ((id & STM32_CAN_RIR_RTR) == 0) {  // Data frame
        CAN_rx_msg->type = DATA_FRAME;
    }
    else {                                // Remote frame
        CAN_rx_msg->type = REMOTE_FRAME;
    }

    
    CAN_rx_msg->len = (CAN2->sFIFOMailBox[0].RDTR) & 0xFUL;
    
    CAN_rx_msg->data[0] = 0xFFUL &  CAN2->sFIFOMailBox[0].RDLR;
    CAN_rx_msg->data[1] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDLR >> 8);
    CAN_rx_msg->data[2] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDLR >> 16);
    CAN_rx_msg->data[3] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDLR >> 24);
    CAN_rx_msg->data[4] = 0xFFUL &  CAN2->sFIFOMailBox[0].RDHR;
    CAN_rx_msg->data[5] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDHR >> 8);
    CAN_rx_msg->data[6] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDHR >> 16);
    CAN_rx_msg->data[7] = 0xFFUL & (CAN2->sFIFOMailBox[0].RDHR >> 24);
    
    CAN2->RF0R |= 0x20UL;
  } // END CAN2

}
 
/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @param ch channel 1 or 2
 * @param CAN_tx_msg CAN message structure for transmission
 * @returns true if succeeds
 */
bool CANBus::send(uint8_t ch, CAN_msg_t* CAN_tx_msg) {
  volatile int count = 0;

  uint32_t out = 0;
  if (CAN_tx_msg->format == EXTENDED_FORMAT) {  // Extended frame format
      out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
  } else {  // Standard frame format
      out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
  }

  // Remote frame
  if (CAN_tx_msg->type == REMOTE_FRAME) {
      out |= STM32_CAN_TIR_RTR;
  }

  if (ch == 1) {
    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
    
    CAN1->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[2] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[0]      ));
    CAN1->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[6] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[4]      ));

    // Send Go
    CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;
/*
    // Wait until the mailbox is empty
    while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < CAN_SEND_DELAY);

    // The mailbox don't becomes empty while loop
    if (CAN1->sTxMailBox[0].TIR & 0x1UL) {
      #ifdef CAN_REPORT_FAIL
      DEBUGLN("Send Fail");
      DEBUGLN(CAN1->ESR);
      DEBUGLN(CAN1->MSR);
      DEBUGLN(CAN1->TSR);
      #endif
      return false;
    }*/
    return true;
  } // end CAN1

  if (ch == 2) {
    CAN2->sTxMailBox[0].TDTR &= ~(0xF);
    CAN2->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
    
    CAN2->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[2] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[0]      ));
    CAN2->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[6] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[4]      ));

    // Send Go
    CAN2->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

    // Wait until the mailbox is empty
    while(CAN2->sTxMailBox[0].TIR & 0x1UL && count++ < CAN_SEND_DELAY);

    // The mailbox don't becomes empty while loop
    if (CAN2->sTxMailBox[0].TIR & 0x1UL) {
      #ifdef CAN_REPORT_FAIL
      DEBUGLN("Send Fail");
      DEBUGLN(CAN2->ESR);
      DEBUGLN(CAN2->MSR);
      DEBUGLN(CAN2->TSR);
      #endif
      return false;
    }
    return true;
  } // end CAN2

  // invalid channel selection
  return false;
}

 /**
 * Returns whether there are CAN messages available.
 *
 * @param ch channel 1 or 2
 * @returns If pending CAN messages are in the CAN controller
 */
uint8_t CANBus::available(uint8_t ch) {
  if (ch == 1) {
    // Check for pending FIFO 0 messages
    return CAN1->RF0R & 0x3UL;
  } // end CAN1

  if (ch == 2) {
    // Check for pending FIFO 0 messages
    return CAN2->RF0R & 0x3UL;
  } // end CAN2

  return 0;
}


/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions A valid CAN message is received
 * @param CAN_rx_msg - CAN message structure for reception
 */
void CANBus::receive(CAN_msg_t* CAN_rx_msg) {
  receive(CAN_ACTIVE_CHANNEL,CAN_rx_msg);
}
 
/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @param CAN_tx_msg - CAN message structure for transmission
 */
bool CANBus::send(CAN_msg_t* CAN_tx_msg) {
  return send(CAN_ACTIVE_CHANNEL,CAN_tx_msg);
}

/**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 */
uint8_t CANBus::available() {
  return available(CAN_ACTIVE_CHANNEL);
}

void CANBus::setup() {
  DEBUGLN(F("CANsetup()"));

  pinMode(PIN_CAN_SILENT,OUTPUT);
  digitalWrite(PIN_CAN_SILENT,LOW);

  readAddress();

  bool ret = init(CAN_SPEED, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  if(ret) {
    DEBUGLN(F("CANsetup OK"));
    canR=0;
  } else {
    DEBUGLN(F("CANsetup FAILED"));
    canR=255;
  }

  DEBUG("CAN address=");
  DEBUGLN(CANbus.CANBusAddress);
}


void CANBus::stepTest() {
  this->writeTest();
  this->readTest();
  LEDsetColor(canR,canG,canB);
}


void CANBus::readTest() {
  if(this->available()) {
    CAN_msg_t CAN_RX_msg;
    this->receive(&CAN_RX_msg);
    countG = (countG+1)%2;
    canG=(countG==1)?255:0;
  }
}


// tested double > buffer using using https://gregstoll.com/~gregstoll/floattohex/
// sends the angle in big-endian format.
void CANBus::writeTest() {
  unsigned long currentMillis = millis();
  const long interval = 100;           // transmission interval (milliseconds)

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    countS = (countS+1)%2;
    canB=(countS==1)?255:0;

    CAN_msg_t CAN_TX_msg;
    CAN_TX_msg.id = (0x1<<7) + this->CANBusAddress;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.len = 8;
    uint8_t *samesies = (uint8_t*)&sensorAngleUnit;
    CAN_TX_msg.data[0] = samesies[0];
    CAN_TX_msg.data[1] = samesies[1];
    CAN_TX_msg.data[2] = samesies[2];
    CAN_TX_msg.data[3] = samesies[3];

#ifdef DEBUG_CAN
    DEBUG("Sending ");
    for(int i=0;i<8;++i) {
      DEBUG2(CAN_TX_msg.data[i],HEX);
    }
    DEBUGLN("");
#endif

    
    canR = this->send(&CAN_TX_msg) ? 0 : 255;
  }
}