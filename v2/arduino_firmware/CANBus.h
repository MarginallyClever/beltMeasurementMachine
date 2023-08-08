#pragma once
//-----------------------------------------------------------------------------
// CAN bus for Daisy Driver
// from https://github.com/nopnop2002/Arduino-STM32-CAN/blob/master/stm32f407/stm32f407.ino
//-----------------------------------------------------------------------------

// stm32f405 has two channels.  Valid values are 1 or 2
// Channel 1 is pins PB8/PB9, so that's the only good setting for this board.
#define CAN_ACTIVE_CHANNEL 1  

//-----------------------------------------------------------------------------

/* Symbolic names for bit rate of CAN message                                */
typedef enum {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

/* Symbolic names for formats of CAN message                                 */
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {DATA_FRAME = 0, REMOTE_FRAME}         CAN_FRAME;

//-----------------------------------------------------------------------------

#define CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE     1000
#define CAN_STM32_ERROR_MSR_INAK_NOT_SET         1001
#define CAN_STM32_ERROR_MSR_INAK_NOT_CLEARED     1002
#define CAN_STM32_ERROR_UNSUPPORTED_FRAME_FORMAT 1003

#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU

//-----------------------------------------------------------------------------

typedef struct {
  uint32_t id;        /* 29 bit identifier                               */
  uint8_t  data[8];   /* Data field                                      */
  uint8_t  len;       /* Length of data field in bytes                   */
  uint8_t  ch;        /* Object channel(Not use)                         */
  uint8_t  format;    /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
  uint8_t  type;      /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef const struct {
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
} CAN_bit_timing_config_t;

//-----------------------------------------------------------------------------
// macros for working with CAN_msg_t

#define CAN_START(canMsg)        {  \
  canMsg.format = STANDARD_FORMAT;  \
  canMsg.type = DATA_FRAME;         \
  canMsg.len=0;                     }

#define CAN_ADD_SHORT(canMsg,v)  {  canMsg.data[canMsg.len++] = ((v)&0xFF);  }

#define CAN_ADD_LONG(canMsg,v)   {  \
  CAN_ADD_SHORT(canMsg,((v>>8)&0xFF));  \
  CAN_ADD_SHORT(canMsg,((v)&0xFF));     }

#define CAN_ADD_FLOAT(canMsg,v)  {  \
  uint8_t *samesies = (uint8_t*)&(v);  \
  CAN_ADD_SHORT(canMsg,(samesies[3]));  \
  CAN_ADD_SHORT(canMsg,(samesies[2]));  \
  CAN_ADD_SHORT(canMsg,(samesies[1]));  \
  CAN_ADD_SHORT(canMsg,(samesies[0]));  }

#define CAN_GET_SHORT(canMsg,v)  canMsg.data[v++]

#define CAN_GET_LONG(canMsg,v)   ((uint16_t)CAN_GET_SHORT(canMsg,v) << 8) | ((uint16_t)CAN_GET_SHORT(canMsg,v))

inline float CAN_GET_FLOAT(CAN_msg_t &canMsg,uint8_t &v) {
  uint32_t i =  (((uint32_t)CAN_GET_SHORT(canMsg,v) << 24) |  
                 ((uint32_t)CAN_GET_SHORT(canMsg,v) << 16) |  
                 ((uint32_t)CAN_GET_SHORT(canMsg,v) <<  8) |  
                 ((uint32_t)CAN_GET_SHORT(canMsg,v)      ));
  float r = *(float*)&i;
  return r;
}

//-----------------------------------------------------------------------------
class CANBus {
  private:
    /**
    * Initializes the CAN GPIO registers.
    *
    * @params: addr    - Specified GPIO register address.
    * @params: index   - Specified GPIO index.
    * @params: speed   - Specified OSPEEDR register value.(Optional)
    *
    */
    void setGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t speed = 3);

    // Calculation of bit timing dependent on peripheral clock rate
    int16_t computeTimings(const uint32_t peripheral_clock_rate,
                              const uint32_t target_bitrate,
                              CAN_bit_timing_config_t* const out_timings);

    // Print registers.
    void printRegister(const char * buf, uint32_t reg);

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
    */
    void setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);

    /**
    * Decodes CAN messages from the data registers and populates a 
    * CAN message struct with the data fields.
    * 
    * @preconditions A valid CAN message is received
    * @param ch channel 1 or 2
    * @param CAN_rx_msg - CAN message structure for reception
    */
    void receive(uint8_t ch,CAN_msg_t* CAN_rx_msg);
    
    /**
    * Encodes CAN messages using the CAN message struct and populates the 
    * data registers with the sent.
    * 
    * @param ch channel 1 or 2
    * @param CAN_tx_msg - CAN message structure for transmission
    */
    bool send(uint8_t ch,CAN_msg_t* CAN_tx_msg);

    /**
    * Returns whether there are CAN messages available.
    *
    * @param ch channel 1 or 2
    * @returns If pending CAN messages are in the CAN controller
    */
    uint8_t available(uint8_t ch);


  public:

    uint8_t CANBusAddress;


    /**
    * Initializes the CAN controller with specified bit rate.
    *
    * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
    * @params: remap   - Select CAN port. 
    *                    =0:CAN_RX mapped to PA11, CAN_TX mapped to PA12
    *                    =1:Not used
    *                    =2:CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)
    *                    =3:CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)
    */
    bool init(BITRATE bitrate, int remap);

    /**
    * Decodes CAN messages from the data registers and populates a 
    * CAN message struct with the data fields.
    * 
    * @preconditions A valid CAN message is received
    * @param CAN_rx_msg - CAN message structure for reception
    */
    void receive(CAN_msg_t* CAN_rx_msg);
    
    /**
    * Encodes CAN messages using the CAN message struct and populates the 
    * data registers with the sent.
    * 
    * @param CAN_tx_msg - CAN message structure for transmission
    */
    bool send(CAN_msg_t* CAN_tx_msg);

    /**
    * @returns If pending CAN messages are in the CAN controller
    */
    uint8_t available();

    void readAddress();
    void setup();

    void stepTest();
    void readTest();
    void writeTest();
};

//-----------------------------------------------------------------------------

extern CANBus CANbus;