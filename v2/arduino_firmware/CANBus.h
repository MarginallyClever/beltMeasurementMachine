#pragma once
//-----------------------------------------------------------------------------
// CAN bus for Daisy Driver
// from https://github.com/nopnop2002/Arduino-STM32-CAN/tree/master/stm32f103
//-----------------------------------------------------------------------------

//#define CAN_DEBUG  // uncomment to turn on serial debugging

// Symbolic names for bit rate of CAN message
typedef enum {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

// Real speed for bit rate of CAN message
uint32_t SPEED[6] = {50*1000, 100*1000, 125*1000, 250*1000, 500*1000, 1000*1000};

// Symbolic names for formats of CAN message
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

// Symbolic names for type of CAN message
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
  uint32_t id;        // 29 bit identifier
  uint8_t  data[8];   // Data field
  uint8_t  len;       // Length of data field in bytes
  uint8_t  ch;        // Object channel(Not use)
  uint8_t  format;    // 0 - STANDARD, 1- EXTENDED IDENTIFIER
  uint8_t  type;      // 0 - DATA FRAME, 1 - REMOTE FRAME
} CAN_msg_t;

typedef struct {
    uint16_t baud_rate_prescaler;                /// [1 to 1024]
    uint8_t time_segment_1;                      /// [1 to 16]
    uint8_t time_segment_2;                      /// [1 to 8]
    uint8_t resynchronization_jump_width;        /// [1 to 4] (recommended value is 1)
} CAN_bit_timing_config_t;

//-----------------------------------------------------------------------------

extern uint8_t CANBusAddress;

//-----------------------------------------------------------------------------

// Calculation of bit timing dependent on peripheral clock rate
extern int16_t CANComputeTimings(const uint32_t peripheral_clock_rate,
                          const uint32_t target_bitrate,
                          CAN_bit_timing_config_t* const out_timings);

// Print registers.
extern void CANPrintRegister(const char * buf, uint32_t reg);

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
extern void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);

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
extern bool CANInit(BITRATE bitrate, int remap);


/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 */
extern void CANReceive(CAN_msg_t* CAN_rx_msg);
 
/**
 * Encodes CAN messages using the CAN message struct and populates the 
 * data registers with the sent.
 * 
 * @params CAN_tx_msg - CAN message structure for transmission
 * 
 */
extern bool CANSend(CAN_msg_t* CAN_tx_msg);

/**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 *
 */
extern uint8_t CANMsgAvail(void);


extern void CANsetup();
extern void CANstep();