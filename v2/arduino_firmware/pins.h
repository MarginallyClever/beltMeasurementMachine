#pragma once
//-----------------------------------------------------------------------------
// Pins for STM32F405RGTx microcontroller.
// The names match those given in the schematic diagram.
//-----------------------------------------------------------------------------
// TMC stepper driver
#define PIN_TMC_SKIP PC13
#define PIN_TMC_STALL PC14
#define PIN_TMC_SPI_MODE PC15
#define PIN_TMC_STEP_EN PB0
#define PIN_PWM_TMC_CURRSET PC6
#define PIN_TMC_STEP PB13
#define PIN_TMC_DIR PB12
#define PIN_TMC_EN PB14
#define PIN_TMC_STEP_GATE PB15
#define PIN_TMC_STEP_RDY PA15

// SPI to the TMC driver
#define PIN_SPI1_TMC_CS PA4
#define PIN_SPI1_CLK PA5
#define PIN_SPI1_MISO PA6
#define PIN_SPI1_MOSI PA7

// IPS2200 absolute rotation sensor
#define PIN_IPS_COS PA0
#define PIN_IPS_COSN PA1
#define PIN_IPS_SIN PA2
#define PIN_IPS_SINN PA3

// RGB status led
#define PIN_PWM_RGB_G PC7
#define PIN_PWM_RGB_R PC8
#define PIN_PWM_RGB_B PC9

// CAN bus
#define PIN_CAN_RX PB8
#define PIN_CAN_TX PB9
#define PIN_CAN_SILENT PB5

// CAN address pins

#define PIN_CAN_ADDR0 PC0
#define PIN_CAN_ADDR1 PC1
#define PIN_CAN_ADDR2 PC2
#define PIN_CAN_ADDR3 PC3
#define PIN_CAN_ADDR4 PC4
#define PIN_CAN_ADDR5 PC5

// all other pins
#define PIN_SWD_SWO PB3
#define PIN_VBUS_DET PA9
#define PIN_NRST NRST
#define PIN_BOOT0 BOOT0
#define PIN_BOOT1 PB_2

#define PIN_USART_TX PC10
#define PIN_USART_RX PC11

#define PIN_I2C_SCL PB10
#define PIN_I2C_SDA PB11

#define PIN_MCU_USB_DNEG PA11
#define PIN_MCU_USB_DPOS PA12

#define PIN_SWD_SWDIO PA13
#define PIN_SWD_CLK PA14
