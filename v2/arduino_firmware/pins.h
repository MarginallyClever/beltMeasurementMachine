#pragma once
//-----------------------------------------------------------------------------
// Pins for STM32F103C8T6 microcontroller.
// The names match those given in the schematic diagram.
//-----------------------------------------------------------------------------
// TMC stepper driver
#define PIN_TMC_SKIP PC13  // 2
#define PIN_TMC_STALL PC14  // 3
#define PIN_TMC_SPI_MODE PC15  // 4
#define PIN_TMC_STEP_EN PB0  // 18
#define PIN_PWM_TMC_CURRSET PB1  // 19
#define PIN_TMC_STEP PB12  // 25
#define PIN_TMC_DIR PB13  // 26
#define PIN_TMC_EN PB14  // 27
#define PIN_TMC_STEP_GATE PB15  // 28
#define PIN_TMC_STEP_RDY PA15  // 38

// SPI to the TMC driver
#define PIN_SPI_TMC_CS PA4  // 14
#define PIN_SPI_CLK PA5  // 15
#define PIN_SPI_MISO PA6  // 16
#define PIN_SPI_MOSI PA7  // 17

// IPS2200 absolute rotation sensor
#define PIN_IPS_COS PA0  // 10
#define PIN_IPS_COSN PA1  // 11
#define PIN_IPS_SIN PA2  // 12
#define PIN_IPS_SINN PA3  // 13

// RGB status led
#define PIN_PWM_RGB_B PA8  // 29
#define PIN_PWM_RGB_R PA9  // 30
#define PIN_PWM_RGB_G PA10  // 31

// CAN bus
#define PIN_CAN_RX PB8  // 45
#define PIN_CAN_TX PB9  // 46
#define PIN_CAN_SILENT PB5  // 41

// all other pins
#define PIN_SWD_SWO PB3  // 39
#define PIN_VBUS_DET PB4  // 40
#define PIN_NRST NRST  // 7
#define PIN_BOOT0 BOOT0  // 44
#define PIN_BOOT1 PB2  // 20

#define PIN_USART_TX PB6  // 42
#define PIN_USART_RX PB7  // 43

#define PIN_I2C_SCL PB10  // 21
#define PIN_I2C_SDA PB11  // 22

#define PIN_MCU_USB_DNEG PA11  // 32
#define PIN_MCU_USB_DPOS PA12  // 33

#define PIN_SWD_SWDIO PA13  // 34
#define PIN_SWD_CLK PA14  // 37