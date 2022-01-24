#pragma once

/*
 * Version 0.1 and 0.2
 */

// SPI pins
#define GPIO_PIN_MOSI      PA7
#define GPIO_PIN_MISO      PA6
#define GPIO_PIN_SCK       PA5

// Radio GPIOs (SX1280)
#define GPIO_PIN_NSS_128x  PA12
#define GPIO_PIN_DIO0_128x PD2
#define GPIO_PIN_DIO1_128x UNDEF_PIN
#define GPIO_PIN_RST_128x  PA15
#define GPIO_PIN_BUSY      PA14
#define GPIO_PIN_TXEN_128x PC10
#define GPIO_PIN_RXEN_128x PA13
// Radio GPIOs (SX1276)
#define GPIO_PIN_NSS_127x  PB1
#define GPIO_PIN_DIO0_127x PB0
#define GPIO_PIN_RST_127x  PB2

// V0.2
#define GPIO_PIN_LED_RGB   PA11 // WS2812 RGB

// ESPbackpack logger (USART1)
#define CTRL_SERIAL Serial1
//#define CTRL_SERIAL_BAUD    420000
//#define CTRL_SERIAL_BAUD    460800
#define CTRL_SERIAL_BAUD    921600
#define DEFINE_SERIAL1
#define SERIAL1_USE_DMA     1
#define PRINTF_NUM_BLOCKS   128
#define PRINTF_BUFF_SIZE    256

// Both radios are included
#define RADIO_SX128x 1
#define RADIO_SX127x 1

// Gimbal pins
#define GIMBAL_L1       PC1 // G_R_1
#define GIMBAL_L2       PC0 // G_R_2
#define GIMBAL_R1       PC3 // G_L_1
#define GIMBAL_R2       PC2 // G_L_2

#define SWITCH_1_1  PB3     // S1_1
#define SWITCH_1_2  PB4     // S1_2
#define SWITCH_2_1  PB5     // S2_1
//#define SWITCH_2_2  PB6     // S2_2
#define SWITCH_3_1  PB14    // S3_1
#define SWITCH_3_2  PB15    // S3_2
//#define SWITCH_4_1  PB12    // S4_1
//#define SWITCH_4_2  PB13    // S4_2
#define SWITCH_4_1  PB13    // S4_2
#define SWITCH_5_1  PC12    // S5_1
#define SWITCH_5_2  PC11    // S5_2
#define SWITCH_6_1  PB9     // S6_1
#define SWITCH_6_2  PB8     // S6_2

// ********** OUTPUT MAPPING **********
#define SWITCH_CH_1     SWITCH_3
#define SWITCH_CH_2     SWITCH_1
#define SWITCH_CH_3     SWITCH_2
#define SWITCH_CH_4     SWITCH_4
#define SWITCH_CH_5     SWITCH_5
#define SWITCH_CH_6     SWITCH_6


#define DEBUG_SERIAL CTRL_SERIAL

/* Radio modules */
//#define TARGET_MODULE_LORA1276F30
#define TARGET_MODULE_E28_VER   27 // 500mW
