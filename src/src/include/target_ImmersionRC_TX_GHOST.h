#pragma once

/*
ImmersionRC Ghost TX Module
*/
#define RADIO_SX128x         1
// SPI pins
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5
// Radio GPIOs (SX1280)
#define GPIO_PIN_NSS         PA15
#define GPIO_PIN_DIO0        PB2
#define GPIO_PIN_BUSY        PB15
#define GPIO_PIN_TX_ENABLE   PA8
#define GPIO_PIN_RX_ENABLE   PB14
// Radio GPIOs common
#define GPIO_PIN_RST         PB0
// LED
#define GPIO_PIN_LED_RGB     PB6 // WS2812 RGB
// S.Port
#define GPIO_PIN_RCSIGNAL_RX UNDEF_PIN // Not used
#define GPIO_PIN_RCSIGNAL_TX PA10 // One wire, swapped
#define RCSIGNAL_INVERTED    1
#define RCSIGNAL_USE_DMA     0

//#define GPIO_PIN_BUZZER      PC13

// https://www.skyworksinc.com/-/media/SkyWorks/Documents/Products/2101-2200/SE2622L_202733C.pdf
#define GPIO_PIN_PA_ENABLE   PB11

#define GPIO_PIN_ANT_CTRL_1  PA9
#define GPIO_PIN_ANT_CTRL_2  PB13

#if TARGET_TX_GHOST_LITE
    #define GPIO_PIN_RF_AMP_DET  PA3  // not used atm
#endif // TARGET_TX_GHOST_LITE

#if TARGET_TX_GHOST_LITE
    #define TARGET_PWR_MAX_SX128x   PWR_250mW
    //#define TARGET_PWR_LUT_SX128x   -16,-14,-11,-8,-4,-4,-4,-4
    #define TARGET_PWR_LUT_SX128x   -18,-17,-14,-10,-4,-4,-4,-4
#else
    #define TARGET_PWR_MAX_SX128x   PWR_250mW
    #define TARGET_PWR_LUT_SX128x   -18,-17,-14,-10,-4,-4,-4,-4
#endif
