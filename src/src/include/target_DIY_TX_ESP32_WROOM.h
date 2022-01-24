#pragma once

// Radio GPIOs
#if ESP32_WROOM_SLIMMER
    #define GPIO_PIN_DIO0      4
    #define GPIO_PIN_DIO1      2
    #define GPIO_PIN_RST       14
    #define GPIO_PIN_BUSY      21
    #define GPIO_PIN_TX_ENABLE 26
    #define GPIO_PIN_RX_ENABLE 27
#elif NAMIMNORC_TX_OLED
    // NamimnoRC Flash OLED, 2400MHz TX module
    #define GPIO_PIN_DIO0      17
    #define GPIO_PIN_DIO1      16
    #define GPIO_PIN_RST       21
    #define GPIO_PIN_BUSY      22
    #define GPIO_PIN_TX_ENABLE 33
    #define GPIO_PIN_RX_ENABLE 32
    #define GPIO_PIN_PA_ENABLE 25
    #define GPIO_PIN_FAN_CTRL  2
    #define GPIO_PIN_LED_RGB   4
#else
    #define GPIO_PIN_DIO0      4
    #define GPIO_PIN_DIO1      16
    #define GPIO_PIN_RST       15
    #define GPIO_PIN_BUSY      2
    #define GPIO_PIN_TX_ENABLE 26
    #define GPIO_PIN_RX_ENABLE 17
#endif

// SPI pins
#define GPIO_PIN_NSS       5  // V_SPI_CS0
#define GPIO_PIN_MOSI      23 // V_SPI
#define GPIO_PIN_MISO      19 // V_SPI
#define GPIO_PIN_SCK       18 // V_SPI
// S.Port
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13

#if (WIFI_LOGGER && !WIFI_UPDATER) || ESP_NOW
#define CTRL_SERIAL
#endif

/* Radio power control */
#if TARGET_HM_ES24TX
    #define TARGET_PWR_MAX_SX128x   PWR_250mW
    #define TARGET_PWR_LUT_SX128x   -17,-13,-9,-6,-2,-2,-2,-2
#elif NAMIMNORC_TX_OLED
    #define TARGET_PWR_MAX_SX128x   PWR_1000mW
    #define TARGET_PWR_LUT_SX128x   -18,-18,-13,-10,-5,2,3,3
#elif !defined(TARGET_MODULE_LORA1280F27)
    #define TARGET_MODULE_E28_VER   27 // 500mW
#endif
