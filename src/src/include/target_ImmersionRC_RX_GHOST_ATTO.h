#pragma once

/*
ImmersionRC Ghost ATTO RX Module
*/
#define RADIO_SX128x         1
// SPI pins
#define GPIO_PIN_MOSI        PB5
#define GPIO_PIN_MISO        PB4
#define GPIO_PIN_SCK         PB3
// Radio GPIOs (SX1280)
#define GPIO_PIN_NSS         PA15
#define GPIO_PIN_DIO0        PA1
#define GPIO_PIN_RST         PB0
#define GPIO_PIN_BUSY        PA3
// CRSF
#define GPIO_PIN_RCSIGNAL_RX PB6 // USART1, half duplex
#define GPIO_PIN_RCSIGNAL_TX PA2 // USART2, half duplex
// LED
#define GPIO_PIN_LED_RGB     PA7 // WS2812 RGB

#define TARGET_INDENTIFIER   "GHST_ATTO"
