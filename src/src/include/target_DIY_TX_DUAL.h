#pragma once

/*
 * Version v0.3
 */

// SPI pins
#define GPIO_PIN_MOSI      PA7
#define GPIO_PIN_MISO      PA6
#define GPIO_PIN_SCK       PA5

// Radio GPIOs (SX1280)
#define GPIO_PIN_NSS_128x  PB0
#define GPIO_PIN_DIO0_128x PB15
#define GPIO_PIN_DIO1_128x PA8
#define GPIO_PIN_BUSY      PB14
#define GPIO_PIN_TXEN_128x PB12
#define GPIO_PIN_RXEN_128x PB1
// Radio GPIOs (SX1276)
#define GPIO_PIN_NSS_127x  PA12
#define GPIO_PIN_DIO0_127x PA11
// Radio GPIOs common
#define GPIO_PIN_RST       PB13

#define GPIO_PIN_LED_RGB   PC13 // WS2812 RGB

// S.Port (USART2)
#define GPIO_PIN_RCSIGNAL_RX PA3
#define GPIO_PIN_RCSIGNAL_TX PA2
#define BUFFER_OE            PA1
#define BUFFER_OE_INVERTED   1
#define RCSIGNAL_USE_DMA     0

// ESPbackpack logger (USART1)
#define CTRL_SERIAL Serial1
#define CTRL_SERIAL_BAUD    921600
#define DEFINE_SERIAL1
#define DEBUG_SERIAL Serial1

// Both radios are included
#define RADIO_SX128x 1
#define RADIO_SX127x 1

/* Radio modules */
#define TARGET_MODULE_LORA1276F30   1
#if !TARGET_MODULE_E28_VER
#define TARGET_MODULE_E28_VER       27 // 500mW
#endif
