#pragma once

/*
Designed by NamimnoRC
*/
#if DOMAIN_24GHZ
    #define TARGET_INDENTIFIER  "NRC_FLASH"
    #define GPIO_PIN_RST        PB4
    #define GPIO_PIN_BUSY       PB5
    #define GPIO_PIN_DIO0       PB6
    #define GPIO_PIN_DIO1       PB7
    #define GPIO_PIN_NSS        PA4
    #define GPIO_PIN_MOSI       PA7
    #define GPIO_PIN_MISO       PA6
    #define GPIO_PIN_SCK        PA5
    #define GPIO_PIN_LED_RED    PA1
#else // !DOMAIN_24GHZ
    #define TARGET_INDENTIFIER  "NRC_VOYAGER"
    #define GPIO_PIN_RST        PC14
    #define GPIO_PIN_DIO0       PA15
    #define GPIO_PIN_DIO1       PA1
    #define GPIO_PIN_NSS        PB12
    #define GPIO_PIN_MOSI       PB15
    #define GPIO_PIN_MISO       PB14
    #define GPIO_PIN_SCK        PB13
    #define GPIO_PIN_LED_RED    PA11
    // RF Switch: LOW = RX, HIGH = TX
    #define GPIO_PIN_TX_ENABLE  PB3
#endif // DOMAIN_24GHZ

#define GPIO_PIN_RCSIGNAL_RX    PA10
#define GPIO_PIN_RCSIGNAL_TX    PA9
