#pragma once

#define GPIO_PIN_NSS         15
#define GPIO_PIN_MOSI        13
#define GPIO_PIN_MISO        12
#define GPIO_PIN_SCK         14
#define GPIO_PIN_RST         2
#define GPIO_PIN_LED         16
#define GPIO_PIN_BUTTON      0
#if DOMAIN_24GHZ
    #define GPIO_PIN_DIO0    4
    #define GPIO_PIN_BUSY    5
    #define TARGET_INDENTIFIER "2400ESPRX"
#elif TARGET_NAMIMNORC_900_ESP_RX
    #define GPIO_PIN_DIO0    5
    #define GPIO_PIN_DIO1    4
    #define TARGET_INDENTIFIER "900VOYAGERRX"
#else
    #define GPIO_PIN_DIO0    4
    #define GPIO_PIN_DIO1    5
    #define TARGET_INDENTIFIER "900ESPRX"
#endif
