#pragma once

#define GPIO_PIN_NSS         15
#define GPIO_PIN_MOSI        13
#define GPIO_PIN_MISO        12
#define GPIO_PIN_SCK         14
#define GPIO_PIN_RST         2
#define GPIO_PIN_LED         16
#if !SERVO_OUTPUTS_ENABLED
    #define GPIO_PIN_BUTTON     0
#endif
#if TARGET_IFLIGHT_RX_2400
    #define GPIO_PIN_DIO0       5
    #define GPIO_PIN_BUSY       4
    #define GPIO_PIN_RX_ENABLE  9
    #define GPIO_PIN_TX_ENABLE  10
    #ifndef TARGET_INDENTIFIER
        #define TARGET_INDENTIFIER  "2400IFLIGHTRX"
    #endif
    #define SX1280_REGULATOR_MODE_DCDC 1
#elif TARGET_BETAFPV_NANO_RX_2400
    #define GPIO_PIN_DIO0       4
    #define GPIO_PIN_BUSY       5
    #define GPIO_PIN_RX_ENABLE  9
    #define GPIO_PIN_TX_ENABLE  10
    #ifndef TARGET_INDENTIFIER
        #define TARGET_INDENTIFIER  "2400BETAFPVRX"
    #endif
    #define SX1280_REGULATOR_MODE_DCDC 1
    #define RECEIVER_TRANSMIT_POWER 1
#elif DOMAIN_24GHZ
    #define GPIO_PIN_DIO0       4
    #define GPIO_PIN_BUSY       5
    #ifndef TARGET_INDENTIFIER
        #define TARGET_INDENTIFIER  "2400ESPRX"
    #endif
#elif TARGET_NAMIMNORC_900_ESP_RX
    #define GPIO_PIN_DIO0       5
    #define GPIO_PIN_DIO1       4
    #ifndef TARGET_INDENTIFIER
        #define TARGET_INDENTIFIER  "900VOYAGERRX"
    #endif
#else
    #define GPIO_PIN_DIO0       4
    #define GPIO_PIN_DIO1       5
    #ifndef TARGET_INDENTIFIER
        #define TARGET_INDENTIFIER  "900ESPRX"
    #endif
#endif

#if SERVO_OUTPUTS_ENABLED
    #define SERVO_PIN_CH1 0     // BOOT button
    #define SERVO_PIN_CH2 1     // TXD0
    #define SERVO_PIN_CH3 3     // RXD0
    #define SERVO_PIN_CH4 9
    #define SERVO_PIN_CH5 10
#endif
