#pragma once

/*
Designed by NamimnoRC
*/
#if DOMAIN_24GHZ
    #define GPIO_PIN_RST            PB4
    #define GPIO_PIN_BUSY           PB5
    #define GPIO_PIN_DIO0           PB6
    #define GPIO_PIN_DIO1           PB7
    #define GPIO_PIN_NSS            PA4
    #define GPIO_PIN_MOSI           PA7
    #define GPIO_PIN_MISO           PA6
    #define GPIO_PIN_SCK            PA5
    // SKY65383-11 front end control
    #define GPIO_PIN_RX_ENABLE      PA8     // CRX
    #define GPIO_PIN_TX_ENABLE      PA11    // CTX
    #define GPIO_PIN_PA_ENABLE      PA12    // CSD
    #define RADIO_SX128x            1
    /* Power control */
    #define TARGET_PWR_MAX_SX128x   PWR_1000mW
    #define TARGET_PWR_LUT_SX128x   -18,-18,-15,-12,-8,-5,3,3
#else // !DOMAIN_24GHZ
    #define GPIO_PIN_NSS            PB12
    #define GPIO_PIN_DIO0           PA15
    #define GPIO_PIN_MOSI           PB15
    #define GPIO_PIN_MISO           PB14
    #define GPIO_PIN_SCK            PB13
    #define GPIO_PIN_RST            PC14
    #define GPIO_PIN_RX_ENABLE      PB3  //HIGH = RX, LOW = TX
    /* DAC settings */
    #define GPIO_PIN_SDA            PB9
    #define GPIO_PIN_SCL            PB8
    #define DAC_I2C_ADDRESS         0b0001101
    #define RADIO_SX127x            1
    /* Power control */
    #define DAC_IN_USE              1
    #define TARGET_PWR_MAX_SX127x   PWR_1000mW
    #define TARGET_PWR_LUT_SX127x   0,0,0,0,0,0,0,0
#endif // DOMAIN_24GHZ

/* S.Port input signal */
#define GPIO_PIN_RCSIGNAL_RX    PB11 /* USART3 */
#define GPIO_PIN_RCSIGNAL_TX    PB10 /* USART3 */
#define BUFFER_OE               PA1
#define BUFFER_OE_INVERTED      1
#define RCSIGNAL_USE_DMA        0
#define GPIO_PIN_FAN_CTRL       PB1

/* Backpack logger connection */
#define CTRL_SERIAL Serial1
#define DEFINE_SERIAL1
#define DEBUG_SERIAL Serial1

/* WS2812 led */
#define GPIO_PIN_LED_RGB        PB0


// DAC init value for 900 TX module
#if defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_EU_868_R9)
    // 868MHz
    #define DAC_LUT_VALUES   \
        {10, 10, 8, 895},    \
        {25, 14, 12, 1030},  \
        {50, 17, 15, 1128},  \
        {100, 20, 18, 1240}, \
        {250, 24, 22, 1465}, \
        {500, 27, 25, 1700}, \
        {1000, 30, 28, 2050},\
        {2000, 33, 31, 2600},
#else
    // 915MHz
    #define DAC_LUT_VALUES   \
        {10, 10, 8, 500},    \
        {25, 14, 12, 860},   \
        {50, 17, 15, 1000},  \
        {100, 20, 18, 1170}, \
        {250, 24, 22, 1460}, \
        {500, 27, 25, 1730}, \
        {1000, 30, 28, 2100},\
        {2000, 33, 31, 2600},
#endif
