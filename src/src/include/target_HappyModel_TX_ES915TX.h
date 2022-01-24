#pragma once

/*
 * HappyModel ES915TX 1W module
 */

#define GPIO_PIN_NSS         PB12
#define GPIO_PIN_MOSI        PB15
#define GPIO_PIN_MISO        PB14
#define GPIO_PIN_SCK         PB13
#define GPIO_PIN_RST         PC14
#define GPIO_PIN_SDA         PB7
#define GPIO_PIN_SCL         PB6
#define GPIO_PIN_RCSIGNAL_RX PB11 // USART3 RX for S.Port
#define GPIO_PIN_RCSIGNAL_TX PB10 // USART3 TX for S.Port, needs BUFFER_OE

#define GPIO_PIN_DIO0        PA15
#define GPIO_PIN_LED_RED     PA11 // Red LED
#define GPIO_PIN_LED_GREEN   PA12 // Green LED
#define GPIO_PIN_BUTTON      PA8  // pullup e.g. LOW when pressed
#define GPIO_PIN_BUZZER      PB1  // confirmed

#define GPIO_PIN_RFamp_APC1       PA6 //CONFIRMED SANDRO// APC2 is connected through a I2C dac and is handled elsewhere
#define GPIO_PIN_RFswitch_CONTROL PB3 //CONFIRMED SANDRO HIGH = RX, LOW = TX
// PwrAmp, RFControl, dodgy measurement with SDR, descending
// high low  -5
// low  low  -30
// low  high -40
// high high -40

/* R9M DAC control address */
#define DAC_I2C_ADDRESS         0b0001100

//#define GPIO_PIN_DIO1 PA1  //Not Needed, HEARTBEAT pin

//#define GPIO_PIN_DEBUG_RX PA3 // confirmed, USART2
//#define GPIO_PIN_DEBUG_TX PA2 // confirmed, USART2

#define BUFFER_OE     PA5  //CONFIRMED

#define GPIO_PIN_LED_RED_INV    0
#define GPIO_PIN_LED_GREEN_INV  0

/* Power control */
#define DAC_IN_USE              1
#define TARGET_PWR_MAX_SX127x   PWR_1000mW
#define TARGET_PWR_LUT_SX127x   0,0,0,0,0,0,0,0


#if defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_EU_868_R9)
    // 868MHz
    #define DAC_LUT_VALUES      \
        {10, 10, 8, 375},       \
        {25, 14, 12, 850},      \
        {50, 17, 15, 1200},     \
        {100, 20, 18, 1400},    \
        {250, 24, 22, 1700},    \
        {500, 27, 25, 2000},    \
        {1000, 30, 28, 2000}, /* limit to 500mW */  \
        {2000, 33, 31, 2000}, /* limit to 500mW */
#else
    // 915MHz
    #define DAC_LUT_VALUES      \
        {10, 10, 8, 875},       \
        {25, 14, 12, 1065},     \
        {50, 17, 15, 1200},     \
        {100, 20, 18, 1355},    \
        {250, 24, 22, 1600},    \
        {500, 27, 25, 1900},    \
        {1000, 30, 28, 2400},   \
        {2000, 33, 31, 2600}, // Danger untested at high power
#endif
