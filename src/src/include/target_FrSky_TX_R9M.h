#pragma once

/*
 * FrSky R9M 1W module
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

#define GPIO_PIN_DIP1        PA12 // dip switch 1
#define GPIO_PIN_DIP2        PA11 // dip switch 2

#define GPIO_PIN_RFamp_APC1       PA6 //CONFIRMED SANDRO// APC2 is connected through a I2C dac and is handled elsewhere
#define GPIO_PIN_RFswitch_CONTROL PB3 //CONFIRMED SANDRO HIGH = RX, LOW = TX
// PwrAmp, RFControl, dodgy measurement with SDR, descending
// high low  -5
// low  low  -30
// low  high -40
// high high -40

// Serial1 is connected to internal ESP module if in use
#define CTRL_SERIAL Serial1
#define DEFINE_SERIAL1
#define DEBUG_SERIAL Serial1

// Serial2 is connected to external pins of R9M
#if TELEMETRY_EXTERNAL
#error "TELEMETRY_EXTERNAL is not needed!"
#define BT_SERIAL       Serial2
#define DEFINE_SERIAL2
#ifndef TELEMETRY_EXTERNAL_BAUDRATE
#define TELEMETRY_EXTERNAL_BAUDRATE 57600
#endif /* TELEMETRY_EXTERNAL_BAUDRATE */
#define BT_SERIAL_BAUD  TELEMETRY_EXTERNAL_BAUDRATE
#endif /* TELEMETRY_EXTERNAL */

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
        {10, 10, 8, 650},       \
        {25, 14, 12, 860},      \
        {50, 17, 15, 1000},     \
        {100, 20, 18, 1160},    \
        {250, 24, 22, 1420},    \
        {500, 27, 25, 1730},    \
        {1000, 30, 28, 2100},   \
        {2000, 33, 31, 2600}, // Danger untested at high power
#else
    // 915MHz
    #define DAC_LUT_VALUES      \
        {10, 10, 8, 720},       \
        {25, 14, 12, 875},      \
        {50, 17, 15, 1000},     \
        {100, 20, 18, 1140},    \
        {250, 24, 22, 1390},    \
        {500, 27, 25, 1730},    \
        {1000, 30, 28, 2100},   \
        {2000, 33, 31, 2600}, // Danger untested at high power
#endif
