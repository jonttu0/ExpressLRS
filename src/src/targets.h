#pragma once

#include "Arduino.h"

#define EMPTY()
#define UNDEF_PIN (-1)

/// General Features ///
#define LED_MAX_BRIGHTNESS 50 //0..255 for max led brightness

/******************************************************************************************/
/*                                     ESP TX CONFIGS                                     */
/******************************************************************************************/

#if defined(TARGET_TTGO_LORA_V1_AS_TX)
#define GPIO_PIN_NSS         18
#define GPIO_PIN_DIO0        26
#define GPIO_PIN_DIO1        -1
#define GPIO_PIN_MOSI        27
#define GPIO_PIN_MISO        19
#define GPIO_PIN_SCK         5
#define GPIO_PIN_RST         14
#define GPIO_PIN_OLED_SDA    4
#define GPIO_PIN_OLED_SCK    15
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
//#define GPIO_PIN_LED         22 // 22 is not ok

#if defined(WIFI_LOGGER) && !defined(WIFI_UPDATER)
#define CTRL_SERIAL
#endif


#elif defined(TARGET_TTGO_LORA_V2_AS_TX)
#define GPIO_PIN_NSS         18
#define GPIO_PIN_DIO0        26
#define GPIO_PIN_DIO1        -1
#define GPIO_PIN_MOSI        27
#define GPIO_PIN_MISO        19
#define GPIO_PIN_SCK         5
#define GPIO_PIN_RST         14
#define GPIO_PIN_OLED_SDA    21
#define GPIO_PIN_OLED_SCK    22
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13


#elif defined(TARGET_ESP32_WROOM_RFM95)
#define GPIO_PIN_NSS       5  // V_SPI_CS0
#define GPIO_PIN_DIO0      35 //26
#define GPIO_PIN_DIO1      34 //25
#define GPIO_PIN_MOSI      23 // V_SPI
#define GPIO_PIN_MISO      19 // V_SPI
#define GPIO_PIN_SCK       18 // V_SPI
#define GPIO_PIN_RST       -1 //14  // not connected ATM
// so we don't have to solder the extra resistor,
// we switch rx/tx using gpio mux
#define GPIO_PIN_RCSIGNAL_RX 2
#define GPIO_PIN_RCSIGNAL_TX 2


/******************************************************************************************/
/*                                     SX1280 CONFIGS                                     */
/******************************************************************************************/

#elif defined(TARGET_SX1280_TX_ESP32_WROOM)
// Radio GPIOs
#if TARGET_SX1280_TX_ESP32_WROOM_SLIMMER
#define GPIO_PIN_DIO0      4
#define GPIO_PIN_DIO1      2
#define GPIO_PIN_RST       14
#define GPIO_PIN_BUSY      21
#define GPIO_PIN_TX_ENABLE 26
#define GPIO_PIN_RX_ENABLE 27
#else // !TARGET_SX1280_TX_ESP32_WROOM_SLIMMER
#define GPIO_PIN_DIO0      4
#define GPIO_PIN_DIO1      16
#define GPIO_PIN_RST       15
#define GPIO_PIN_BUSY      2
#define GPIO_PIN_TX_ENABLE 26
#define GPIO_PIN_RX_ENABLE 17
#endif // TARGET_SX1280_TX_ESP32_WROOM_SLIMMER
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


#elif defined(TARGET_SX1280_RX_STM32F1)
#define GPIO_PIN_NSS         PA4
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5

#define GPIO_PIN_DIO0        PB3
#define GPIO_PIN_DIO1        PB12
#define GPIO_PIN_DIO2        PB13
#define GPIO_PIN_RST         PB9
#define GPIO_PIN_BUSY        PB5

#define GPIO_PIN_RCSIGNAL_RX PA10 // USART1
#define GPIO_PIN_RCSIGNAL_TX PA9  // USART1

#define GPIO_PIN_LED         PB15

#define TARGET_INDENTIFIER   "2020_GNICE"


#elif defined(TARGET_SX1280_RX_NANO_v05)
#define GPIO_PIN_NSS         PA4
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5

#define GPIO_PIN_DIO0        PA10
#define GPIO_PIN_DIO1        PA9
#define GPIO_PIN_DIO2        PA8
#define GPIO_PIN_RST         PB4
#define GPIO_PIN_BUSY        PA11

#define GPIO_PIN_RCSIGNAL_RX PB7  // USART1, AFAIO
#define GPIO_PIN_RCSIGNAL_TX PB6  // USART1, AFAIO

#define GPIO_PIN_LED         PB5

#define TARGET_INDENTIFIER   "CCG_PP"


#elif defined(TARGET_SX1280_RX_NANO_DIV)
#define GPIO_PIN_NSS         PA3
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5

#define GPIO_PIN_DIO0        PC6
#define GPIO_PIN_DIO1        PA8
#define GPIO_PIN_DIO2        PB1
#define GPIO_PIN_RST         PA15
#define GPIO_PIN_BUSY        PA11

#define GPIO_PIN_RCSIGNAL_RX PB7  // USART1, AFAIO
#define GPIO_PIN_RCSIGNAL_TX PB6  // USART1, AFAIO

#define GPIO_PIN_LED         PB0

#define TARGET_INDENTIFIER   "CCG_DIV"


#elif defined(TARGET_SX1280_RX_PICO_G0)
#define GPIO_PIN_NSS         PA4
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5

#define GPIO_PIN_DIO0        PA8
#define GPIO_PIN_DIO1        PB1
#define GPIO_PIN_BUSY        PC6

#define GPIO_PIN_RCSIGNAL_RX PB7  // USART1, AFAIO
#define GPIO_PIN_RCSIGNAL_TX PB6  // USART1, AFAIO

#define GPIO_PIN_LED         PA2

#define TARGET_INDENTIFIER   "PICO_G0"


/******************************************************************************************/
/*                                     ESP RX CONFIGS                                     */
/******************************************************************************************/
#elif defined(TARGET_ESP8285_RX)
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


/******************************************************************************************/
/*                                   STM32 RX CONFIGS                                     */
/******************************************************************************************/
#elif defined(TARGET_RHF76_052)
/*
    Other pins:
    PA15        LOW to enable default bootloader
*/

#define GPIO_PIN_NSS         PA4
#define GPIO_PIN_DIO0        PB10
#define GPIO_PIN_DIO1        PB2
#define GPIO_PIN_DIO2        PB0  // not used at the moment
#define GPIO_PIN_DIO3        PB1  // not used at the moment
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5
#define GPIO_PIN_RST         PB11
/* PA1 and PA2 is used to selct HIGH or LOW RF band! */
/*
    LOW 0 : HIGH 0 = OFF
    LOW 1 : HIGH 0 = LOW BAND (433)
    LOW 0 : HIGH 1 = HIGH BAND (868 / 915)
*/
#define GPIO_SELECT_RFIO_HIGH PA2
#define GPIO_SELECT_RFIO_LOW  PA1
// USART1: TX=PA9, RX=PA10 (AF4) or TX=PB6, RX=PB7 (AF0)
#define GPIO_PIN_RCSIGNAL_RX  PB7  // USART1, PIN23
#define GPIO_PIN_RCSIGNAL_TX  PB6  // USART1, PIN22
#define GPIO_PIN_LED          PB4  // on board led (green), PIN16
// USART2: TX=PA2, RX=PA3 or TX=PA14, RX=PA15. Both AF4
//#define GPIO_PIN_DEBUG_RX    PA3 // USART2, PIN??
//#define GPIO_PIN_DEBUG_TX    PA2 // USART2, PIN??

#define TARGET_INDENTIFIER   "RHF76"


#elif defined(TARGET_RAK4200)

/* https://downloads.rakwireless.com/LoRa/RAK4200/Hardware-Specification/RAK4200_Schematic.pdf */

#define GPIO_PIN_NSS         PA4
#define GPIO_PIN_DIO0        PB0
#define GPIO_PIN_DIO1        PB1
#define GPIO_PIN_DIO2        PB5  // not used at the moment
#define GPIO_PIN_DIO3        PB4  // not used at the moment
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5
#define GPIO_PIN_RST         PA0
/** RF Switch config (https://www.njr.com/electronic_device/PDF/NJG1801K75_E.pdf)
 *
 * PA8 (VCTL2) and PA11 (VCTL1) is used to selct RF_IN or RF_OUT
 *   RF_IN  (PC-P1): VCTL1 LOW  , VCTL2 HIGH
 *   RF_OUT (PC-P2): VCTL1 HIGH , VCTL2 LOW
 *
 * Note: This is inverted, LOW = ENABLED!
 *
 * RAK VCTL1 = PA11
 * RAK VCTL2 = PA8
*/
#define GPIO_PIN_TX_ENABLE   PA11
#define GPIO_PIN_RX_ENABLE   PA8

// RAK4200 USART1: TX=PA9, RX=PA10
#define GPIO_PIN_RCSIGNAL_RX PA10   // USART1, PIN5
#define GPIO_PIN_RCSIGNAL_TX PA9    // USART1, PIN4
#define GPIO_PIN_LED         PA12   // PIN6 (UART1_DE)
// USART2: TX=PA2, RX=PA3
//#define GPIO_PIN_DEBUG_RX    PA3 // USART2, PIN1
//#define GPIO_PIN_DEBUG_TX    PA2 // USART2, PIN2

//#define DBG_PIN_TMR_ISR      PB6  // SCL
//#define DBG_PIN_RX_ISR       PB7  // SDA

#define TARGET_INDENTIFIER   "RAK4200"


/******************************************************************************************/
/*                                        R9 CONFIGS                                      */
/******************************************************************************************/

/*
Credit to Jacob Walser (jaxxzer) for the pinout!!!
https://github.com/jaxxzer
*/
#elif defined(TARGET_R9M_RX)

#define GPIO_PIN_NSS         PB12
#define GPIO_PIN_DIO0        PA15
//#define GPIO_PIN_DIO1        -1 //PA1 // NOT CORRECT!!! PIN STILL NEEDS TO BE FOUND BUT IS CURRENTLY UNUSED
#define GPIO_PIN_MOSI        PB15
#define GPIO_PIN_MISO        PB14
#define GPIO_PIN_SCK         PB13
#define GPIO_PIN_RST         PC14
#if defined(TARGET_R9SLIM_PLUS)
#  define GPIO_PIN_RCSIGNAL_RX PB11 // USART3
#  define GPIO_PIN_RCSIGNAL_TX PA9  // USART1
#elif defined(TARGET_R900MINI_RX)
// Inverted half dupplex
#  define GPIO_PIN_RCSIGNAL_INVERTED 1
#  define GPIO_PIN_RCSIGNAL_RX PA9
#  define GPIO_PIN_RCSIGNAL_TX PA9  // USART1
#else
#  define GPIO_PIN_RCSIGNAL_RX PA10 // USART1
#  define GPIO_PIN_RCSIGNAL_TX PA9  // USART1
#endif
#if defined(TARGET_R9MX)
#  define TARGET_INDENTIFIER   "R9MX"
#  define GPIO_PIN_LED_RED   PB2
#  define GPIO_PIN_LED_GREEN PB3
//#  define GPIO_PIN_BUTTON    PB0  // pullup e.g. LOW when pressed
#elif defined(TARGET_R9SLIM_PLUS)
#  define TARGET_INDENTIFIER   "R9SLIM+"
#  define GPIO_PIN_LED_RED   PA11
#  define GPIO_PIN_LED_GREEN PA12
//#  define GPIO_PIN_BUTTON    PC13 // pullup e.g. LOW when pressed
// ****** antenna switch pins *******
/* PB3: RX = HIGH, TX = LOW */
#  define GPIO_PIN_RX_ENABLE   PB3
/* PB9: antenna 1 (left) = HIGH, antenna 2 (right) = LOW
 * Note: Right Antenna is selected by default, LOW */
#  define GPIO_PIN_ANTENNA_SELECT PB9
#elif defined(TARGET_R900MINI_RX)
#  define TARGET_INDENTIFIER   "R900MINI"
#  define GPIO_PIN_LED_RED   PA11
#  define GPIO_PIN_LED_GREEN PA12
//#  define GPIO_PIN_BUTTON    PC13 // pullup e.g. LOW when pressed
#else
#  define TARGET_INDENTIFIER   "R9MM"
#  define GPIO_PIN_LED_RED   PC1
#  define GPIO_PIN_LED_GREEN PB3
//#  define GPIO_PIN_BUTTON    PC13 // pullup e.g. LOW when pressed
//#define GPIO_PIN_DEBUG_RX    PA3 // confirmed, USART2
//#define GPIO_PIN_DEBUG_TX    PA2 // confirmed, USART2
#endif

// External pads
// #define R9m_Ch1    PA8
// #define R9m_Ch2    PA11
// #define R9m_Ch3    PA9
// #define R9m_Ch4    PA10
// #define R9m_sbus   PA2
// #define R9m_sport  PA5
// #define R9m_isport PB11

#if SERVO_OUTPUTS_ENABLED
#define SERVO_PIN_CH1 PA8   // TIM1_CH1
#define SERVO_PIN_CH2 PA11  // TIM1_CH4
#define SERVO_PIN_CH3 PA9   // TIM1_CH2
#define SERVO_PIN_CH4 PA10  // TIM1_CH3
#endif

// Invert RED led (used to indicate failure)
#define GPIO_PIN_LED_RED_INV    1
#define GPIO_PIN_LED_GREEN_INV  0

#elif defined(TARGET_R9M_TX)

#define GPIO_PIN_NSS         PB12
#define GPIO_PIN_MOSI        PB15
#define GPIO_PIN_MISO        PB14
#define GPIO_PIN_SCK         PB13
#define GPIO_PIN_RST         PC14
#define GPIO_PIN_SDA         PB7
#define GPIO_PIN_SCL         PB6
#define GPIO_PIN_RCSIGNAL_RX PB11 // USART3 RX for S.Port
#define GPIO_PIN_RCSIGNAL_TX PB10 // USART3 TX for S.Port, needs BUFFER_OE

#if defined(R9M_LITE_TX)
#define GPIO_PIN_DIO0        PC15
#define GPIO_PIN_LED_RED     PA1 // Red LED // not yet confirmed
#define GPIO_PIN_LED_GREEN   PA4 // Green LED // not yet confirmed

#define GPIO_PIN_RX_ENABLE   PC13 // HIGH = RX, LOW = TX

#elif defined(R9M_lITE_PRO_TX)
#error "LITE PRO is not ok yet!"

#undef GPIO_PIN_RST
#define GPIO_PIN_RST         PA9  // NRESET
#define GPIO_PIN_DIO0        PA8  // confirmed
#define GPIO_PIN_LED_RED     PB3  // Red LED
#define GPIO_PIN_LED_GREEN   PA15 // Green LED
#define GPIO_PIN_LED_BLUE    PB4  // Blue LED

//#define GPIO_PIN_RFamp_APC1       PA4  //2.7V
//#define GPIO_PIN_RFamp_APC2       PA5  //100mW@590mV, 200mW@870mV, 500mW@1.093V, 1W@1.493V
#define GPIO_PIN_RFswitch_CONTROL PA6  // confirmed  //HIGH = RX, LOW = TX

#define GPIO_PIN_VRF1        PA7  // 26SU Sample RF1
#define GPIO_PIN_VRF2        PB1  // 26SU Sample RF2
#define GPIO_PIN_SWR         PA0  // SWR? ADC1_IN1

#define GPIO_PIN_RX_ENABLE   PC13 //PB3 // need to confirm

#else /* R9M_TX */

#define GPIO_PIN_DIO0        PA15
#define GPIO_PIN_LED_RED     PA11 // Red LED
#define GPIO_PIN_LED_GREEN   PA12 // Green LED
#define GPIO_PIN_BUTTON      PA8  // pullup e.g. LOW when pressed
#define GPIO_PIN_BUZZER      PB1  // confirmed
#if !TARGET_HM_ES915TX
#define GPIO_PIN_DIP1        PA12 // dip switch 1
#define GPIO_PIN_DIP2        PA11 // dip switch 2
#endif // !TARGET_HM_ES915TX
#define GPIO_PIN_RFamp_APC1       PA6 //CONFIRMED SANDRO// APC2 is connected through a I2C dac and is handled elsewhere
#define GPIO_PIN_RFswitch_CONTROL PB3 //CONFIRMED SANDRO HIGH = RX, LOW = TX
// PwrAmp, RFControl, dodgy measurement with SDR, descending
// high low  -5
// low  low  -30
// low  high -40
// high high -40

#if !TARGET_HM_ES915TX
// Serial1 is connected to internal ESP module if in use
#define CTRL_SERIAL Serial1
#define DEFINE_SERIAL1
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
#endif // !TARGET_HM_ES915TX

/* R9M DAC control address */
#define DAC_I2C_ADDRESS         0b0001100

//#define GPIO_PIN_DIO1 PA1  //Not Needed, HEARTBEAT pin
#endif /* R9M_LITE_TX */

//#define GPIO_PIN_DEBUG_RX PA3 // confirmed, USART2
//#define GPIO_PIN_DEBUG_TX PA2 // confirmed, USART2

#define BUFFER_OE     PA5  //CONFIRMED

#define GPIO_PIN_LED_RED_INV    0
#define GPIO_PIN_LED_GREEN_INV  0

#elif defined(TARGET_TX_DUAL_STM32F1)
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

// Both radios are included
#define RADIO_SX128x 1
#define RADIO_SX127x 1


#elif defined(TARGET_HANDSET_STM32F722)
// SPI pins
#define GPIO_PIN_MOSI      PA7
#define GPIO_PIN_MISO      PA6
#define GPIO_PIN_SCK       PA5

// Radio GPIOs (SX1280)
#define GPIO_PIN_NSS_128x  PA12
#define GPIO_PIN_DIO0_128x PD2
#define GPIO_PIN_DIO1_128x UNDEF_PIN
#define GPIO_PIN_RST_128x  PA15
#define GPIO_PIN_BUSY      PA14
#define GPIO_PIN_TXEN_128x PC10
#define GPIO_PIN_RXEN_128x PA13
// Radio GPIOs (SX1276)
#define GPIO_PIN_NSS_127x  PB1
#define GPIO_PIN_DIO0_127x PB0
#define GPIO_PIN_RST_127x  PB2

// V0.2
#define GPIO_PIN_LED_RGB   PA11 // WS2812 RGB

// ESPbackpack logger (USART1)
#define CTRL_SERIAL Serial1
//#define CTRL_SERIAL_BAUD    420000
//#define CTRL_SERIAL_BAUD    460800
#define CTRL_SERIAL_BAUD    921600
#define DEFINE_SERIAL1
#define SERIAL1_USE_DMA     1
#define PRINTF_NUM_BLOCKS   128
#define PRINTF_BUFF_SIZE    256

// Both radios are included
#define RADIO_SX128x 1
#define RADIO_SX127x 1

// Gimbal pins
#define GIMBAL_L1       PC1 // G_R_1
#define GIMBAL_L2       PC0 // G_R_2
#define GIMBAL_R1       PC3 // G_L_1
#define GIMBAL_R2       PC2 // G_L_2

#define SWITCH_1_1  PB3     // S1_1
#define SWITCH_1_2  PB4     // S1_2
#define SWITCH_2_1  PB5     // S2_1
//#define SWITCH_2_2  PB6     // S2_2
#define SWITCH_3_1  PB14    // S3_1
#define SWITCH_3_2  PB15    // S3_2
//#define SWITCH_4_1  PB12    // S4_1
//#define SWITCH_4_2  PB13    // S4_2
#define SWITCH_4_1  PB13    // S4_2
#define SWITCH_5_1  PC12    // S5_1
#define SWITCH_5_2  PC11    // S5_2
#define SWITCH_6_1  PB9     // S6_1
#define SWITCH_6_2  PB8     // S6_2

// ********** OUTPUT MAPPING **********
#define SWITCH_CH_1     SWITCH_3
#define SWITCH_CH_2     SWITCH_1
#define SWITCH_CH_3     SWITCH_2
#define SWITCH_CH_4     SWITCH_4
#define SWITCH_CH_5     SWITCH_5
#define SWITCH_CH_6     SWITCH_6

#elif defined(TARGET_NAMIMNORC_TX)
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
    #define RADIO_SX128x 1
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
    #define RADIO_SX127x 1
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
/* WS2812 led */
#define GPIO_PIN_LED_RGB        PB0

#elif defined(TARGET_NAMIMNORC_RX)
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

#elif TARGET_IMRC_GHOST_TX
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

#elif TARGET_RX_GHOST_ATTO_V1
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

#else
#error "Invalid target!"
#endif


/**********************************
           DEFAULTS
 **********************************/
#ifndef RADIO_SX128x
#define RADIO_SX128x 0
#endif
#if !defined(RADIO_SX127x)
#if !RADIO_SX128x
#define RADIO_SX127x 1
#else
#define RADIO_SX127x 0
#endif
#endif

#ifndef GPIO_PIN_RFswitch_CONTROL
#define GPIO_PIN_RFswitch_CONTROL UNDEF_PIN
#endif
#ifndef GPIO_PIN_RFamp_APC1
#define GPIO_PIN_RFamp_APC1 UNDEF_PIN
#endif
#ifndef GPIO_PIN_RFamp_APC2
#define GPIO_PIN_RFamp_APC2 UNDEF_PIN
#endif

#ifndef GPIO_PIN_OLED_SDA
#define GPIO_PIN_OLED_SDA UNDEF_PIN
#endif
#ifndef GPIO_PIN_OLED_SCK
#define GPIO_PIN_OLED_SCK UNDEF_PIN
#endif

#ifndef GPIO_PIN_RCSIGNAL_RX
#define GPIO_PIN_RCSIGNAL_RX UNDEF_PIN
#endif
#ifndef GPIO_PIN_RCSIGNAL_TX
#define GPIO_PIN_RCSIGNAL_TX UNDEF_PIN
#endif

#ifndef GPIO_PIN_RX_ENABLE
#define GPIO_PIN_RX_ENABLE  UNDEF_PIN
#endif
#ifndef GPIO_PIN_TX_ENABLE
#define GPIO_PIN_TX_ENABLE  UNDEF_PIN
#endif
#ifndef GPIO_PIN_PA_ENABLE
#define GPIO_PIN_PA_ENABLE  UNDEF_PIN
#endif
#ifndef BUFFER_OE
#define BUFFER_OE           UNDEF_PIN
#endif
#ifndef BUFFER_OE_INVERTED
#define BUFFER_OE_INVERTED  0
#endif
#ifndef RCSIGNAL_INVERTED
#define RCSIGNAL_INVERTED   0
#endif
#ifndef RCSIGNAL_USE_DMA
#define RCSIGNAL_USE_DMA    1
#endif
#ifndef GPIO_PIN_LED_RGB
#define GPIO_PIN_LED_RGB    UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_RED
#define GPIO_PIN_LED_RED    UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_RED_INV
#define GPIO_PIN_LED_RED_INV 0
#endif
#ifndef GPIO_PIN_LED_GREEN
#define GPIO_PIN_LED_GREEN  UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_GREEN_INV
#define GPIO_PIN_LED_GREEN_INV 0
#endif
#ifndef GPIO_PIN_LED
#define GPIO_PIN_LED        GPIO_PIN_LED_RED
#endif
#ifndef GPIO_PIN_BUTTON
#define GPIO_PIN_BUTTON     UNDEF_PIN
#endif
#ifndef GPIO_PIN_BUZZER
#define GPIO_PIN_BUZZER     UNDEF_PIN
#endif

#ifndef GPIO_PIN_RST
#define GPIO_PIN_RST UNDEF_PIN
#endif
#if !defined(GPIO_PIN_DIO0) && !(defined(GPIO_PIN_DIO0_127x) || defined(GPIO_PIN_DIO0_128x))
#error "DIO0 is mandatory!"
#endif
#ifndef GPIO_PIN_DIO1
#define GPIO_PIN_DIO1 UNDEF_PIN
#endif
#ifndef GPIO_PIN_DIO2
#define GPIO_PIN_DIO2 UNDEF_PIN
#endif
#ifndef GPIO_PIN_DIO3
#define GPIO_PIN_DIO3 UNDEF_PIN
#endif
#ifndef GPIO_PIN_BUSY
#if RADIO_SX128x
#error "BUSY pin is mandatory witth SX1280!"
#endif // RADIO_SX128x
#define GPIO_PIN_BUSY UNDEF_PIN
#endif
#ifndef GPIO_PIN_ANTENNA_SELECT
#define GPIO_PIN_ANTENNA_SELECT UNDEF_PIN
#endif

#ifndef GPIO_PIN_DEBUG_RX
#define GPIO_PIN_DEBUG_RX UNDEF_PIN
#endif
#ifndef GPIO_PIN_DEBUG_TX
#define GPIO_PIN_DEBUG_TX UNDEF_PIN
#endif

#ifndef DBG_PIN_TMR_ISR
#define DBG_PIN_TMR_ISR UNDEF_PIN
#endif
#ifndef DBG_PIN_RX_ISR
#define DBG_PIN_RX_ISR  UNDEF_PIN
#endif

#ifndef SERVO_PIN_CH1
#define SERVO_PIN_CH1 UNDEF_PIN
#endif
#ifndef SERVO_PIN_CH2
#define SERVO_PIN_CH2 UNDEF_PIN
#endif
#ifndef SERVO_PIN_CH3
#define SERVO_PIN_CH3 UNDEF_PIN
#endif
#ifndef SERVO_PIN_CH4
#define SERVO_PIN_CH4 UNDEF_PIN
#endif


#ifndef GPIO_PIN_NSS_127x
#define GPIO_PIN_NSS_127x GPIO_PIN_NSS
#endif
#ifndef GPIO_PIN_RST_127x
#define GPIO_PIN_RST_127x GPIO_PIN_RST
#endif
#ifndef GPIO_PIN_DIO0_127x
#define GPIO_PIN_DIO0_127x GPIO_PIN_DIO0
#endif
#ifndef GPIO_PIN_DIO1_127x
#define GPIO_PIN_DIO1_127x GPIO_PIN_DIO1
#endif
#ifndef GPIO_PIN_DIO2_127x
#define GPIO_PIN_DIO2_127x GPIO_PIN_DIO2
#endif
#ifndef GPIO_PIN_TXEN_127x
#define GPIO_PIN_TXEN_127x GPIO_PIN_TX_ENABLE
#endif
#ifndef GPIO_PIN_RXEN_127x
#define GPIO_PIN_RXEN_127x GPIO_PIN_RX_ENABLE
#endif
#ifndef GPIO_PIN_PAEN_127x
#define GPIO_PIN_PAEN_127x GPIO_PIN_PA_ENABLE
#endif

#ifndef GPIO_PIN_NSS_128x
#define GPIO_PIN_NSS_128x GPIO_PIN_NSS
#endif
#ifndef GPIO_PIN_RST_128x
#define GPIO_PIN_RST_128x GPIO_PIN_RST
#endif
#ifndef GPIO_PIN_DIO0_128x
#define GPIO_PIN_DIO0_128x GPIO_PIN_DIO0
#endif
#ifndef GPIO_PIN_DIO1_128x
#define GPIO_PIN_DIO1_128x GPIO_PIN_DIO1
#endif
#ifndef GPIO_PIN_DIO2_128x
#define GPIO_PIN_DIO2_128x GPIO_PIN_DIO2
#endif
#ifndef GPIO_PIN_TXEN_128x
#define GPIO_PIN_TXEN_128x GPIO_PIN_TX_ENABLE
#endif
#ifndef GPIO_PIN_RXEN_128x
#define GPIO_PIN_RXEN_128x GPIO_PIN_RX_ENABLE
#endif
#ifndef GPIO_PIN_PAEN_128x
#define GPIO_PIN_PAEN_128x GPIO_PIN_PA_ENABLE
#endif

#ifndef GPIO_PIN_FAN_CTRL
#define GPIO_PIN_FAN_CTRL UNDEF_PIN
#endif
#ifndef GPIO_PIN_RF_AMP_DET
#define GPIO_PIN_RF_AMP_DET UNDEF_PIN
#endif
#ifndef GPIO_PIN_ANT_CTRL_1
#define GPIO_PIN_ANT_CTRL_1 UNDEF_PIN
#endif
#ifndef GPIO_PIN_ANT_CTRL_2
#define GPIO_PIN_ANT_CTRL_2 UNDEF_PIN
#endif
