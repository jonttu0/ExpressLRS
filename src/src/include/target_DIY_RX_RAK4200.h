#pragma once

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
