#pragma once

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
