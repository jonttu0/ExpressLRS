#pragma once

/*
 * FrSky R9M Lite
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

#define GPIO_PIN_DIO0        PC15
#define GPIO_PIN_LED_RED     PA1 // Red LED // not yet confirmed
#define GPIO_PIN_LED_GREEN   PA4 // Green LED // not yet confirmed

#define GPIO_PIN_RX_ENABLE   PC13 // HIGH = RX, LOW = TX

//#define GPIO_PIN_DEBUG_RX PA3 // confirmed, USART2
//#define GPIO_PIN_DEBUG_TX PA2 // confirmed, USART2

#define BUFFER_OE     PA5  //CONFIRMED

#define GPIO_PIN_LED_RED_INV    0
#define GPIO_PIN_LED_GREEN_INV  0
