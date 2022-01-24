#pragma once

/*
Credit to Jacob Walser (jaxxzer) for the pinout!!!
https://github.com/jaxxzer
*/

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
