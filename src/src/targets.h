#pragma once

#include "Arduino.h"

#define EMPTY()
#define UNDEF_PIN (-1)

/// General Features ///
#define LED_MAX_BRIGHTNESS 50 //0..255 for max led brightness

extern const char target_name[];
extern const uint8_t target_name_len;

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
#ifndef SERVO_PIN_CH5
#define SERVO_PIN_CH5 UNDEF_PIN
#endif
#ifndef SERVO_PIN_CH6
#define SERVO_PIN_CH6 UNDEF_PIN
#endif
#ifndef SERVO_PIN_CH7
#define SERVO_PIN_CH7 UNDEF_PIN
#endif
#ifndef SERVO_PIN_CH8
#define SERVO_PIN_CH8 UNDEF_PIN
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

#ifndef RECEIVER_TRANSMIT_POWER
#define RECEIVER_TRANSMIT_POWER 0b1111 // default RX to max power for tlm
#endif
