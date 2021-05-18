#ifndef DEBUG_H_
#define DEBUG_H_

#include <Arduino.h>
#include <HardwareSerial.h>
#include "HwSerial.h"
#include "printf.h"
#include "targets.h"

#ifndef DEBUG_SERIAL

#if defined(PLATFORM_ESP32)
#if WIFI_LOGGER && !WIFI_UPDATER
#define DEBUG_SERIAL wifi_logger_serial
#else
#define DEBUG_SERIAL Serial
#endif
#elif defined(R9M_LITE_TX) || defined(R9M_lITE_PRO_TX)
#ifdef DEFINE_SERIAL1
//#define DEBUG_SERIAL Serial1
#endif
#elif defined(TARGET_R9M_TX) || defined(TARGET_TX_DUAL_STM32F1) || defined(TARGET_NAMIMNORC_TX)
// FIXME: Change flagging!
#ifdef DEFINE_SERIAL1
#define DEBUG_SERIAL Serial1
#endif
#elif defined(TARGET_HANDSET_STM32F722)
#define DEBUG_SERIAL CTRL_SERIAL
#elif CRC16_POLY_TESTING || RX_DEBUG_ENABLED
#define DEBUG_SERIAL CrsfSerial
#warning "CRSF serial debug enabled! No real connection to FC!"
#else
//#define DEBUG_SERIAL CrsfSerial
#endif

#endif // ifndef DEBUG_SERIAL

#ifdef DEBUG_SERIAL
// Debug enabled
#if ARDUINO
#if PLATFORM_ESP8266 || PLATFORM_ESP32
/* ESP Arduino platforms have prinf included */
#define DEBUG_PRINTF(...)  DEBUG_SERIAL.printf(__VA_ARGS__)
#else
/* Use custom for others */
#define DEBUG_PRINTF(...)  Printf::printf_(__VA_ARGS__)
#endif
#else // !ARDUINO
/* Native "bare metal" implementation does not support print/println methods! */
#define DEBUG_PRINTF(...)  Printf::printf_(__VA_ARGS__)
#endif // ARDUINO
#define NO_DATA_TO_FC 1

#else // !DEBUG_SERIAL
// Debug disabled
#define DEBUG_PRINTF(...)
#endif // DEBUG_SERIAL

#endif // DEBUG_H_
