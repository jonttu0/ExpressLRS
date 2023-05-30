#pragma once

#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP8266
#include <FS.h>
#endif
#if USE_LITTLE_FS
#include <LittleFS.h>
#define FILESYSTEM LittleFS // TODO: make LittleFS to default
#else
#define FILESYSTEM SPIFFS
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 460800
#endif
#if UART_DEBUG_EN
#undef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

extern const char version_string[];

class AsyncWebSocketClient;
class AsyncEventSourceClient;

String mac_addr_print(uint8_t const * const mac_addr);

void websocket_send_txt(char const * data, AsyncWebSocketClient * const client = NULL);
void websocket_send_txt(String & data, AsyncWebSocketClient * const client = NULL);
void websocket_send_bin(uint8_t const * data, uint8_t len, AsyncWebSocketClient * const client = NULL);

void async_event_send(String & data, const char * event, AsyncEventSourceClient * const client = NULL);

void laptimer_start_stop(bool const start);

typedef struct {
    uint16_t msg_id;
    uint8_t payload[];
} websoc_bin_hdr_t;

enum {
    WSMSGID_ERROR_IND = 0xCAFE,
    WSMSGID_BASE_ESPNOW = 0x1100,
    WSMSGID_BASE_STM32 = 0x1200,
    WSMSGID_BASE_ELRS = 0x2200,
    WSMSGID_BASE_HANDSET = 0x2300,
    WSMSGID_BASE_HANDSET_TLM = 0x2380,
    WSMSGID_BASE_MSP = 0x2400,
    WSMSGID_BASE_LAPTIMER = 0x2500,
};

/* Generic message identifiers */
enum {
    // ESP-NOW configuration
    WSMSGID_ESPNOW_ADDRS = WSMSGID_BASE_ESPNOW,

    // STM32 control messages
    WSMSGID_STM32_RESET = WSMSGID_BASE_STM32,

    // MSP messages
    WSMSGID_VIDEO_FREQ = WSMSGID_BASE_MSP,
    WSMSGID_RECORDING_CTRL,
};

/* LapTimer message identifiers */
enum {
    WSMSGID_LAPTIMER_START_STOP = WSMSGID_BASE_LAPTIMER,
    WSMSGID_LAPTIMER_LAPTIME,
};
