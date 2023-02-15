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

class AsyncWebSocketClient;

String mac_addr_print(uint8_t const * const mac_addr);

void websocket_send(String & data, AsyncWebSocketClient * client = NULL);
void websocket_send(char const * data, AsyncWebSocketClient * client = NULL);
void websocket_send(uint8_t const * data, uint8_t len, AsyncWebSocketClient * client = NULL);
void websocket_send_bin(uint8_t const * data, uint8_t len, AsyncWebSocketClient * client = NULL);

typedef struct {
    uint16_t msg_id;
    uint8_t  payload[];
} websoc_bin_hdr_t;

enum {
    WSMSGID_ERROR_IND        = 0xCAFE,
    WSMSGID_BASE_ESPNOW      = 0x1100,
    WSMSGID_BASE_STM32       = 0x1200,
    WSMSGID_BASE_ELRS        = 0x2200,
    WSMSGID_BASE_TELEMETRY   = 0x2280,
    WSMSGID_BASE_HANDSET     = 0x2300,
    WSMSGID_BASE_HANDSET_TLM = 0x2380,
    WSMSGID_BASE_MSP         = 0x2400,
};

/* Generic message identifiers */
enum {
    // MSP messages
    WSMSGID_VIDEO_FREQ = WSMSGID_BASE_MSP,
    WSMSGID_RECORDING_CTRL,
};
