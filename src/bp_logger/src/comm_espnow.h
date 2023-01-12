#pragma once

#include <Arduino.h>
#include "msp.h"

#if !ESP_NOW
#define WIFI_CHANNEL 0 // Not defined
#if defined(ESP_NOW_CHANNEL)
#undef ESP_NOW_CHANNEL
#endif
#define ESP_NOW_CHANNEL 1

#else // ESP_NOW
#define WIFI_CHANNEL 2
#ifndef ESP_NOW_CHANNEL
#define ESP_NOW_CHANNEL 1
#endif
#if (ESP_NOW_CHANNEL == WIFI_CHANNEL)
#error "WiFi Channel config error! ESPNOW and WiFi must be on different channels"
#endif
#endif // ESP_NOW


typedef int (*esp_now_msp_rcvd_cb_t)(mspPacket_t &msp_pkt);


void espnow_init(uint32_t channel, esp_now_msp_rcvd_cb_t cb, CtrlSerial *serial_ptr);
void espnow_update_clients(uint8_t const * const data, uint8_t len, int wsnum = -1);
String & espnow_get_info();
void espnow_send_msp(mspPacket_t &msp);
