#pragma once

#include <Arduino.h>
#include "msp.h"

#ifdef ESP_NOW
#ifndef ESP_NOW_PEERS
#undef ESP_NOW
#endif // ESP_NOW_PEERS
#endif // ESP_NOW

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


void espnow_init(uint32_t channel);
String & espnow_get_info();
void espnow_send_msp(mspPacket_t &msp);
