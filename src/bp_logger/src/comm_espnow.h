#pragma once

#include <Arduino.h>
#include "msp.h"


typedef int (*esp_now_msp_rcvd_cb_t)(mspPacket_t &msp_pkt);


void espnow_init(uint32_t channel, esp_now_msp_rcvd_cb_t cb);
uint8_t espnow_channel(void);
void espnow_update_clients(uint8_t const * const data, uint8_t len, int wsnum = -1);
String & espnow_get_info(void);
void espnow_send_msp(mspPacket_t &msp);
