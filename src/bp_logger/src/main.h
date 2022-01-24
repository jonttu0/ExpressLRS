#pragma once

#include <Arduino.h>

#include <FS.h>
#ifdef USE_LITTLE_FS
#include <LittleFS.h>
#define FILESYSTEM LittleFS
#else
#define FILESYSTEM SPIFFS
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 460800
#endif

void websocket_send(String & data, int num = -1);
void websocket_send(char const * data, int num = -1);

void MspVtxWrite(const char * input, int num = -1);
void MspVtxWriteToElrs(uint16_t freq);
