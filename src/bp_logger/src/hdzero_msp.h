#pragma once

#include "platform.h"
#include "msp.h"
#include "main.h"
#include <stdint.h>


class AsyncWebSocketClient;


class HDZeroMsp
{
public:
    HDZeroMsp(CtrlSerial *serial) : _serial(serial) {}
    ~HDZeroMsp() {}

    void init(void);

    void syncSettings(AsyncWebSocketClient * client);
    String syncSettingsJson(void) {return String();};
    void sendSettingsJson(void * client) {};

    int parse_data(uint8_t const chr);
    int parse_command(char * cmd, size_t len, void * client);
    int parse_command(websoc_bin_hdr_t const * const cmd, size_t len, void * client);

    int handle_received_msp(mspPacket_t &msp_in);

    void loop(void);

private:
    enum {
        STATE_GET_CH_INDEX,
        STATE_GET_FREQ,
        STATE_GET_RECORDING,
        STATE_READY,
    };

    CtrlSerial * _serial;

    MSP _handler;
    mspPacket_t msp_out;
    uint32_t init_called_ms;
    uint8_t init_state;

    void MspWrite(uint8_t const * const buff, uint16_t const len, uint16_t const function);

    void handleUserText(const char * input, size_t len);
    void handleVtxFrequency(uint16_t freq, void * client = NULL, bool espnow = true);
    void handleRecordingState(uint8_t start);

    void sendVtxFrequencyToWebsocket(uint16_t freq, AsyncWebSocketClient * client = NULL);
    void sendVRecordingStateToWebsocket(uint8_t state, AsyncWebSocketClient * client = NULL);

    //
    void getChannelIndex(void);
    void getFrequency(void);
    void getRecordingState(void);
};
