#pragma once

#include "msp_handler_base.h"

class HDZeroMsp : public MspHandlerBase
{
public:
    HDZeroMsp(CtrlSerial * serial) : MspHandlerBase(serial)
    {
    }
    ~HDZeroMsp()
    {
    }

    void init(void);

    void syncSettings(void);
    void syncSettings(AsyncWebSocketClient * const client);
    void syncSettings(AsyncEventSourceClient * const client);

    int parseSerialData(uint8_t chr);

    void loop(void);

private:
    enum {
        STATE_GET_FW_VER,
        STATE_GET_CH_INDEX,
        STATE_GET_FREQ,
        STATE_GET_RECORDING,
        STATE_READY,
    };

    uint32_t init_called_ms;
    uint8_t init_state;

    // From WEB UI
    int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(mspPacket_t & msp_in);

    void sendMspToHdzero(uint8_t const * const buff, uint16_t const len, uint16_t const function);

    void handleUserTextCommand(const char * input, size_t len);
    void handleVtxFrequencyCommand(uint16_t freq, AsyncWebSocketClient * const client);
    void handleRecordingStateCommand(uint8_t start);

    //
    void getFwVersion(void);
    void getChannelIndex(void);
    void getFrequency(void);
    void getRecordingState(void);

    uint16_t getFreqByIndex(uint8_t index)
    {
        if (index == (3 * 8 + 1))
            return 5760; // F2
        if (index == (3 * 8 + 3))
            return 5580; // F4
        index -= 4 * 8;
        if (index <= 7)
            return 5658U + (37U * index);
        return 0;
    }

    uint8_t getIndexByFreq(uint16_t const freq)
    {
        // MAP freq to HDZ supported channel indeces
        switch (freq) {
            case 5760: // F2
                return 3 * 8 + 1;
            case 5800: // F4
                return 3 * 8 + 3;
            case 5658: // R1
                return 4 * 8 + 0;
            case 5695: // R2
                return 4 * 8 + 1;
            case 5732: // R3
                return 4 * 8 + 2;
            case 5769: // R4
                return 4 * 8 + 3;
            case 5806: // R5
                return 4 * 8 + 4;
            case 5843: // R6
                return 4 * 8 + 5;
            case 5880: // R7
                return 4 * 8 + 6;
            case 5917: // R8
                return 4 * 8 + 7;
            default:
                break;
        }
        return 0;
    }
};
