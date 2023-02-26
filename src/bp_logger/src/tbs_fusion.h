#pragma once

#include "msp_handler_base.h"

class TbsFusion : public MspHandlerBase
{
public:
    TbsFusion(CtrlSerial * serial) : MspHandlerBase(serial)
    {
    }
    ~TbsFusion()
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
        STATE_GET_CH_INDEX,
        STATE_GET_FREQ,
        STATE_GET_RECORDING,
        STATE_READY,
    };

    CtrlSerial * _serial;

    MSP _handler;
    mspPacket_t msp_out;

    // From WEB UI
    int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(mspPacket_t & msp_in);

    void sendMspToUart(uint8_t const * const buff, uint16_t const len, uint16_t const function);
    void CrsfWrite(uint8_t * buff, uint8_t const size) const;

    void handleUserTextCommand(const char * input, size_t len);
    void handleVtxFrequencyCommand(uint16_t freq, AsyncWebSocketClient * const client);
};
