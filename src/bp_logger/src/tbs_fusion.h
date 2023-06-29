#pragma once

#include "msp_handler_base.h"

class TbsFusion : public MspHandlerBase
{
public:
    TbsFusion(CtrlSerial * serial) : MspHandlerBase(serial)
    {
        fetch_dev_info = false;
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
        STATE_READY,
        STATE_CLEAR_OSD,
    };
    uint8_t current_state;
    uint32_t osd_timeout;

    bool fetch_dev_info;

    // From WEB UI
    int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client);
    int parseCommandPriv(mspPacket_t & msp_in);

    void sendMspToUart(uint8_t const * const buff, uint16_t const len, uint16_t const function);
    void CrsfWrite(uint8_t * const buff, size_t size) const;
    void CrsfWriteCommand(uint8_t * const buff, size_t size) const;

    void handleUserTextCommand(const char * input, size_t len, AsyncWebSocketClient * const client);
    void handleUserTextCommand(const char * text1, const char * text2, const char * text3, uint8_t pos, uint8_t lap);
    void handleVtxFrequencyCommand(uint16_t freq, AsyncWebSocketClient * const client);
    void handleLaptimerState(uint16_t const race_id,
                             uint16_t const round_num,
                             bool const state,
                             AsyncWebSocketClient * const client = NULL);
    void handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL);
    void handleTelemetryGps(uint8_t const * const payload);
    void handleTelemetryLinkStats(uint8_t const * const payload);
    void handleTelemetryBattery(uint8_t const * const payload);
    void pingSend(void) const;

    void param_read_send(uint8_t field_id, uint8_t junk) const;
    bool param_entry_send(uint8_t param_index, uint8_t dest_addr, const char * info_msg = NULL);
};
