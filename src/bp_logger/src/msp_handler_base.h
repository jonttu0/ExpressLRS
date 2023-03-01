#pragma once

#include "platform.h"
#include "msp.h"
#include "main.h"
#include "comm_espnow.h"
#include "storage.h"
#include <stdint.h>

class MspHandlerBase
{
public:
    MspHandlerBase(CtrlSerial * serial) : _serial(serial)
    {
        _handler.markPacketFree();
        m_version_info = "";
    }
    ~MspHandlerBase()
    {
    }

    virtual void init(void)
    {
    }
    virtual void loop(void)
    {
    }

    // Send settings to client (websocket or event)
    virtual void syncSettings(void)
    {
    }
    virtual void syncSettings(AsyncWebSocketClient * const client)
    {
    }
    virtual void syncSettings(AsyncEventSourceClient * const client)
    {
    }

    // Byte stream input to be checked for incoming MSP
    virtual int parseSerialData(uint8_t chr)
    {
        return -1;
    }
    // Message inputs from Web UI
    int parseCommand(char const * cmd, size_t const len, AsyncWebSocketClient * const client)
    {
        return parseCommandPriv(cmd, len, client);
    }
    int parseCommand(websoc_bin_hdr_t const * const cmd, size_t const len, AsyncWebSocketClient * const client);
    int parseCommand(mspPacket_t & msp_in); // Handle received MSP packet

    void clientSendVtxFrequency(uint16_t const freq, AsyncWebSocketClient * const client = NULL);
    void clientSendVRecordingState(uint8_t const state, AsyncWebSocketClient * const client = NULL);

    // Laptimer commands parsing for Web UI
    void clientSendLaptimerState(uint16_t const race_id, bool const state, AsyncWebSocketClient * const client = NULL);
    void clientSendLaptimerStateStart(uint16_t const race_id, AsyncWebSocketClient * const client = NULL)
    {
        clientSendLaptimerState(race_id, true, client);
    }
    void clientSendLaptimerStateStop(uint16_t const race_id, AsyncWebSocketClient * const client = NULL)
    {
        clientSendLaptimerState(race_id, false, client);
    }
    void clientSendLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client = NULL);

protected:
    CtrlSerial * _serial;

    MSP _handler;
    mspPacket_t msp_out;

    String m_version_info;

#define VTX_BAND_MAX    6
#define VTX_CHANNEL_MAX 8
    const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX] = {
        {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // A
        {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // B
        {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // E
        {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // F / Airwave
        {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // R / Immersion Raceband
        {5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613}, // L
    };

    virtual int parseCommandPriv(char const * cmd, size_t len, AsyncWebSocketClient * const client) = 0;
    virtual int
    parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client) = 0;
    virtual int parseCommandPriv(mspPacket_t & msp_in) = 0;

    virtual void handleVtxFrequencyCommand(uint16_t const freq, AsyncWebSocketClient * const client) = 0;

    uint16_t parseFreq(uint8_t const * const payload)
    {
        uint16_t const freq = ((uint16_t)payload[1] << 8) + payload[0];
        if (freq < (VTX_BAND_MAX * VTX_CHANNEL_MAX)) {
            uint8_t const _band = freq / VTX_CHANNEL_MAX;
            uint8_t const _index = freq % VTX_CHANNEL_MAX;
            return frequency_table[_band][_index];
        }
        return freq;
    }

    bool storeVtxFreq(AsyncWebSocketClient * const client, uint16_t const freq)
    {
        String dbg_info = "Invalid VTX freq received!";
        if (freq == 0) {
            websocket_send_txt(dbg_info, client);
            return false;
        }
        dbg_info = "Setting vtx freq to: ";
        dbg_info += freq;
        dbg_info += "MHz";
        websocket_send_txt(dbg_info, client);

        if (eeprom_storage.vtx_freq != freq) {
            eeprom_storage.vtx_freq = freq;
            eeprom_storage.markDirty();
        }
        return true;
    }

private:
};
