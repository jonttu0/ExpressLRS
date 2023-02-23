#pragma once

#include "platform.h"
#include "msp.h"
#include "main.h"
#include "comm_espnow.h"
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
    virtual int parse_data(uint8_t chr)
    {
        return -1;
    }
    // Message inputs
    virtual int parse_command(char const * cmd, size_t len, AsyncWebSocketClient * const client)
    {
        return -1;
    }
    virtual int parse_command(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client)
    {
        return -1;
    }
    // Handle received MSP packet
    virtual int handle_received_msp(mspPacket_t & msp_in)
    {
        return -1;
    }

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

    void sendMspVtxSetToEspnow(uint16_t const freq, int8_t const power = -1, bool const pitmode = false)
    {
        // Send to other esp-now clients
        msp_out.reset();
        msp_out.type = MSP_PACKET_V2_COMMAND;
        msp_out.flags = 0;
        msp_out.function = MSP_VTX_SET_CONFIG;
        msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
        msp_out.payload[0] = (freq & 0xff);
        msp_out.payload[1] = (freq >> 8);
        if (0 <= power) {
            msp_out.payloadSize += 1;
            msp_out.payload[2] = power;
        }
        if (pitmode) {
            msp_out.payloadSize += 1;
            msp_out.payload[3] = pitmode;
        }
        espnow_send_msp(msp_out);
    }

    uint16_t checkInputMspVtxSet(mspPacket_t & msp_in)
    {
        if (msp_in.type == MSP_PACKET_V2_COMMAND) {
            if (msp_in.function == MSP_VTX_SET_CONFIG && 2 <= msp_in.payloadSize) {
                return parseFreq(msp_in.payload);
            }
        }
        return 0;
    }

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

private:
};
