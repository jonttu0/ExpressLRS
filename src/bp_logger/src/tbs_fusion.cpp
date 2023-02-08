#include "tbs_fusion.h"
#include "CRSF.h"

#define RETRY_INTERVAL_MS 1000

typedef struct {
    crsf_ext_header_t header;
    uint8_t payload[6];
    uint8_t crc;
} crsf_freq_set_t;

static CRSF crsf(NULL);

void TbsFusion::init(void)
{
    _handler.markPacketFree();
}

void TbsFusion::syncSettings(void)
{
}

void TbsFusion::syncSettings(AsyncWebSocketClient * const client)
{
    // Send settings
    sendVtxFrequencyToWebsocket(eeprom_storage.vtx_freq);
}

void TbsFusion::syncSettings(AsyncEventSourceClient * const client)
{
    String json = "{\"vtxfreq\":";
    json += eeprom_storage.vtx_freq;
    json += '}';
    async_event_send(json, "vtxfreq", client);
    async_event_send(m_version_info, "vrx_version", client);
}

// Data from Serial input
int TbsFusion::parseSerialData(uint8_t const chr)
{
    String info = "";
    if (_handler.processReceivedByte(chr)) {
        /* Handle the received MSP message */
        mspPacket_t & msp_in = _handler.getPacket();
        if (msp_in.type != MSP_PACKET_V2_COMMAND || msp_in.type != MSP_PACKET_V2_RESPONSE) {
            info = "Invalid MSP received! func: ";
            info += msp_in.function;
        } else {
            // TODO: handle input MSP messages!
        }

        if (info.length()) {
            websocket_send_txt(info);
        } else {
            espnow_send_msp(msp_in);
        }
        _handler.markPacketFree();

    } else if (!_handler.mspOngoing()) {
        crsf_ext_header_t const * const p_message = (crsf_ext_header_t *)crsf.ParseInByte(chr);
        if (p_message) {
            // Check CRSF input packet
            info = "CRSF packet: 0x" + String(p_message->type, HEX);
            switch (p_message->type) {
                default:
                    break;
            }
            websocket_send_txt(info);
        }
        return crsf.IsFrameActive() ? 0 : -1;
    }
    return 0;
}

// From WEB UI
int TbsFusion::parseCommand(char const * cmd, size_t len, AsyncWebSocketClient * const client)
{
    // ExLRS setting commands
    const char * temp = strstr(cmd, "SET_text=");
    if (temp) {
        handleUserTextCommand(&temp[9], (len - 9));
        return 0;
    }
    return -1;
}

// From WEB UI
int TbsFusion::parseCommand(websoc_bin_hdr_t const * const cmd, size_t len, AsyncWebSocketClient * const client)
{
    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send_txt(settings_out, client);
        return -1;
    }

    switch (cmd->msg_id) {
        case WSMSGID_VIDEO_FREQ: {
            uint16_t const freq = parseFreq(cmd->payload);
            handleVtxFrequencyCommand(freq, client);
            break;
        }
        case WSMSGID_RECORDING_CTRL: {
            break;
        }
        default:
            return -1;
    }
    return 0;
}

// This is received from outside (ESP-NOW). Return -1 to get packet written to Serial
int TbsFusion::parseCommand(mspPacket_t & msp_in)
{
    uint16_t const freq = checkInputMspVtxSet(msp_in);
    if (freq) {
        /* Pass command to HDZero */
        handleVtxFrequencyCommand(freq, NULL, false);
        /* Infrom web clients */
        sendVtxFrequencyToWebsocket(freq);
    }
    /* Return 0 to ignore packet */
    return 0;
}

void TbsFusion::loop(void)
{
}

void TbsFusion::sendMspToUart(uint8_t const * const buff, uint16_t const len, uint16_t const function)
{
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = function;
    msp_out.payloadSize = (!buff) ? 0 : len;
    if (buff && len)
        memcpy((void *)msp_out.payload, buff, len);
    // Send packet to HDZero
    MSP::sendPacket(&msp_out, _serial);
}

void TbsFusion::CrsfWrite(uint8_t * buff, uint8_t const size) const
{
    buff[size - 1] = crsf.CalcCRC(&buff[2], (buff[1] - 1));
    _serial->write(buff, size);
}

void TbsFusion::handleUserTextCommand(const char * input, size_t const len)
{
    if (input == NULL)
        return;

    // TODO: write text to OSD

    String dbg_info = "OSD Text: '";
    dbg_info += input;
    dbg_info += "'";
    websocket_send_txt(dbg_info);
}

void TbsFusion::handleVtxFrequencyCommand(uint16_t const freq, AsyncWebSocketClient * const client, bool const espnow)
{
    if (storeVtxFreq(client, freq) == 0) {
        return;
    }

    crsf_freq_set_t command;
    command.header.device_addr = CRSF_SYNC_BYTE;
    command.header.frame_size = sizeof(crsf_freq_set_t) - CRSF_FRAME_START_BYTES;
    command.header.type = CRSF_FRAMETYPE_SET_FREQ;
    command.header.dest_addr = 0;
    command.header.orig_addr = 0;
    command.payload[0] = 0x10;
    command.payload[1] = 0xEC;
    command.payload[2] = 0x0E;
    command.payload[3] = freq >> 8;   // freq byte 1
    command.payload[4] = freq & 0xFF; // freq byte 2
    command.payload[5] = 0x01;

    CrsfWrite((uint8_t *)&command, sizeof(command));

    // Send to other esp-now clients
    if (espnow) {
        // msp_out.function = MSP_VTX_SET_CONFIG;
        // espnow_send_msp(msp_out);
    }
}

void TbsFusion::sendVtxFrequencyToWebsocket(uint16_t const freq)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send_bin(response, sizeof(response));
}
