#include "hdzero_msp.h"

#define RETRY_INTERVAL_MS 1000

/*
 *  Get band/channel index  0x0300
 *  Set band/channel index  0x0301
 *  Get frequency           0x0302
 *  Set frequency           0x0303
 *  Get recording state     0x0304
 *  Set recording state     0x0305
 *  Get VRx mode            0x0306
 *  Set VRx mode            0x0307
 *  Get RSSI                0x0308
 *  Get battery voltage     0x0309
 *  Get firmware            0x030A
 *  Set buzzer              0x030B
 *  Set OSD Element         0x00B6
 */
#define HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET 0x0300
#define HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET 0x0301
#define HDZ_MSP_FUNC_FREQUENCY_GET          0x0302
#define HDZ_MSP_FUNC_FREQUENCY_SET          0x0303
#define HDZ_MSP_FUNC_RECORDING_STATE_GET    0x0304
#define HDZ_MSP_FUNC_RECORDING_STATE_SET    0x0305
#define HDZ_MSP_FUNC_VRX_MODE_GET           0x0306
#define HDZ_MSP_FUNC_VRX_MODE_SET           0x0307
#define HDZ_MSP_FUNC_RSSI_GET               0x0308
#define HDZ_MSP_FUNC_BATTERY_VOLTAGE_GET    0x0309
#define HDZ_MSP_FUNC_FIRMWARE_GET           0x030A
#define HDZ_MSP_FUNC_BUZZER_SET             0x030B
// #define MSP_ELRS_BACKPACK_SET_OSD_ELEMENT       0x030C
#define HDZ_MSP_FUNC_OSD_ELEMENT_SET 0x00B6

void HDZeroMsp::init(void)
{
    // init_called_ms = millis() + 2000; // Delay first query
    init_state = STATE_GET_FW_VER;
}

void HDZeroMsp::syncSettings(void)
{
}

void HDZeroMsp::syncSettings(AsyncWebSocketClient * const client)
{
    // Send settings
    sendVtxFrequencyToWebsocket(eeprom_storage.vtx_freq, client);
}

void HDZeroMsp::syncSettings(AsyncEventSourceClient * const client)
{
    String json = "{\"vtxfreq\":";
    json += eeprom_storage.vtx_freq;
    json += '}';
    async_event_send(json, "vtxfreq", client);
    async_event_send(m_version_info, "vrx_version", client);
}

int HDZeroMsp::parseSerialData(uint8_t const chr)
{
    if (_handler.processReceivedByte(chr)) {
        /* Handle the received MSP message */
        mspPacket_t & msp_in = _handler.getPacket();
        String info = "";
        if (msp_in.type != MSP_PACKET_V2_COMMAND || msp_in.type != MSP_PACKET_V2_RESPONSE) {
            info = "Invalid MSP received! func: ";
            info += msp_in.function;

        } else if (msp_in.function == HDZ_MSP_FUNC_FIRMWARE_GET) {
            m_version_info = "TODO:need2decode";
            if (init_state == STATE_GET_FW_VER)
                init_state = STATE_GET_CH_INDEX;

        } else if (msp_in.function == HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET) {
            uint16_t const freq = getFreqByIndex(msp_in.payload[0]);
            info = "Channel index received: ";
            info += msp_in.payload[0];
            info += " -> ";
            info += freq;
            info += "MHz";

            // TODO, FIXME: Do something with the current frequency!
            if (freq && eeprom_storage.vtx_freq != freq) {
                eeprom_storage.vtx_freq = freq;
                eeprom_storage.markDirty();
                sendVtxFrequencyToWebsocket(freq);
            }

            if (init_state == STATE_GET_CH_INDEX)
                init_state = STATE_GET_FREQ;

        } else if (msp_in.function == HDZ_MSP_FUNC_FREQUENCY_GET) {
            uint16_t const freq = parseFreq(msp_in.payload);
            info = "Frequency received: ";
            info += freq;
            info += "MHz";

            // TODO, FIXME: Do something with the current frequency!
            if (eeprom_storage.vtx_freq != freq) {
                eeprom_storage.vtx_freq = freq;
                eeprom_storage.markDirty();
                sendVtxFrequencyToWebsocket(freq);
            }

            if (init_state == STATE_GET_FREQ)
                init_state = STATE_GET_RECORDING;

        } else if (msp_in.function == HDZ_MSP_FUNC_RECORDING_STATE_GET) {
            info = "Recording state received: ";
            info += msp_in.payload[0] ? "ON" : "OFF";

            sendVRecordingStateToWebsocket(msp_in.payload[0]);

            if (init_state == STATE_GET_RECORDING)
                init_state = STATE_READY;
        } else if (msp_in.function == HDZ_MSP_FUNC_VRX_MODE_GET) {
            info += "HDZ_MSP_FUNC_VRX_MODE_GET";
        } else if (msp_in.function == HDZ_MSP_FUNC_RSSI_GET) {
            info += "HDZ_MSP_FUNC_RSSI_GET";
        } else if (msp_in.function == HDZ_MSP_FUNC_BATTERY_VOLTAGE_GET) {
            info += "HDZ_MSP_FUNC_BATTERY_VOLTAGE_GET";
        }

        if (info.length()) {
            websocket_send_txt(info);
        } else {
            espnow_send_msp(msp_in);
        }

        _handler.markPacketFree();
    } else if (!_handler.mspOngoing()) {
        return -1;
    }
    return 0;
}

int HDZeroMsp::parseCommand(char const * cmd, size_t const len, AsyncWebSocketClient * const client)
{
    char * temp;
    // ExLRS setting commands
    temp = strstr(cmd, "SET_text=");
    if (temp) {
        handleUserTextCommand(&temp[9], (len - 9));
        return 0;
    }
    return -1;
}

int HDZeroMsp::parseCommand(websoc_bin_hdr_t const * const cmd, size_t const len, AsyncWebSocketClient * const client)
{
    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send_txt(settings_out, client);
        return -1;
    }

    switch (cmd->msg_id) {
        case WSMSGID_VIDEO_FREQ: {
            uint16_t const freq = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            handleVtxFrequencyCommand(freq, client);
            break;
        }
        case WSMSGID_RECORDING_CTRL: {
            handleRecordingStateCommand(!!cmd->payload[0]);
            break;
        }
        default:
            return -1;
    }
    return 0;
}

// This is received from outside (ESP-NOW)
int HDZeroMsp::parseCommand(mspPacket_t & msp_in)
{
    /* Just validate the packet and let the espnow handler to forward it */

    uint16_t const freq = checkInputMspVtxSet(msp_in);
    if (freq) {
        /* Pass command to HDZero */
        handleVtxFrequencyCommand(freq, NULL, false);
        /* Infrom web clients */
        sendVtxFrequencyToWebsocket(freq);
        return 0;
    }

    /*  HDZero uses only MSP V2 */
    if (msp_in.type == MSP_PACKET_V2_COMMAND) {
        /* Check the allowed functions */
        switch (msp_in.function) {
            case HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET: {
                uint16_t const freq = getFreqByIndex(msp_in.payload[0]);
                if (!freq)
                    return 0;
                if (eeprom_storage.vtx_freq != freq) {
                    eeprom_storage.vtx_freq = freq;
                    eeprom_storage.markDirty();
                }
                sendVtxFrequencyToWebsocket(freq);
                break;
            }
            case HDZ_MSP_FUNC_FREQUENCY_SET: {
                uint16_t const freq = parseFreq(msp_in.payload);
                if (!freq || !getIndexByFreq(freq))
                    return 0; // ignore invalid freqs
                if (eeprom_storage.vtx_freq != freq) {
                    eeprom_storage.vtx_freq = freq;
                    eeprom_storage.markDirty();
                }
                sendVtxFrequencyToWebsocket(freq);
                break;
            }
            case HDZ_MSP_FUNC_RECORDING_STATE_SET: {
                sendVRecordingStateToWebsocket(msp_in.payload[0]);
            }
            default:
                break;
        }

        if ((HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET <= msp_in.function && HDZ_MSP_FUNC_BUZZER_SET >= msp_in.function) ||
            HDZ_MSP_FUNC_OSD_ELEMENT_SET == msp_in.function) {
            /* Return -1 to get packet written to HDZero VRX */
            return -1;
        }
    }
    /* Return 0 to ignore packet */
    return 0;
}

void HDZeroMsp::loop(void)
{
#if !UART_DEBUG_EN
    uint32_t const now = millis();
    if (RETRY_INTERVAL_MS <= (int32_t)(now - init_called_ms)) {
        init_called_ms = now;
        switch (init_state) {
            case STATE_GET_FW_VER:
                getFwVersion();
                break;
            case STATE_GET_CH_INDEX:
                getChannelIndex();
                break;
            case STATE_GET_FREQ:
                getFrequency();
                break;
            case STATE_GET_RECORDING:
                getRecordingState();
                break;
            case STATE_READY:
            default:
                break;
        };
    }
#endif // !UART_DEBUG_EN
}

void HDZeroMsp::sendMspToHdzero(uint8_t const * const buff, uint16_t const len, uint16_t const function)
{
    // Fill MSP packet
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = function;
    msp_out.payloadSize = (!buff) ? 0 : len;
    if (buff && len)
        memcpy((void *)msp_out.payload, buff, len);
    // Send packet to HDZero
    MSP::sendPacket(&msp_out, _serial);
}

void HDZeroMsp::handleUserTextCommand(const char * input, size_t const len)
{
    if (input == NULL)
        return;

    // Write to HDZ VRX
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = HDZ_MSP_FUNC_OSD_ELEMENT_SET;
    msp_out.payloadSize = len + 4;
    msp_out.payload[0] = 0x3; // Write string
    // Use fixed position
    msp_out.payload[1] = 0; // row
    msp_out.payload[2] = 0; // column
    msp_out.payload[3] = 0; // attribute, 0x80 for DISPLAYPORT_ATTR_BLINK
    memcpy(&msp_out.payload[4], input, len);
    MSP::sendPacket(&msp_out, _serial);

    String dbg_info = "OSD Text: '";
    dbg_info += input;
    dbg_info += "'";
    websocket_send_txt(dbg_info);
}

void HDZeroMsp::handleVtxFrequencyCommand(uint16_t const freq, AsyncWebSocketClient * const client, bool const espnow)
{
    if (storeVtxFreq(client, freq) == 0) {
        return;
    }

    // Send to HDZero
#if 0
    uint8_t chan_index = getIndexByFreq(freq);
    sendMspToHdzero(&chan_index, 1, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET);
#else
    uint8_t payload[] = {(uint8_t)(freq & 0xff), (uint8_t)(freq >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_FREQUENCY_SET);
#endif

    // Send to other esp-now clients
    if (espnow) {
        msp_out.function = MSP_VTX_SET_CONFIG;
        espnow_send_msp(msp_out);
    }

    getChannelIndex();
    getFrequency();
}

void HDZeroMsp::handleRecordingStateCommand(uint8_t const start)
{
    uint16_t const delay_s = 1;
    // Send to HDZero
    uint8_t payload[] = {start, (uint8_t)(delay_s & 0xff), (uint8_t)(delay_s >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_RECORDING_STATE_SET);
    sendVRecordingStateToWebsocket(start);
}

void HDZeroMsp::sendVtxFrequencyToWebsocket(uint16_t const freq, AsyncWebSocketClient * const client)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void HDZeroMsp::sendVRecordingStateToWebsocket(uint8_t const state, AsyncWebSocketClient * const client)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_RECORDING_CTRL >> 8),
        (uint8_t)WSMSGID_RECORDING_CTRL,
        state,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void HDZeroMsp::getFwVersion(void)
{
    // websocket_send_txt("getChannelIndex()");
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_FIRMWARE_GET);
}

void HDZeroMsp::getChannelIndex(void)
{
    // websocket_send_txt("getChannelIndex()");
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET);
}

void HDZeroMsp::getFrequency(void)
{
    // websocket_send_txt("getFrequency()");
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_FREQUENCY_GET);
}

void HDZeroMsp::getRecordingState(void)
{
    // websocket_send_txt("getRecordingState()");
    sendMspToHdzero(NULL, 0, HDZ_MSP_FUNC_RECORDING_STATE_GET);
}
