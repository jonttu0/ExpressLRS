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
 *  Set OSD Element         0x030C
 *  Set Head Tracking       0x030D
 *      - enable/disable head-tracking forwarding packets to the TX
 *  Set RTC                 0x030E
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
#define HDZ_MSP_FUNC_OSD_ELEMENT_SET        0x030C
#define HDZ_MSP_FUNC_SET_HEAD_TRACKING      0x030D
#define HDZ_MSP_FUNC_SET_RTC                0x030E

#define MSP_ELRS_SET_OSD                    0x00B6


// Incoming packets originating from the VRx
#define MSP_ELRS_BACKPACK_SET_MODE              0x0380  // enable wifi/binding mode
#define MSP_ELRS_BACKPACK_GET_VERSION           0x0381  // get the bacpack firmware version
#define MSP_ELRS_BACKPACK_GET_STATUS            0x0382  // get the status of the backpack
#define MSP_ELRS_BACKPACK_SET_PTR               0x0383  // forwarded back to TX backpack



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
    clientSendVtxFrequency(eeprom_storage.vtx_freq, client);
}

void HDZeroMsp::syncSettings(AsyncEventSourceClient * const client)
{
    String json = "{\"vtxfreq\":";
    json += eeprom_storage.vtx_freq;
    json += ",\"recording_control\":1";
    json += ",\"osd_text\":1";
    json += ",\"laptimer\":1";
    json += ",\"vrxversion\":1";
    json += ",\"espnow\":";
    json += ESP_NOW;
    json += ",\"model\":\"HDZero\"";
    json += '}';
    async_event_send(json, "fea_config", client);
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
            m_version_info = "FW info not available!";
            if (msp_in.payloadSize) {
                m_version_info = "FW: ";
                m_version_info += msp_in.payload[1];
                m_version_info += '.';
                m_version_info += msp_in.payload[2];
                m_version_info += '.';
                m_version_info += msp_in.payload[3];
            }
            info = m_version_info;
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
                clientSendVtxFrequency(freq);
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
                clientSendVtxFrequency(freq);
            }

            if (init_state == STATE_GET_FREQ)
                init_state = STATE_GET_RECORDING;

        } else if (msp_in.function == HDZ_MSP_FUNC_RECORDING_STATE_GET) {
            info = "Recording state received: ";
            info += msp_in.payload[0] ? "ON" : "OFF";

            clientSendVRecordingState(msp_in.payload[0]);

            if (init_state == STATE_GET_RECORDING)
                init_state = STATE_READY;

        } else if (msp_in.function == HDZ_MSP_FUNC_VRX_MODE_GET) {
            info += "HDZ_MSP_FUNC_VRX_MODE_GET";

        } else if (msp_in.function == HDZ_MSP_FUNC_RSSI_GET) {
            info  = "RSSI: 1)";
            info += msp_in.payload[1];
            info += " 2)";
            info += msp_in.payload[2];
            info += " 3)";
            info += msp_in.payload[3];
            info += " 4)";
            info += msp_in.payload[4];

        } else if (msp_in.function == HDZ_MSP_FUNC_BATTERY_VOLTAGE_GET) {
            uint16_t const voltage = ((uint16_t)msp_in.payload[1] << 8) + msp_in.payload[0];
            info  = "Battery Voltage: ";
            info += voltage;

        } else if (msp_in.function == MSP_ELRS_BACKPACK_SET_MODE) {
            info += "MSP_ELRS_BACKPACK_SET_MODE...";
            if (msp_in.payloadSize == 1) {
                if (msp_in.payload[0] == 'B') {
                    info += "Enter binding...";
                } else if (msp_in.payload[0] == 'W') {
                    info += "Enable WIFI...";
                }
                // send "in-progress" response
                uint8_t data = 'P';
                sendMspToHdzero(&data, sizeof(data), MSP_ELRS_BACKPACK_SET_MODE);
            }
        } else if (msp_in.function == MSP_ELRS_BACKPACK_GET_VERSION) {
            info += "MSP_ELRS_BACKPACK_GET_VERSION...";
            sendMspToHdzero((uint8_t*)version_string, strlen(version_string), MSP_ELRS_BACKPACK_SET_MODE);
        } else if (msp_in.function == MSP_ELRS_BACKPACK_GET_STATUS) {
            info += "MSP_ELRS_BACKPACK_GET_STATUS...";
                static const uint8_t unbound[6] = {0,0,0,0,0,0};
                enum {
                    WIFI_UPDATE = 1 << 0,
                    BINDING_MODE = 1 << 1,
                    BOUND_OK = 1 << 2,
                };
                uint8_t const my_uid[] = {MY_UID};
                uint8_t response[1 + sizeof(my_uid)];
                response[0] = BOUND_OK;
                memcpy(&response[1], my_uid, sizeof(my_uid));
                sendMspToHdzero(response, sizeof(response), MSP_ELRS_BACKPACK_GET_STATUS);
        } else if (msp_in.function == MSP_ELRS_BACKPACK_SET_PTR) {
            info += "MSP_ELRS_BACKPACK_SET_PTR...";
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

void HDZeroMsp::handleVtxFrequencyCmd(uint16_t const freq, AsyncWebSocketClient * const client)
{
    if (freq && eeprom_storage.vtx_freq == freq) {
        // Freq is already okhandleVtxFrequencyCommand
        return;
    }
    // Adjust VRx
    handleVtxFrequencyCommand(freq, client);
    // ... GUI is updated when channel query resp is received
}

int HDZeroMsp::parseCommandPriv(char const * cmd, size_t const len, AsyncWebSocketClient * const client)
{
    char * temp;
    // ExLRS setting commands
    temp = strstr(cmd, "SET_text=");
    if (temp) {
        handleUserTextCommand(&temp[9], (len - 9), client);
        return 0;
    }
    return -1;
}

int HDZeroMsp::parseCommandPriv(websoc_bin_hdr_t const * const cmd, size_t const len, AsyncWebSocketClient * const client)
{
    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send_txt(settings_out, client);
        return -1;
    }

    switch (cmd->msg_id) {
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
int HDZeroMsp::parseCommandPriv(mspPacket_t & msp_in)
{
    /* Just validate the packet and let the espnow handler to forward it */

    /*  HDZero uses only MSP V2 */
    if (msp_in.type == MSP_PACKET_V2_COMMAND || msp_in.type == MSP_PACKET_V2_RESPONSE) {
        switch (msp_in.function) {
            case HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET: {
                uint16_t const freq = getFreqByIndex(msp_in.payload[0]);
                if (!freq)
                    return 0;
                if (eeprom_storage.vtx_freq != freq) {
                    eeprom_storage.vtx_freq = freq;
                    eeprom_storage.markDirty();
                }
                clientSendVtxFrequency(freq);
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
                clientSendVtxFrequency(freq);
                break;
            }
            case HDZ_MSP_FUNC_RECORDING_STATE_SET: {
                clientSendVRecordingState(msp_in.payload[0]);
            }

            case MSP_ELRS_FUNC: {
                /* Ingore */
                return 0;
            }

            case HDZ_MSP_FUNC_OSD_ELEMENT_SET: {
                // OSD element write function is diffrent so change it
                msp_in.function = MSP_ELRS_SET_OSD;
                return -1;  // Forward to VRX
            }

            default:
                break;
        }

        if ((HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET <= msp_in.function && HDZ_MSP_FUNC_SET_RTC >= msp_in.function)) {
            /* Return -1 to get packet written to VRX */
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

void HDZeroMsp::handleUserTextCommand(const char * input, size_t const len, AsyncWebSocketClient * const client)
{
    enum {
        OSD_CMD_HEARTBEAT = 0,
        OSD_CMD_RELEASE_PORT = 1,
        OSD_CMD_SCREEN_CLEAR = 2,
        OSD_CMD_SCREEN_WRITE = 3,
        OSD_CMD_SCREEN_DRAW = 4,
    };

    if (input == NULL)
        return;

    // Clear OSD
    uint8_t payload = OSD_CMD_SCREEN_CLEAR;
    sendMspToHdzero(&payload, sizeof(payload), MSP_ELRS_SET_OSD);

    // Fill OSD data
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = MSP_ELRS_SET_OSD;
    msp_out.payloadSize = len + 4;
    msp_out.payload[0] = OSD_CMD_SCREEN_WRITE;
    // Use fixed position for now...
    //  VMAX = 18
    //  HMAX = 50
    msp_out.payload[1] = 8; // row
    msp_out.payload[2] = 0; // column
    msp_out.payload[3] = 0; // attribute, 0x80 for DISPLAYPORT_ATTR_BLINK
    memcpy(&msp_out.payload[4], input, len);
    MSP::sendPacket(&msp_out, _serial);

    // Draw OSD
    payload = OSD_CMD_SCREEN_DRAW;
    sendMspToHdzero(&payload, sizeof(payload), MSP_ELRS_SET_OSD);

    String dbg_info = "OSD Text: '";
    for (size_t iter = 0; iter < len; iter++)
        dbg_info += input[iter];
    dbg_info += "'";
    websocket_send_txt(dbg_info, client);
}

void HDZeroMsp::handleVtxFrequencyCommand(uint16_t const freq, AsyncWebSocketClient * const client)
{
    // Send to HDZero
#if 0
    uint8_t chan_index = getIndexByFreq(freq);
    sendMspToHdzero(&chan_index, 1, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET);
#else
    uint8_t payload[] = {(uint8_t)(freq & 0xff), (uint8_t)(freq >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_FREQUENCY_SET);
#endif
    getChannelIndex();
    getFrequency();
}

void HDZeroMsp::handleRecordingStateCommand(uint8_t const start)
{
    uint16_t const delay_s = 1;
    // Send to HDZero
    uint8_t payload[] = {start, (uint8_t)(delay_s & 0xff), (uint8_t)(delay_s >> 8)};
    sendMspToHdzero(payload, sizeof(payload), HDZ_MSP_FUNC_RECORDING_STATE_SET);
    clientSendVRecordingState(start);
}

void HDZeroMsp::getFwVersion(void)
{
    // websocket_send_txt("getFwVersion()");
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

void HDZeroMsp::handleLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client)
{
    String info = "Lap time lap:";
    info += lap->lap_index;
    info += ", ms:";
    info += lap->lap_time_ms;
    info += ", ";

    // uint32_t const milliseconds = lap->lap_time_ms % 1000;
    // uint32_t const seconds = lap->lap_time_ms / 1000;
    // lap->lap_index;

    // TODO: Push to OSD

    websocket_send_txt(info, client);
}
