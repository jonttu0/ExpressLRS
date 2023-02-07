#include "hdzero_msp.h"
#include "main.h"
#include "storage.h"
#include "comm_espnow.h"
#include "led.h"
#include "buzzer.h"


#define RETRY_INTERVAL_MS  1000

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
//#define MSP_ELRS_BACKPACK_SET_OSD_ELEMENT       0x030C
#define HDZ_MSP_FUNC_OSD_ELEMENT_SET        0x00B6


void HDZeroMsp::init(void)
{
    _handler.markPacketFree();
    /* Reset values */
    //init_called_ms = millis() + 2000; // Delay first query
    init_state = STATE_GET_CH_INDEX;
}


void HDZeroMsp::syncSettings(int const num)
{
    // Send settings
    sendVtxFrequencyToWebsocket(eeprom_storage.vtx_freq);
}


int HDZeroMsp::parse_data(uint8_t const chr) {
    if (_handler.processReceivedByte(chr)) {
        /* Handle the received MSP message */
        mspPacket_t &msp_in = _handler.getPacket();
        String info = "";
        if (msp_in.type != MSP_PACKET_V2_COMMAND ||
                msp_in.type != MSP_PACKET_V2_RESPONSE) {
            info = "Invalid MSP received! func: ";
            info += msp_in.function;

        } else if (msp_in.function == HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET) {
            info = "Channel index received: ";
            info += msp_in.payload[0];
            uint16_t freq = 0;
            uint8_t const band = msp_in.payload[0] >> 3;
            uint8_t const channel = msp_in.payload[0] & 7;
            if (band == 3) { // F
                uint16_t const r_freqs[] = {0, 5760, 0, 5800, 0, 0, 0, 0};
                freq = r_freqs[channel];
            } else if (band == 4) { // R
                uint16_t const r_freqs[] = {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917};
                freq = r_freqs[channel];
            }

            info += " -> ";
            info += freq;

            // TODO, FIXME: Do something with the current frequency!
            if (freq && eeprom_storage.vtx_freq != freq) {
                eeprom_storage.vtx_freq = freq;
                eeprom_storage.markDirty();
                sendVtxFrequencyToWebsocket(freq);
            }

            if (init_state == STATE_GET_CH_INDEX)
                init_state = STATE_GET_FREQ;

        } else if (msp_in.function == HDZ_MSP_FUNC_FREQUENCY_GET) {
            uint16_t const freq = msp_in.payload[0] + ((uint16_t)msp_in.payload[1] << 8);
            info = "Frequency received: ";
            info += freq;

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
        }

        if (info.length()) {
            websocket_send(info);
        } else {
            espnow_send_msp(msp_in);
        }

        _handler.markPacketFree();
    } else if (!_handler.mspOngoing()) {
        return -1;
    }
    return 0;
}


int HDZeroMsp::parse_command(char * cmd, size_t len, int const num)
{
    char * temp;
    // ExLRS setting commands
    temp = strstr(cmd, "SET_text=");
    if (temp) {
        handleUserText(&temp[9], (len - 9));
        return 0;
    }
    return -1;
}


int HDZeroMsp::parse_command(websoc_bin_hdr_t const * const cmd, size_t const len, int const num)
{
    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send(settings_out, num);
        return -1;
    }

    switch (cmd->msg_id) {
        case WSMSGID_VIDEO_FREQ: {
            uint16_t const freq = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            handleVtxFrequency(freq, num);
            break;
        }
        case WSMSGID_RECORDING_CTRL: {
            handleRecordingState(!!cmd->payload[0]);
            break;
        }
        default:
            return -1;
    }
    return 0;
}


// This is received from outside (ESP-NOW)
int HDZeroMsp::handle_received_msp(mspPacket_t &msp_in)
{
    /* Just validate the packet and let the espnow handler to forward it */

    /*  HDZero uses only MSP V2 */
    if (msp_in.type == MSP_PACKET_V2_COMMAND) {
        /* Check the allowed functions */
        if ((HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET <= msp_in.function &&
                    HDZ_MSP_FUNC_BUZZER_SET >= msp_in.function) ||
                HDZ_MSP_FUNC_OSD_ELEMENT_SET == msp_in.function) {
            /* Return -1 to get packet written to HDZero VRX */
            return -1;
        } else if (msp_in.function == MSP_VTX_SET_CONFIG) {
            uint16_t freq = msp_in.payload[1];
            freq <<= 8;
            freq += msp_in.payload[0];
            if (3 <= msp_in.payloadSize) {
                // power
            }
            if (4 <= msp_in.payloadSize) {
                // pitmode
            }
            /* Pass command to HDZero */
            handleVtxFrequency(freq, -1, false);
            /* Infrom web clients */
            sendVtxFrequencyToWebsocket(freq);
        }
    }
    /* Return 0 to ignore packet */
    return 0;
}


void HDZeroMsp::loop(void)
{
    uint32_t const now = millis();
    if (RETRY_INTERVAL_MS <= (int32_t)(now - init_called_ms)) {
        init_called_ms = now;
        switch (init_state) {
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
}

void HDZeroMsp::MspWrite(uint8_t const * const buff, uint16_t const len, uint16_t const function)
{
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = function;
    msp_out.payloadSize = (!buff) ? 0 : len;
    if (buff && len)
        memcpy((void*)msp_out.payload, buff, len);
    // Send packet to HDZero
    MSP::sendPacket(&msp_out, _serial);
}

void HDZeroMsp::handleUserText(const char * input, size_t const len)
{
    if (input == NULL)
        return;

    // Write to HDZ VRX
    msp_out.reset();
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
    websocket_send(dbg_info);
}


void HDZeroMsp::handleVtxFrequency(uint16_t const freq, int const num, bool const espnow)
{
    String dbg_info = "Setting vtx freq to: ";
    dbg_info += freq;
    dbg_info += "MHz";

    if (freq == 0)
        return;

    if (eeprom_storage.vtx_freq != freq) {
        eeprom_storage.vtx_freq = freq;
        eeprom_storage.markDirty();
    }
#if 1
    // MAP freq to HDZ channel index
    uint8_t chan_index = 4*8;
    switch (freq) {
        case 5658: // R1
            chan_index += 0;
            break;
        case 5695:
            chan_index += 1;
            break;
        case 5732:
            chan_index += 2;
            break;
        case 5769:
            chan_index += 3;
            break;
        case 5806:
            chan_index += 4;
            break;
        case 5843:
            chan_index += 5;
            break;
        case 5880:
            chan_index += 6;
            break;
        case 5917: // R8
            chan_index += 7;
            break;
        case 5760: // F2
            chan_index = 3*8 + 1;
            break;
        case 5800: // F4
            chan_index = 3*8 + 3;
            break;
    }
    MspWrite(&chan_index, 1, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_SET);
    getChannelIndex();
#else
    // Send to HDZero
    uint8_t payload[] = {(uint8_t)(freq & 0xff), (uint8_t)(freq >> 8)};
    //payload[2] = power;
    //payload[3] = (power == 0); // pit mode
    MspWrite(payload, sizeof(payload), HDZ_MSP_FUNC_FREQUENCY_SET);
    MspWrite(payload, sizeof(payload), HDZ_MSP_FUNC_FREQUENCY_GET);
#endif

    // Send to other esp-now clients
    if (espnow) {
        msp_out.function = MSP_VTX_SET_CONFIG;
        espnow_send_msp(msp_out);
    }

    websocket_send(dbg_info, num);
}


void HDZeroMsp::handleRecordingState(uint8_t const start)
{
    uint16_t const delay_s = 1;
    // Send to HDZero
    uint8_t payload[] = {start, (uint8_t)(delay_s & 0xff), (uint8_t)(delay_s >> 8)};
    MspWrite(payload, sizeof(payload), HDZ_MSP_FUNC_RECORDING_STATE_SET);
    sendVRecordingStateToWebsocket(start);
}


void HDZeroMsp::sendVtxFrequencyToWebsocket(uint16_t const freq)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send(response, sizeof(response));
}


void HDZeroMsp::sendVRecordingStateToWebsocket(uint8_t const state)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_RECORDING_CTRL >> 8),
        (uint8_t)WSMSGID_RECORDING_CTRL,
        state,
    };
    websocket_send(response, sizeof(response));
}


void HDZeroMsp::getChannelIndex(void)
{
    //websocket_send("getChannelIndex()");
    MspWrite(NULL, 0, HDZ_MSP_FUNC_BAND_CHANNEL_INDEX_GET);
}


void HDZeroMsp::getFrequency(void)
{
    //websocket_send("getFrequency()");
    MspWrite(NULL, 0, HDZ_MSP_FUNC_FREQUENCY_GET);
}


void HDZeroMsp::getRecordingState(void)
{
    //websocket_send("getRecordingState()");
    MspWrite(NULL, 0, HDZ_MSP_FUNC_RECORDING_STATE_GET);
}
