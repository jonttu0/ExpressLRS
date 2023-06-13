#include "msp_handler_base.h"

int MspHandlerBase::parseCommand(websoc_bin_hdr_t const * const cmd,
                                 size_t const len,
                                 AsyncWebSocketClient * const client)
{
    switch (cmd->msg_id) {
        case WSMSGID_VIDEO_FREQ: {
            if (!len) {
                return 0; // Invalid message!
            }
            uint16_t const freq = parseFreq(cmd->payload);
            if (storeVtxFreq(client, freq)) {
                handleVtxFrequencyCommand(freq, client);
                espnow_vtxset_send(freq);
            }
            break;
        }
        case WSMSGID_LAPTIMER_OSD_POS: {
            uint16_t const row = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            uint16_t const col = ((uint16_t)cmd->payload[3] << 8) + cmd->payload[2];
            if (osdRowMax() <= row) {
                websocket_send_txt("[ERROR] Invalid row value.");
            } else if (osdColumnMax() <= col) {
                websocket_send_txt("[ERROR] Invalid column value.");
            } else {
                eeprom_storage.laptimer_osd_pos.row = row;
                eeprom_storage.laptimer_osd_pos.column = col;
                eeprom_storage.markDirty();
            }
            break;
        }
        case WSMSGID_LAPTIMER_OSD_TIMEOUT: {
            uint16_t const timeout = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            eeprom_storage.laptimer_osd_timeout = timeout * 1000;
            eeprom_storage.markDirty();
            break;
        }
        default:
            return parseCommandPriv(cmd, len, client);
    }
    return 0;
}

// Handle received MSP packet
int MspHandlerBase::parseCommand(mspPacket_t & msp_in)
{
    if (msp_in.type == MSP_PACKET_V2_COMMAND) {
        if (msp_in.function == MSP_VTX_SET_CONFIG && 2 <= msp_in.payloadSize) {
            uint16_t const freq = parseFreq(msp_in.payload);
            if (storeVtxFreq(NULL, freq)) {
                handleVtxFrequencyCommand(freq, NULL);
                /* Infrom web clients */
                clientSendVtxFrequency(freq);
            }
            return 0;
        }
    }
    return parseCommandPriv(msp_in);
}

void MspHandlerBase::clientSendVtxFrequency(uint16_t const freq, AsyncWebSocketClient * const client)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void MspHandlerBase::clientSendVRecordingState(uint8_t const state, AsyncWebSocketClient * const client)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_RECORDING_CTRL >> 8),
        (uint8_t)WSMSGID_RECORDING_CTRL,
        state,
    };
    websocket_send_bin(response, sizeof(response), client);
}

void MspHandlerBase::clientSendLaptimerState(uint16_t const race_id,
                                             uint16_t const round_num,
                                             bool const state,
                                             AsyncWebSocketClient * const client)
{
    m_laptimer_state = state;
    uint8_t response[] = {
        (uint8_t)(WSMSGID_LAPTIMER_START_STOP >> 8),
        (uint8_t)WSMSGID_LAPTIMER_START_STOP,
        (uint8_t)(race_id >> 8),
        (uint8_t)race_id,
        (uint8_t)state,
        (uint8_t)(round_num >> 8),
        (uint8_t)round_num,
    };
    websocket_send_bin(response, sizeof(response), client);
    handleLaptimerState(race_id, round_num, state, client);
}

void MspHandlerBase::clientSendLaptimerLap(laptimer_lap_t const * lap, AsyncWebSocketClient * const client)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_LAPTIMER_LAPTIME >> 8),
        (uint8_t)WSMSGID_LAPTIMER_LAPTIME,
        (uint8_t)(lap->lap_time_ms >> 24),
        (uint8_t)(lap->lap_time_ms >> 16),
        (uint8_t)(lap->lap_time_ms >> 8),
        (uint8_t)lap->lap_time_ms,
        (uint8_t)(lap->race_id >> 8),
        (uint8_t)lap->race_id,
        (uint8_t)(lap->lap_index),
    };
    websocket_send_bin(response, sizeof(response), client);
    handleLaptimerLap(lap, client);
}
