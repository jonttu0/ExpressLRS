#include "hdzero_msp.h"
#include "main.h"
#include "storage.h"
#include "comm_espnow.h"
#include "led.h"
#include "buzzer.h"


#define MSP_SAVE_DELAY_MS   100
#define MSP_DEBUG_RX        0

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
#define HDZ_MSP_FUNC_OSD_ELEMENT_SET        0x00B6


void HDZeroMsp::init(void)
{
    _handler.markPacketFree();
    /* Reset values */

    // delay until client is ready
    //delay(7000);
}


void HDZeroMsp::syncSettings(int const num)
{
    // Send settings

}


int HDZeroMsp::parse_data(uint8_t const chr) {
    if (_handler.processReceivedByte(chr)) {
        /* Handle the received MSP message */
        mspPacket_t &msp_in = _handler.getPacket();
        String info = "";
        if (msp_in.type != MSP_PACKET_V2_COMMAND &&
                msp_in.type != MSP_PACKET_V2_RESPONSE) {
            info = "MSP received: error func: ";
            info += msp_in.function;
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
    temp = strstr(cmd, "SET_vtx_freq");
    if (temp) {
        handleVtxFrequency(&temp[12], num);
        return 0;
    }
    return -1;
}


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
        }
    }
    /* Return 0 to ignore packet */
    return 0;
}


void HDZeroMsp::loop(void)
{
}


void HDZeroMsp::handleUserText(const char * input, size_t const len)
{
    if (input == NULL)
        return;

    // Write to HDZ VRX
    mspPacket_t msp_out;
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

    websocket_send("OSD Text: ");
    websocket_send(input);
}


void HDZeroMsp::handleVtxFrequency(const char * input, int num)
{
    String settings_out = "[ERROR] invalid command";
    uint16_t freq;

    if (input == NULL || *input == '?') {
        settings_out = "HDZ_CRTL_vtx_freq=";
        settings_out += eeprom_storage.vtx_freq;
    } else if (input[0] == '=') {
        settings_out = "Setting vtx freq to: ";

        freq = (input[1] - '0');
        freq = freq*10 + (input[2] - '0');
        freq = freq*10 + (input[3] - '0');
        freq = freq*10 + (input[4] - '0');

        if (freq == 0)
            return;

        settings_out += freq;
        settings_out += "MHz";

        // Send to VRX
        mspPacket_t msp_out;
        msp_out.reset();
        msp_out.type = MSP_PACKET_V2_COMMAND;
        msp_out.flags = 0;
        msp_out.function = HDZ_MSP_FUNC_FREQUENCY_SET;
        msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
        msp_out.payload[0] = (freq & 0xff);
        msp_out.payload[1] = (freq >> 8);
        MSP::sendPacket(&msp_out, _serial);

        // Send to other esp-now clients
        msp_out.function = MSP_VTX_SET_CONFIG;
        espnow_send_msp(msp_out);
    }
    websocket_send(settings_out, num);
}
