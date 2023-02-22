#include "expresslrs_msp.h"
#include "main.h"
#include "storage.h"
#include "comm_espnow.h"
#include "handset.h"
#include "led.h"
#include "buzzer.h"
#include "rc_channels.h"

#define MSP_SAVE_DELAY_MS 100

enum {
    // ELRS messages
    WSMSGID_ELRS_SETTINGS = WSMSGID_BASE_ELRS,
    WSMSGID_ELRS_DOMAIN,
    WSMSGID_ELRS_RATE,
    WSMSGID_ELRS_POWER,
    WSMSGID_ELRS_TLM,
    WSMSGID_ELRS_RF_POWER_TEST,

    WSMSGID_TLM_GPS = WSMSGID_BASE_TELEMETRY,

#if CONFIG_HANDSET
    // Handset messages
    WSMSGID_HANDSET_MIXER = WSMSGID_BASE_HANDSET,
    WSMSGID_HANDSET_CALIBRATE,
    WSMSGID_HANDSET_ADJUST,
    WSMSGID_HANDSET_RELOAD,
    WSMSGID_HANDSET_SAVE,
    WSMSGID_HANDSET_BATT_INFO,
    WSMSGID_HANDSET_BATT_CONFIG,

    WSMSGID_HANDSET_TLM_LINK_STATS = WSMSGID_BASE_HANDSET_TLM,
    WSMSGID_HANDSET_TLM_BATTERY,
    WSMSGID_HANDSET_TLM_GPS,
#endif
};

int ExpresslrsMsp::send_current_values(void * client)
{
    if (!settings_valid) {
        // Not valid, get latest first
        uint8_t buff[] = {ELRS_CMD_GET, 0};
        SettingsWrite(buff, sizeof(buff));
        return 0;
    }

    String data = syncSettingsJson();
    async_event_send(data, "elrs_settings");
    uint8_t settings_resp[] = {
        (uint8_t)(WSMSGID_ELRS_SETTINGS >> 8),
        (uint8_t)WSMSGID_ELRS_SETTINGS,
        settings_region,
        settings_rate,
        settings_power,
        settings_power_max,
        settings_tlm,
        (uint8_t)(eeprom_storage.vtx_freq >> 8),
        (uint8_t)eeprom_storage.vtx_freq,
    };
    websocket_send_bin(settings_resp, sizeof(settings_resp), (AsyncWebSocketClient *)client);
    return 0;
}

void ExpresslrsMsp::SettingsWrite(uint8_t * buff, uint8_t len)
{
    // Fill MSP packet
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_INT_MSP_PARAMS;
    msp_out.payloadSize = len;
    memcpy((void *)msp_out.payload, buff, len);
    // Send packet
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::MspWrite(uint8_t * buff, uint8_t const len, uint8_t const function)
{
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_CMD;
    msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
    msp_out.function = function;
    msp_out.payloadSize = (!buff) ? 0 : len;
    if (buff && len)
        memcpy((void *)msp_out.payload, buff, len);
    // Send packet to ELRS
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleVtxFrequency(uint16_t const freq, void * client)
{
    if (freq == 0)
        return;

    // Send to ELRS
    sendVtxFrequencyToSerial(freq, client);

    // Send to other esp-now clients
    msp_out.reset();
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = MSP_VTX_SET_CONFIG;
    msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
    msp_out.payload[0] = (freq & 0xff);
    msp_out.payload[1] = (freq >> 8);
    espnow_send_msp(msp_out);
}

void ExpresslrsMsp::sendVtxFrequencyToSerial(uint16_t const freq, void * client)
{
    String dbg_info = "Setting vtx freq to: ";
    dbg_info += freq;
    dbg_info += "MHz";
    websocket_send_txt(dbg_info, (AsyncWebSocketClient *)client);

    if (freq == 0)
        return;

    if (eeprom_storage.vtx_freq != freq) {
        eeprom_storage.vtx_freq = freq;
        eeprom_storage.markDirty();
    }

    uint8_t payload[] = {(uint8_t)(freq & 0xff), (uint8_t)(freq >> 8)};
    // payload[2] = power;
    // payload[3] = (power == 0); // pit mode
    MspWrite(payload, sizeof(payload), MSP_VTX_SET_CONFIG);

    // Request save to make set permanent
    msp_send_save_requested_ms = millis();
}

void ExpresslrsMsp::sendVtxFrequencyToWebsocket(uint16_t const freq)
{
    uint8_t vtx_response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send_bin(vtx_response, sizeof(vtx_response), NULL);
}

#if CONFIG_HANDSET
void ExpresslrsMsp::handleHandsetCalibrate(uint8_t const * const input)
{
    uint_fast8_t const control_dir = input[0] & 0xF;
    uint_fast8_t const control_axis = (input[0] >> 4) & 0xF;
    uint8_t value = 0;

    if (control_dir == 1)
        value = GIMBAL_CALIB_LOW;
    else if (control_dir == 2)
        value = GIMBAL_CALIB_MID;
    else if (control_dir == 3)
        value = GIMBAL_CALIB_HIGH;
    else
        return;

    if (control_axis == 0)
        value |= GIMBAL_CALIB_L_V;
    else if (control_axis == 1)
        value |= GIMBAL_CALIB_L_H;
    else if (control_axis == 2)
        value |= GIMBAL_CALIB_R_V;
    else if (control_axis == 3)
        value |= GIMBAL_CALIB_R_H;
    else
        return;

    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_HANDSET_CALIBRATE;
    msp_out.addByte(value);
    msp_out.addByte(1); // Start
    msp_out.setIteratorToSize();
    // Send packet
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleHandsetCalibrateResp(uint8_t * data, void * client)
{
    static uint8_t calib_response[] = {
        (uint8_t)(WSMSGID_HANDSET_CALIBRATE >> 8),
        (uint8_t)WSMSGID_HANDSET_CALIBRATE,
    };
    websocket_send_bin(calib_response, sizeof(calib_response), (AsyncWebSocketClient *)client);
}

void ExpresslrsMsp::handleHandsetMixer(uint8_t const * const input, size_t const length)
{
    uint32_t iter, index;
    uint8_t const * read = input;
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_HANDSET_MIXER;
    for (iter = 0; iter < ARRAY_SIZE(mixer) && (size_t)(read - input) < length; iter++) {
        // channel index
        index = *read++;
        msp_out.addByte(index);
        // channel out
        msp_out.addByte(*read++);
        // inverted
        msp_out.addByte(*read++);
        // Scale
        if (index < 4) {
            msp_out.addByte(*read++);
        }
    }
    msp_out.setIteratorToSize();
    // Send packet
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleHandsetMixerResp(uint8_t * data, void * client)
{
    static uint8_t mixer_response[2 + 2 + 4 * ARRAY_SIZE(mixer)];
    uint8_t * outptr;
    uint8_t iter;
    if (data) {
        for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
            if (data[0] < ARRAY_SIZE(mixer)) {
                mixer[data[0]] = (struct mixer){.index = data[1], .inv = data[2], .scale = data[3]};
            }
            data += 4;
        }
        handset_num_switches = *data++;
        handset_num_aux = *data++;
    }

    mixer_response[0] = (uint8_t)(WSMSGID_HANDSET_MIXER >> 8);
    mixer_response[1] = (uint8_t)WSMSGID_HANDSET_MIXER;
    mixer_response[2] = handset_num_aux;
    mixer_response[3] = handset_num_switches;
    outptr = &mixer_response[4];
    for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
        outptr[0] = iter;
        outptr[1] = mixer[iter].index;
        outptr[2] = mixer[iter].inv;
        outptr[3] = mixer[iter].scale;
        outptr += 4;
    }
    websocket_send_bin(mixer_response, sizeof(mixer_response), (AsyncWebSocketClient *)client);
}

void ExpresslrsMsp::handleHandsetAdjust(uint8_t const * const input)
{
    uint_fast8_t const control = input[0] & 0xF;
    uint_fast8_t const control_axis = (input[0] >> 4) & 0xF;

    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;

    if (control == 1)
        msp_out.function = ELRS_HANDSET_ADJUST_MIN;
    else if (control == 2)
        msp_out.function = ELRS_HANDSET_ADJUST_MID;
    else if (control == 3)
        msp_out.function = ELRS_HANDSET_ADJUST_MAX;
    else
        return;

    if (4 <= control_axis)
        return;

    msp_out.payload[0] = control_axis;
    msp_out.payload[1] = input[2];
    msp_out.payload[2] = input[1];
    // Send packet
    msp_out.payloadSize = 3;
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleHandsetAdjustResp(uint8_t * data, void * client)
{
    if (data) {
        // update the local values
        memcpy(gimbals, data, sizeof(gimbals));
    }
    // Send to web client(s)
    static uint8_t adjust_info[2 + sizeof(gimbals)];
    adjust_info[0] = (uint8_t)(WSMSGID_HANDSET_ADJUST >> 8);
    adjust_info[1] = (uint8_t)WSMSGID_HANDSET_ADJUST;
    memcpy(&adjust_info[2], gimbals, sizeof(gimbals));
    websocket_send_bin(adjust_info, sizeof(adjust_info), (AsyncWebSocketClient *)client);
}

void ExpresslrsMsp::HandsetConfigGet(void * client, uint8_t force)
{
    if (!handset_mixer_ok || !handset_adjust_ok || force) {
        // Fill MSP packet
        msp_out.reset();
        msp_out.type = MSP_PACKET_V1_ELRS;
        msp_out.flags = MSP_ELRS_INT;
        msp_out.function = ELRS_HANDSET_CONFIGS_LOAD;
        msp_out.payload[0] = 1;
        msp_out.payload[1] = 1;
        msp_out.payloadSize = 2;
        // Send packet
        MSP::sendPacket(&msp_out, _serial);
        websocket_send_txt("Configs load...", (AsyncWebSocketClient *)client);
        return;
    }

    handleHandsetMixerResp(NULL, client);
    delay(10);
    handleHandsetAdjustResp(NULL, client);
}

void ExpresslrsMsp::HandsetConfigSave(void * client)
{
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_HANDSET_CONFIGS_SAVE;
    msp_out.payload[0] = 1;
    msp_out.payload[1] = 1;
    msp_out.payloadSize = 2;
    // Send packet
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleHandsetTlmLnkStatsAndBatt(uint8_t * data)
{
    static uint8_t tlm_info[2 + sizeof(LinkStatsLink_t) + sizeof(LinkStatsBatt_t)];
    LinkStats_t * stats = (LinkStats_t *)data;
    uint8_t * outptr = &tlm_info[2];
    // Adjust values
    stats->link.downlink_RSSI = (uint8_t)(stats->link.downlink_RSSI - 120);
    // fill the message
    tlm_info[0] = (uint8_t)(WSMSGID_HANDSET_TLM_LINK_STATS >> 8);
    tlm_info[1] = (uint8_t)WSMSGID_HANDSET_TLM_LINK_STATS;
    // Add link stats
    memcpy(outptr, &stats->link, sizeof(LinkStatsLink_t));
    outptr += sizeof(LinkStatsLink_t);
    // Add battery info (note: values are in bigendian format)
    outptr[0] = (uint8_t)(stats->batt.voltage);
    outptr[1] = (uint8_t)(stats->batt.voltage >> 8);
    outptr[2] = (uint8_t)(stats->batt.current);
    outptr[3] = (uint8_t)(stats->batt.current >> 8);
    outptr[4] = (uint8_t)(stats->batt.capacity);
    outptr[5] = (uint8_t)(stats->batt.capacity >> 8);
    outptr[6] = (uint8_t)(stats->batt.capacity >> 16);
    outptr[7] = (uint8_t)stats->batt.remaining;
    outptr += 8;
    websocket_send_bin(tlm_info, (outptr - tlm_info));
}

void ExpresslrsMsp::handleHandsetTlmGps(uint8_t * data)
{
    static uint8_t gpc_info[2 + sizeof(GpsOta_t)];
    GpsOta_t * stats = (GpsOta_t *)data;
    // Convert data
    stats->heading /= 10;
    stats->altitude -= 1000;
    // fill the message
    gpc_info[0] = (uint8_t)(WSMSGID_HANDSET_TLM_GPS >> 8);
    gpc_info[1] = (uint8_t)WSMSGID_HANDSET_TLM_GPS;
    memcpy(&gpc_info[2], stats, sizeof(GpsOta_t));
    websocket_send_bin(gpc_info, sizeof(gpc_info));
}

// #define ADC_VOLT(X) (((X) * ADC_R2) / (ADC_R1 + ADC_R2))
#define ADC_SCALE   (ADC_R1 / ADC_R2)
#define ADC_REF_mV  (1075U * ADC_SCALE)
#define ADC_MAX     1023U
#define ADC_VOLT(X) (((uint32_t)(X)*ADC_REF_mV) / ADC_MAX)

static uint32_t batt_voltage;
static uint32_t batt_voltage_warning_limit;
static uint32_t batt_voltage_dead_limit;

static uint32_t batt_voltage_meas_last_ms;
static uint32_t batt_voltage_warning_last_ms;
static uint32_t batt_voltage_warning_timeout = 5000;
#ifdef WS2812_PIN
static uint8_t batt_voltage_last_bright;
#endif

void ExpresslrsMsp::battery_voltage_report(void * client)
{
    uint8_t batt_info[] = {(uint8_t)(WSMSGID_HANDSET_BATT_INFO >> 8),
                           (uint8_t)WSMSGID_HANDSET_BATT_INFO,
                           (uint8_t)(batt_voltage >> 24),
                           (uint8_t)(batt_voltage >> 16),
                           (uint8_t)(batt_voltage >> 8),
                           (uint8_t)(batt_voltage >> 0),
                           (uint8_t)eeprom_storage.batt_voltage_scale,
                           (uint8_t)eeprom_storage.batt_voltage_warning};
    websocket_send_bin(batt_info, sizeof(batt_info), (AsyncWebSocketClient *)client);
}

void ExpresslrsMsp::battery_voltage_parse(uint8_t const scale, uint8_t const warning, void * client)
{
    if (50 <= scale && scale <= 150 && eeprom_storage.batt_voltage_scale != scale) {
        eeprom_storage.batt_voltage_scale = scale;
        eeprom_storage.markDirty();
    }
    if (10 <= warning && warning <= 100 && eeprom_storage.batt_voltage_warning != warning) {
        eeprom_storage.batt_voltage_warning = warning;
        eeprom_storage.markDirty();

        batt_voltage_warning_limit = ((warning * BATT_NOMINAL_mV) / 100);
    }
}

void ExpresslrsMsp::batt_voltage_init(void)
{
    batt_voltage_warning_limit = ((eeprom_storage.batt_voltage_warning * BATT_NOMINAL_mV) / 100);
    batt_voltage_dead_limit = ((BATT_DEAD_DEFAULT * BATT_NOMINAL_mV) / 100);
}

void ExpresslrsMsp::batt_voltage_measure(void)
{
    uint32_t ms = millis();
    if (eeprom_storage.batt_voltage_interval <= (uint32_t)(ms - batt_voltage_meas_last_ms)) {
        int adc = analogRead(A0);
        batt_voltage = ADC_VOLT(adc);
        batt_voltage = (batt_voltage * eeprom_storage.batt_voltage_scale) / 100;

        battery_voltage_report();

        uint32_t brightness = (batt_voltage * 255) / BATT_NOMINAL_mV;
        if (255 < brightness) {
            batt_voltage_warning_timeout = 5000;
            brightness = 255;
        } else if (batt_voltage <= batt_voltage_dead_limit) {
            batt_voltage_warning_timeout = 500;
            brightness = 0;
        } else if (batt_voltage <= batt_voltage_warning_limit) {
            batt_voltage_warning_timeout = 500 + 5 * (int32_t)(batt_voltage - batt_voltage_warning_limit);
            if ((int32_t)batt_voltage_warning_timeout < 500) {
                batt_voltage_warning_timeout = 500;
            }
        }

        led_brightness_set(brightness);

        batt_voltage_meas_last_ms = ms;
    }

    if (batt_voltage && (batt_voltage <= batt_voltage_warning_limit) &&
        (batt_voltage_warning_timeout <= (ms - batt_voltage_warning_last_ms))) {
#ifdef WS2812_PIN
        if (!batt_voltage_last_bright) {
            batt_voltage_last_bright = led_brightness_get();
            led_brightness_set(0, 1);
        } else {
            led_brightness_set(batt_voltage_last_bright);
            led_set();
            batt_voltage_last_bright = 0;
        }
#endif
        // 400Hz, 20ms
        buzzer_beep(400, 20, 0);

        batt_voltage_warning_last_ms = ms;
    }
}

#else
#define batt_voltage_init()
#define batt_voltage_measure()
#endif

void ExpresslrsMsp::init(void)
{
    _handler.markPacketFree();
    /* Reset values */
    settings_rate = 1;
    settings_power = 4;
    settings_power_max = 8;
    settings_tlm = 7;
    settings_region = 255;
    settings_valid = 0;
#if CONFIG_HANDSET
    handset_num_switches = 6;
    handset_num_aux = 5;
    handset_mixer_ok = 0;
    handset_adjust_ok = 0;
    batt_voltage = 0;
    batt_voltage_warning_last_ms = millis();
#endif

    batt_voltage_init();
}

void ExpresslrsMsp::syncSettings(void * client, bool const force)
{
    // Send settings
    send_current_values(client);
#if CONFIG_HANDSET
    // delay(10);
    HandsetConfigGet(client, force);
    // delay(10);
    battery_voltage_report(client);
#endif
}

String ExpresslrsMsp::syncSettingsJson(void)
{
    String json = "{";
    if (settings_valid) {
        json += "\"region\":" + String(settings_region);
        json += ",\"rate\":" + String(settings_rate);
        json += ",\"power\":" + String(settings_power);
        json += ",\"power_max\":" + String(settings_power_max);
        json += ",\"telemetry\":" + String(settings_tlm);
        json += ",\"vtxfreq\":" + String(eeprom_storage.vtx_freq);
    }
#if CONFIG_HANDSET
    if (handset_mixer_ok) {
        json += ",\"mixer\":{";
        json += "\"total\":" + String(handset_num_aux);
        json += ",\"aux\":" + String(handset_num_aux + TX_NUM_ANALOGS);
        json += ",\"switch\":" + String(handset_num_switches);
        json += ",\"config\":[";
        for (uint8_t iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
            if (TX_NUM_MIXER == mixer[iter].index)
                continue;  // Ignore all invalid configs
            if (iter)
                json += ",";
            json += "{";
            json += "\"index\":" + String(mixer[iter].index);
            json += ",\"inv\":" + String(mixer[iter].inv);
            json += ",\"scale\":" + String(mixer[iter].scale);
            json += "}";
        }
        json += "]}";
    }
    if (handset_adjust_ok) {
        json += ",\"gimbals\":[";
        for (uint8_t iter = 0; iter < ARRAY_SIZE(gimbals); iter++) {
            if (iter)
                json += ",";
            json += "{";
            json += "\"min\":" + String(gimbals[iter].low);
            json += ",\"mid\":" + String(gimbals[iter].mid);
            json += ",\"max\":" + String(gimbals[iter].high);
            json += "}";
        }
        json += "]";
    }
#endif
    json += "}";
    return json;
}

void ExpresslrsMsp::sendSettingsJson(void * client)
{
    String data = syncSettingsJson();
    async_event_send(data, "elrs_settings", (AsyncEventSourceClient *)client);
    async_event_send(m_version_info, "elrs_version", (AsyncEventSourceClient *)client);
}

int ExpresslrsMsp::parse_data(uint8_t const chr)
{
    if (_handler.processReceivedByte(chr)) {
        /* Process the received MSP message */
        uint8_t forward = true;
        String info = "MSP received: ";
        mspPacket_t & msp_in = _handler.getPacket();
        if (msp_in.type == MSP_PACKET_V1_ELRS /*&& msp_in.type & MSP_ELRS_INT*/) {
            uint8_t * payload = (uint8_t *)msp_in.payload;
            forward = false;
            switch (msp_in.function) {
                case ELRS_INT_MSP_PARAMS: {
                    msp_in.payloadIterator = 0;
                    forward = true;
                    info += "ELRS params";
                    settings_rate = msp_in.readByte();
                    settings_tlm = msp_in.readByte();
                    settings_power = msp_in.readByte();
                    settings_power_max = msp_in.readByte();
                    settings_region = msp_in.readByte();
                    settings_valid = 1;

                    // Read ELRS code version sha
                    m_version_info = "";
                    for (uint8_t iter = 0; (iter < 6) && !msp_in.iterated(); iter++)
                        m_version_info += String(msp_in.readByte(), HEX);
                    if (msp_in.readByte() == '!')
                        m_version_info += "-dirty";

                    send_current_values(NULL);
                    break;
                }
                case ELRS_INT_MSP_DEV_INFO: {
                    info += "device info";
                    // Respond with the VTX config
                    if (payload[0] == 1 && payload[1] < 3)
                        sendVtxFrequencyToSerial(eeprom_storage.vtx_freq);
                    break;
                }
                case ELRS_INT_MSP_ESPNOW_UPDATE: {
                    info += "[ERROR] ESP-NOW Update from TX co-mcu!";
                    break;
                }
#if CONFIG_HANDSET
                case ELRS_HANDSET_CALIBRATE: {
                    info += "CALIBRATE error";
                    handleHandsetCalibrateResp(payload);
                    break;
                }
                case ELRS_HANDSET_MIXER: {
                    info += "MIXER config";
                    handleHandsetMixerResp(payload);
                    handset_mixer_ok = 1;
                    break;
                }
                case ELRS_HANDSET_ADJUST: {
                    info += "ADJUST config";
                    handleHandsetAdjustResp(payload);
                    handset_adjust_ok = 1;
                    break;
                }
                case ELRS_HANDSET_TLM_LINK_STATS: {
                    info = "";
                    handleHandsetTlmLnkStatsAndBatt(payload);
                    break;
                }
                case ELRS_HANDSET_TLM_GPS: {
                    info = "";
                    handleHandsetTlmGps(payload);
                    break;
                }
#endif /* CONFIG_HANDSET */
                default:
                    info += "UNKNOWN";
                    break;
            };
        } else {
            info = "DL MSP rcvd. func: ";
            info += String(msp_in.function, HEX);
            info += ", size: ";
            info += msp_in.payloadSize;
        }

        if (info.length())
            websocket_send_txt(info);
        if (forward)
            espnow_send_msp(msp_in);

        _handler.markPacketFree();
    } else if (!_handler.mspOngoing()) {
        return -1;
    }
    return 0;
}

int ExpresslrsMsp::parse_command(char * cmd, size_t len, void * client)
{
    return -1;
}

int ExpresslrsMsp::parse_command(websoc_bin_hdr_t const * const cmd, size_t const len, void * client)
{
    /* No payload */
    if (WSMSGID_ELRS_SETTINGS == cmd->msg_id) {
        return send_current_values(client);
    }

    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send_txt(settings_out, (AsyncWebSocketClient *)client);
    }

    switch (cmd->msg_id) {
        case WSMSGID_ELRS_DOMAIN: {
            uint8_t buff[] = {ELRS_CMD_DOMAIN, cmd->payload[0]};
            SettingsWrite(buff, sizeof(buff));
            break;
        }
        case WSMSGID_ELRS_RATE: {
            uint8_t buff[] = {ELRS_CMD_RATE, cmd->payload[0]};
            SettingsWrite(buff, sizeof(buff));
            break;
        }
        case WSMSGID_ELRS_POWER: {
            uint8_t buff[] = {ELRS_CMD_POWER, cmd->payload[0]};
            SettingsWrite(buff, sizeof(buff));
            break;
        }
        case WSMSGID_ELRS_TLM: {
            uint8_t buff[] = {ELRS_CMD_TLM, cmd->payload[0]};
            SettingsWrite(buff, sizeof(buff));
            break;
        }
        case WSMSGID_ELRS_RF_POWER_TEST: { // Test feature
            uint8_t buff[] = {ELRS_CMD_RF_POWER_TEST, cmd->payload[0]};
            SettingsWrite(buff, sizeof(buff));
            break;
        }

#if CONFIG_HANDSET
        case WSMSGID_HANDSET_MIXER: {
            handleHandsetMixer(cmd->payload, len);
            break;
        }
        case WSMSGID_HANDSET_CALIBRATE: {
            handleHandsetCalibrate(cmd->payload);
            break;
        }
        case WSMSGID_HANDSET_ADJUST: {
            handleHandsetAdjust(cmd->payload);
            break;
        }
        case WSMSGID_HANDSET_RELOAD: {
            HandsetConfigGet(client, 1);
            break;
        }
        case WSMSGID_HANDSET_SAVE: {
            HandsetConfigSave(client);
            break;
        }
        case WSMSGID_HANDSET_BATT_CONFIG: {
            battery_voltage_parse(cmd->payload[0], cmd->payload[1], client);
            break;
        }
#endif

        case WSMSGID_VIDEO_FREQ: {
            uint16_t const freq = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            handleVtxFrequency(freq, client);
            break;
        }

        default:
            return -1;
    }
    return 0;
}

int ExpresslrsMsp::handle_received_msp(mspPacket_t & msp_in)
{
    if (msp_in.type == MSP_PACKET_V2_COMMAND) {
        if (msp_in.function == MSP_VTX_SET_CONFIG) {
            uint16_t freq = msp_in.payload[1];
            freq <<= 8;
            freq += msp_in.payload[0];
            if (3 <= msp_in.payloadSize) {
                // power
            }
            if (4 <= msp_in.payloadSize) {
                // pitmode
            }
            /* Pass command to elrs */
            sendVtxFrequencyToSerial(freq);
            /* Infrom web clients */
            sendVtxFrequencyToWebsocket(freq);
            return 0;
        } else if (msp_in.function == ELRS_INT_MSP_ESPNOW_UPDATE) {
            // TODO: handle channel etc!
        }
    }
    return -1;
}

void ExpresslrsMsp::loop(void)
{
    batt_voltage_measure();

    if (msp_send_save_requested_ms && (MSP_SAVE_DELAY_MS <= (millis() - msp_send_save_requested_ms))) {
        /* send write command and clear the flag */
        MspWrite(NULL, 0, MSP_EEPROM_WRITE);
        msp_send_save_requested_ms = 0;
    }
}
