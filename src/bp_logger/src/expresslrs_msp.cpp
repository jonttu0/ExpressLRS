#include "expresslrs_msp.h"
#include "main.h"
#include "storage.h"
#include "comm_espnow.h"
#include "handset.h"
#include "led.h"
#include "buzzer.h"
#include "rc_channels.h"


#define MSP_SAVE_DELAY_MS   100

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


int ExpresslrsMsp::send_current_values(int const ws_num)
{
    if (!settings_valid) {
        // Not valid, get latest first
        uint8_t buff[] = {ELRS_CMD_GET, 0};
        SettingsWrite(buff, sizeof(buff));
        return 0;
    }
    uint8_t response[] = {
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
    websocket_send(response, sizeof(response), ws_num);
    return 0;
}

void ExpresslrsMsp::SettingsWrite(uint8_t * buff, uint8_t len)
{
    // Fill MSP packet
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_INT_MSP_PARAMS;
    msp_out.payloadSize = len;
    memcpy((void*)msp_out.payload, buff, len);
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
        memcpy((void*)msp_out.payload, buff, len);
    // Send packet to ELRS
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleVtxFrequency(uint16_t const freq, int const num)
{
    String dbg_info = "Setting vtx freq to: ";
    dbg_info += freq;
    dbg_info += "MHz";

    if (freq == 0)
        return;

    // Send to ELRS
    sendVtxFrequencyToSerial(freq);

    // Send to other esp-now clients
    msp_out.reset();
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = MSP_VTX_SET_CONFIG;
    msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
    msp_out.payload[0] = (freq & 0xff);
    msp_out.payload[1] = (freq >> 8);
    espnow_send_msp(msp_out);

    websocket_send(dbg_info, num);
}

void ExpresslrsMsp::sendVtxFrequencyToSerial(uint16_t const freq)
{
    if (freq == 0)
        return;

    eeprom_storage.vtx_freq = freq;
    eeprom_storage.markDirty();

    uint8_t payload[] = {(uint8_t)(freq & 0xff), (uint8_t)(freq >> 8)};
    //payload[2] = power;
    //payload[3] = (power == 0); // pit mode
    MspWrite(payload, sizeof(payload), MSP_VTX_SET_CONFIG);

    // Request save to make set permanent
    msp_send_save_requested_ms = millis();
}

void ExpresslrsMsp::sendVtxFrequencyToWebsocket(uint16_t const freq)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_VIDEO_FREQ >> 8),
        (uint8_t)WSMSGID_VIDEO_FREQ,
        (uint8_t)(freq >> 8),
        (uint8_t)freq,
    };
    websocket_send(response, sizeof(response));
}


#if CONFIG_HANDSET
void ExpresslrsMsp::handleHandsetCalibrate(uint8_t const * const input)
{
    uint_fast8_t const control_dir = input[0] & 0xF;
    uint_fast8_t const control_axis = (input[0] >> 4) & 0xF;
    uint8_t value = 0;

    if (control_dir == 1)       value = GIMBAL_CALIB_LOW;
    else if (control_dir == 2)  value = GIMBAL_CALIB_MID;
    else if (control_dir == 3)  value = GIMBAL_CALIB_HIGH;
    else return;

    if (control_axis == 0)      value |= GIMBAL_CALIB_L_V;
    else if (control_axis == 1) value |= GIMBAL_CALIB_L_H;
    else if (control_axis == 2) value |= GIMBAL_CALIB_R_V;
    else if (control_axis == 3) value |= GIMBAL_CALIB_R_H;
    else return;

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


void ExpresslrsMsp::handleHandsetCalibrateResp(uint8_t * data, int num)
{
    uint8_t response[] = {
        (uint8_t)(WSMSGID_HANDSET_CALIBRATE >> 8),
        (uint8_t)WSMSGID_HANDSET_CALIBRATE,
    };
    websocket_send(response, sizeof(response));
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


void ExpresslrsMsp::handleHandsetMixerResp(uint8_t * data, int num)
{
    uint8_t * outptr;
    uint8_t response[2 + 4 * ARRAY_SIZE(mixer)];
    uint8_t iter;
    if (data) {
        for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
            if (data[0] < ARRAY_SIZE(mixer)) {
                mixer[data[0]] = (struct mixer){
                    .index=data[1], .inv=data[2], .scale=data[3]};
            }
            data += 4;
        }
        handset_num_switches = *data++;
        handset_num_aux = *data++;
    }

    response[0] = (uint8_t)(WSMSGID_HANDSET_MIXER >> 8);
    response[1] = (uint8_t)WSMSGID_HANDSET_MIXER;
    response[2] = handset_num_aux;
    response[3] = handset_num_switches;
    outptr = &response[4];
    for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
        outptr[0] = iter;
        outptr[1] = mixer[iter].index;
        outptr[2] = mixer[iter].inv;
        outptr[3] = mixer[iter].scale;
        outptr += 4;
    }
    websocket_send(response, sizeof(response), num);
}


void ExpresslrsMsp::handleHandsetAdjust(uint8_t const * const input)
{
    uint_fast8_t const control = input[0] & 0xF;
    uint_fast8_t const control_axis = (input[0] >> 4) & 0xF;

    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;

    if (control == 0)
        msp_out.function = ELRS_HANDSET_ADJUST_MIN;
    else if (control == 1)
        msp_out.function = ELRS_HANDSET_ADJUST_MID;
    else if (control == 2)
        msp_out.function = ELRS_HANDSET_ADJUST_MAX;
    else
        /* not valid cmd */
        return;

    if (control_axis == 0)      msp_out.payload[0] |= GIMBAL_IDX_L1;
    else if (control_axis == 1) msp_out.payload[0] |= GIMBAL_IDX_L2;
    else if (control_axis == 2) msp_out.payload[0] |= GIMBAL_IDX_R1;
    else if (control_axis == 3) msp_out.payload[0] |= GIMBAL_IDX_R2;
    else return;
    msp_out.payload[1] = input[2];
    msp_out.payload[2] = input[1];
    // Send packet
    msp_out.payloadSize = 3;
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleHandsetAdjustResp(uint8_t * data, int num)
{
    if (data) {
        // update the local values
        memcpy(gimbals, data, sizeof(gimbals));
    }
    // Send to web client(s)
    uint8_t info[2 + sizeof(gimbals)];
    info[0] = (uint8_t)(WSMSGID_HANDSET_ADJUST >> 8);
    info[1] = (uint8_t)WSMSGID_HANDSET_ADJUST;
    memcpy(&info[2], gimbals, sizeof(gimbals));
    websocket_send(info, sizeof(info), num);
}

void ExpresslrsMsp::HandsetConfigGet(uint8_t wsnum, uint8_t force)
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
        return;
    }

    //delay(5);
    handleHandsetMixerResp(NULL, wsnum);
    //delay(5);
    handleHandsetAdjustResp(NULL, wsnum);
}

void ExpresslrsMsp::HandsetConfigSave(uint8_t wsnum)
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

void ExpresslrsMsp::handleHandsetTlmLnkStats(uint8_t * data)
{
    LinkStatsLink_t * stats = (LinkStatsLink_t*)data;
    uint8_t info[2 + sizeof(LinkStatsLink_t)];
    // Adjust values
    stats->downlink_RSSI = (uint8_t)(stats->downlink_RSSI - 120);
    // fill the message
    info[0] = (uint8_t)(WSMSGID_HANDSET_TLM_LINK_STATS >> 8);
    info[1] = (uint8_t)WSMSGID_HANDSET_TLM_LINK_STATS;
    memcpy(&info[2], stats, sizeof(LinkStatsLink_t));
    websocket_send(info, sizeof(info));
}

void ExpresslrsMsp::handleHandsetTlmBattery(uint8_t * data)
{
    LinkStatsBatt_t * stats = (LinkStatsBatt_t*)data;
    uint8_t info[10];
    // fill the message
    info[0] = (uint8_t)(WSMSGID_HANDSET_TLM_BATTERY >> 8);
    info[1] = (uint8_t)WSMSGID_HANDSET_TLM_BATTERY;
    info[2] = (uint8_t)(stats->voltage >> 8);
    info[3] = (uint8_t)stats->voltage;
    info[4] = (uint8_t)(stats->current >> 8);
    info[5] = (uint8_t)stats->current;
    info[6] = (uint8_t)(stats->capacity >> 16);
    info[7] = (uint8_t)(stats->capacity >> 8);
    info[8] = (uint8_t)stats->capacity;
    info[9] = (uint8_t)stats->remaining;
    websocket_send(info, sizeof(info));
}

void ExpresslrsMsp::handleHandsetTlmGps(uint8_t * data)
{
    GpsOta_t * stats = (GpsOta_t*)data;
    uint8_t info[2 + sizeof(GpsOta_t)];
    // Convert data
    stats->heading /= 10;
    stats->altitude -= 1000;
    // fill the message
    info[0] = (uint8_t)(WSMSGID_HANDSET_TLM_GPS >> 8);
    info[1] = (uint8_t)WSMSGID_HANDSET_TLM_GPS;
    memcpy(&info[2], stats, sizeof(GpsOta_t));
    websocket_send(info, sizeof(info));
}


//#define ADC_VOLT(X) (((X) * ADC_R2) / (ADC_R1 + ADC_R2))
#define ADC_SCALE (ADC_R1 / ADC_R2)

#define ADC_REF_mV  (1075U * ADC_SCALE)
#define ADC_MAX     1023U
#define ADC_VOLT(X) (((uint32_t)(X) * ADC_REF_mV) / ADC_MAX)

static uint32_t batt_voltage;
static uint32_t batt_voltage_warning_limit;
static uint32_t batt_voltage_dead_limit;

static uint32_t batt_voltage_meas_last_ms;
static uint32_t batt_voltage_warning_last_ms;
static uint32_t batt_voltage_warning_timeout = 5000;
#ifdef WS2812_PIN
static uint8_t batt_voltage_last_bright;
#endif

void ExpresslrsMsp::battery_voltage_report(int num)
{
    uint8_t info[] = {
        (uint8_t)(WSMSGID_HANDSET_BATT_INFO >> 8),
        (uint8_t)WSMSGID_HANDSET_BATT_INFO,
        (uint8_t)(batt_voltage >> 24),
        (uint8_t)(batt_voltage >> 16),
        (uint8_t)(batt_voltage >> 8),
        (uint8_t)(batt_voltage >> 0),
        (uint8_t)eeprom_storage.batt_voltage_scale,
        (uint8_t)eeprom_storage.batt_voltage_warning
    };
    websocket_send(info, sizeof(info), num);
}

void ExpresslrsMsp::battery_voltage_parse(uint8_t const scale, uint8_t const warning, int num)
{
    if (50 <= scale && scale <= 150) {
        eeprom_storage.batt_voltage_scale = scale;
        eeprom_storage.markDirty();
    }
    if (10 <= scale && scale <= 100) {
        eeprom_storage.batt_voltage_warning = scale;
        eeprom_storage.markDirty();

        batt_voltage_warning_limit = ((scale * BATT_NOMINAL_mV) / 100);
    }
}

void ExpresslrsMsp::batt_voltage_init(void)
{
    batt_voltage_warning_limit =
        ((eeprom_storage.batt_voltage_warning * BATT_NOMINAL_mV) / 100);
    batt_voltage_dead_limit =
        ((BATT_DEAD_DEFAULT * BATT_NOMINAL_mV) / 100);
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
            batt_voltage_warning_timeout = 500 +
                5 * (int32_t)(batt_voltage - batt_voltage_warning_limit);
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


void ExpresslrsMsp::syncSettings(int const num)
{
    // Send settings
    send_current_values(num);;
#if CONFIG_HANDSET
    HandsetConfigGet(num);
    delay(5);
    battery_voltage_report(num);
#endif
}


int ExpresslrsMsp::parse_data(uint8_t const chr) {
    if (_handler.processReceivedByte(chr)) {
        /* Process the received MSP message */
        uint8_t forward = true;
        String info = "MSP received: ";
        mspPacket_t &msp_in = _handler.getPacket();
        if (msp_in.type == MSP_PACKET_V1_ELRS) {
            uint8_t * payload = (uint8_t*)msp_in.payload;
            forward = false;
            switch (msp_in.function) {
                case ELRS_INT_MSP_PARAMS: {
                    forward = true;
                    info += "ELRS params";
                    settings_rate = payload[0];
                    settings_tlm = payload[1];
                    settings_power = payload[2];
                    settings_power_max = payload[3];
                    settings_region = payload[4];
                    settings_valid = 1;

                    send_current_values();
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
                    handleHandsetTlmLnkStats(payload);
                    break;
                }
                case ELRS_HANDSET_TLM_BATTERY: {
                    info = "";
                    handleHandsetTlmBattery(payload);
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
        }

        if (info.length())
            websocket_send(info);
        if (forward)
            espnow_send_msp(msp_in);

        _handler.markPacketFree();
    } else if (!_handler.mspOngoing()) {
        return -1;
    }
    return 0;
}


int ExpresslrsMsp::parse_command(char * cmd, size_t len, int const num)
{
    return -1;
}


int ExpresslrsMsp::parse_command(websoc_bin_hdr_t const * const cmd, size_t const len, int const num)
{
    /* No payload */
    if (WSMSGID_ELRS_SETTINGS == cmd->msg_id) {
        return send_current_values(num);
    }

    if (!len) {
        String settings_out = "[INTERNAL ERROR] something went wrong, payload size is 0!";
        websocket_send(settings_out, num);
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
            HandsetConfigGet(num, 1);
            break;
        }
        case WSMSGID_HANDSET_SAVE: {
            HandsetConfigSave(num);
            break;
        }
        case WSMSGID_HANDSET_BATT_CONFIG: {
            battery_voltage_parse(cmd->payload[0], cmd->payload[1], num);
            break;
        }
#endif

        case WSMSGID_VIDEO_FREQ: {
            uint16_t const freq = ((uint16_t)cmd->payload[1] << 8) + cmd->payload[0];
            handleVtxFrequency(freq, num);
            break;
        }

        default:
            return -1;
    }
    return 0;
}


int ExpresslrsMsp::handle_received_msp(mspPacket_t &msp_in)
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
        }
    }
    return -1;
}


void ExpresslrsMsp::loop(void)
{
    batt_voltage_measure();

    if (msp_send_save_requested_ms &&
        (MSP_SAVE_DELAY_MS <= (millis() - msp_send_save_requested_ms))) {
        /* send write command and clear the flag */
        MspWrite(NULL, 0, MSP_EEPROM_WRITE);
        msp_send_save_requested_ms = 0;
    }
}
