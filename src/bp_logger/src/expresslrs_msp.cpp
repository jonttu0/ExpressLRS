#include "expresslrs_msp.h"
#include "main.h"
#include "storage.h"
#include "comm_espnow.h"
#include "handset.h"
#include "led.h"
#include "buzzer.h"
#include "rc_channels.h"


#define MSP_SAVE_DELAY_MS   100


void ExpresslrsMsp::SettingsWrite(uint8_t * buff, uint8_t len)
{
    // Fill MSP packet
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.payloadSize = len;
    msp_out.function = ELRS_INT_MSP_PARAMS;
    memcpy((void*)msp_out.payload, buff, len);
    // Send packet
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleSettingRate(const char * input, int num)
{
    String settings_out = "[INTERNAL ERROR] something went wrong";
    if (input == NULL || *input == '?') {
        settings_out = "ELRS_setting_rates_input=";
        settings_out += settings_rate;
    } else if (*input == '=') {
        input++;
        settings_out = "Setting rate: ";
        settings_out += input;
        // Write to ELRS
        char val = *input;
        uint8_t buff[] = {1, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
        SettingsWrite(buff, sizeof(buff));
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::handleSettingPower(const char * input, int num)
{
    String settings_out = "[INTERNAL ERROR] something went wrong";
    if (input == NULL || *input == '?') {
        settings_out = "ELRS_setting_power_input=";
        settings_out += settings_power;
        settings_out += ",";
        settings_out += settings_power_max;
    } else if (*input == '=') {
        input++;
        settings_out = "Setting power: ";
        settings_out += input;
        // Write to ELRS
        char val = *input;
        uint8_t buff[] = {3, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
        SettingsWrite(buff, sizeof(buff));
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::handleSettingTlm(const char * input, int num)
{
    String settings_out = "[INTERNAL ERROR] something went wrong";
    if (input == NULL || *input == '?') {
        settings_out = "ELRS_setting_tlm_input=";
        settings_out += settings_tlm;
    } else if (*input == '=') {
        input++;
        settings_out = "Setting telemetry: ";
        settings_out += input;
        // Write to ELRS
        char val = *input;
        uint8_t buff[] = {2, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
        SettingsWrite(buff, sizeof(buff));
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::handleSettingRfPwr(const char * input, int num)
{
    String settings_out = "[INTERNAL ERROR] something went wrong";
    if (input == NULL || *input == '?') {
        return;
    } else if (*input == '=') {
        input++;
        settings_out = "Setting RF PWR: ";
        settings_out += input;
        // Write to ELRS
        char val = *input;
        if ('A' <= val)
            val = 10 + (val - 'A');
        else
            val = (val - '0');
        uint8_t buff[] = {6, (uint8_t)val};
        SettingsWrite(buff, sizeof(buff));
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::handleSettingRfModule(const char * input, int num)
{
    String settings_out = "[INTERNAL ERROR] something went wrong";
    if (input == NULL || *input == '?') {
        return;
    } else if (*input == '=') {
        input++;
        settings_out = "Setting RF module: ";
        settings_out += input;
        // Write to ELRS
        char val = *input;
        if ('A' <= val)
            val = 10 + (val - 'A');
        else
            val = (val - '0');
        uint8_t buff[] = {4, (uint8_t)val};
        SettingsWrite(buff, sizeof(buff));
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::handleSettingDomain(const char * input, int num)
{
    String settings_out = "[ERROR] Domain set is not supported!";
    if (input == NULL || *input == '?') {
        settings_out = "ELRS_setting_region_domain=";
        settings_out += settings_region;
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::SettingsGet(uint8_t wsnum)
{
    if (!settings_valid) {
        /* Unknown... reguest update */
        uint8_t buff[] = {0, 0};
        SettingsWrite(buff, sizeof(buff));
    } else {
        /* Valid, just send those to client */
        delay(5);
        handleSettingDomain(NULL, wsnum);
        delay(5);
        handleSettingRate(NULL, wsnum);
        delay(5);
        handleSettingPower(NULL, wsnum);
        delay(5);
        handleSettingTlm(NULL, wsnum);
        delay(5);
    }
    handleVtxFrequency(NULL, wsnum);
}

void ExpresslrsMsp::handleVtxFrequency(const char * input, int num)
{
    String settings_out = "[ERROR] invalid command";
    uint16_t freq;
    //uint8_t power = 1;

    if (input == NULL || *input == '?') {
        settings_out = "ELRS_setting_vtx_freq=";
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

        // Send to ELRS
        sendVtxFrequencyToSerial(freq);

        // Send to other esp-now clients
        mspPacket_t msp_out;
        msp_out.reset();
        msp_out.type = MSP_PACKET_V2_COMMAND;
        msp_out.flags = 0;
        msp_out.function = MSP_VTX_SET_CONFIG;
        msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
        msp_out.payload[0] = (freq & 0xff);
        msp_out.payload[1] = (freq >> 8);
        espnow_send_msp(msp_out);
    }
    websocket_send(settings_out, num);
}

void ExpresslrsMsp::sendVtxFrequencyToSerial(uint16_t const freq)
{
    if (freq == 0)
        return;

    eeprom_storage.vtx_freq = freq;
    eeprom_storage.markDirty();

    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_CMD;
    msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
    msp_out.function = MSP_VTX_SET_CONFIG;
    msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
    msp_out.payload[0] = (freq & 0xff);
    msp_out.payload[1] = (freq >> 8);
    //msp_out.payload[2] = power;
    //msp_out.payload[3] = (power == 0); // pit mode
    // Send packet to ELRS
    MSP::sendPacket(&msp_out, _serial);

    // Request save to make set permanent
    msp_send_save_requested_ms = millis();
}


#if CONFIG_HANDSET
uint8_t char_to_dec(uint8_t const chr)
{
    if ('0' <= chr && chr <= '9') {
        return chr - '0';
    }
    return 0;
}

uint8_t char_to_hex(uint8_t chr)
{
    if ('A' <= chr && chr <= 'F')
        return (10 + (chr - 'A'));
    if ('a' <= chr && chr <= 'f')
        return (10 + (chr - 'a'));
    return char_to_dec(chr);
}

uint8_t char_u8_to_hex(const char * chr)
{
    uint8_t val = char_to_hex(*chr++);
    val <<= 4;
    val += char_to_hex(*chr++);
    return val;
}

uint8_t char_u8_to_dec(const char * chr)
{
    uint8_t val = char_to_dec(*chr++) * 10;
    val += char_to_dec(*chr++);
    return val;
}

uint16_t char_u12_to_hex(const char * chr)
{
    uint16_t val = char_to_hex(*chr++);
    val <<= 4;
    val += char_to_hex(*chr++);
    val <<= 4;
    val += char_to_hex(*chr++);
    return val;
}

void ExpresslrsMsp::handleHandsetCalibrate(const char * input)
{
    uint8_t value = 0;
    // Fill MSP packet
    msp_out.reset();

    value = char_to_hex(input[3]);
    if (value == 1)
        value = GIMBAL_CALIB_LOW;
    else if (value == 2)
        value = GIMBAL_CALIB_MID;
    else if (value == 3)
        value = GIMBAL_CALIB_HIGH;

    if (!strncmp(input, "L1_", 3)) {
        value |= (GIMBAL_CALIB_L_V);
    } else if (!strncmp(input, "L2_", 3)) {
        value |= (GIMBAL_CALIB_L_H);
    } else if (!strncmp(input, "R1_", 3)) {
        value |= (GIMBAL_CALIB_R_V);
    } else if (!strncmp(input, "R2_", 3)) {
        value |= (GIMBAL_CALIB_R_H);
    }

    if (!value)
        return;

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
    String out = "ELRS_handset_calibrate=ERROR!";
    websocket_send(out, num);
}


void ExpresslrsMsp::handleHandsetMixer(const char * input, size_t length)
{
    uint32_t iter, index, scale;
    const char * read = input;
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_HANDSET_MIXER;
    for (iter = 0; iter < ARRAY_SIZE(mixer) && ((size_t)(read - input) < length); iter++) {
        // channel index
        index = char_to_hex(*read++);
        msp_out.addByte(index);
        // channel out
        msp_out.addByte(char_to_hex(*read++));
        // inverted
        msp_out.addByte(char_to_hex(*read++));
        // Scale
        if (index < 4) {
        scale = char_u8_to_dec(read);
        msp_out.addByte(scale);

        String temp = "Gimbal: ";
        temp += index;
        temp += " Scale: ";
        temp += scale;
        websocket_send(temp);

        read += 2;
        }
    }
    msp_out.setIteratorToSize();
    // Send packet
    MSP::sendPacket(&msp_out, _serial);
}


void ExpresslrsMsp::handleHandsetMixerResp(uint8_t * data, int num)
{
    String out = "ELRS_handset_mixer=";
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

    out += handset_num_aux;
    out += ";";
    out += handset_num_switches;
    out += ";";

    for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
        if (iter)
        out += ',';
        out += iter;
        out += ":";
        out += mixer[iter].index;
        out += ":";
        out += mixer[iter].inv;
        out += ":";
        out += mixer[iter].scale;
    }

    websocket_send(out, num);
}


void ExpresslrsMsp::handleHandsetAdjust(const char * input)
{
    const char * temp;
    uint16_t value;
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;

    if (strncmp(&input[3], "min", 3) == 0)
        msp_out.function = ELRS_HANDSET_ADJUST_MIN;
    else if (strncmp(&input[3], "mid", 3) == 0)
        msp_out.function = ELRS_HANDSET_ADJUST_MID;
    else if (strncmp(&input[3], "max", 3) == 0)
        msp_out.function = ELRS_HANDSET_ADJUST_MAX;
    else
        /* not valid cmd */
        return;

    if (!strncmp(input, "L1_", 3))
        msp_out.payload[0] = GIMBAL_IDX_L1;
    else if (!strncmp(input, "L2_", 3))
        msp_out.payload[0] = GIMBAL_IDX_L2;
    else if (!strncmp(input, "R1_", 3))
        msp_out.payload[0] = GIMBAL_IDX_R1;
    else if (!strncmp(input, "R2_", 3))
        msp_out.payload[0] = GIMBAL_IDX_R2;
    else
        /* not valid cmd */
        return;

    temp = strstr((char*)input, "=");
    value = char_u12_to_hex(&temp[1]);
    msp_out.payload[1] = (uint8_t)(value >> 8);
    msp_out.payload[2] = (uint8_t)value;
    // Send packet
    msp_out.payloadSize = 3;
    MSP::sendPacket(&msp_out, _serial);
}

void ExpresslrsMsp::handleHandsetAdjustResp(uint8_t * data, int num)
{
    String out = "ELRS_handset_adjust=";
    uint8_t iter;
    if (data) {
        memcpy(gimbals, data, sizeof(gimbals));
    }

    for (iter = 0; iter < ARRAY_SIZE(gimbals); iter++) {
        struct gimbal_limit * limits = &gimbals[iter];
        out += limits->low;
        out += ":";
        out += limits->mid;
        out += ":";
        out += limits->high;
        out += ";";
    }

    websocket_send(out, num);
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

    delay(5);
    handleHandsetMixerResp(NULL, wsnum);
    delay(5);
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
    String out = "ELRS_tlm_uldl=";
    LinkStatsLink_t * stats = (LinkStatsLink_t*)data;
    // Uplink
    out += "ULQ:"; out += stats->uplink_Link_quality;
    out += ",UR1:"; out += (int8_t)stats->uplink_RSSI_1;
    out += ",UR2:"; out += (int8_t)stats->uplink_RSSI_2;
    out += ",USN:"; out += (int8_t)stats->uplink_SNR;
    //out += ",PWR:"; out += stats->uplink_TX_Power;
    //out += ",MO:"; out += stats->rf_Mode;
    // Downlink
    out += ",DLQ:"; out += stats->downlink_Link_quality;
    out += ",DR1:"; out += (int8_t)(stats->downlink_RSSI - 120);
    out += ",DSN:"; out += (int8_t)stats->downlink_SNR;
    websocket_send(out);
}

void ExpresslrsMsp::handleHandsetTlmBattery(uint8_t * data)
{
    String out = "ELRS_tlm_batt=";
    LinkStatsBatt_t * stats = (LinkStatsBatt_t*)data;
    out += "V:"; out += BYTE_SWAP_U16(stats->voltage);
    out += ",A:"; out += BYTE_SWAP_U16(stats->current);
    out += ",C:"; out += BYTE_SWAP_U32((uint32_t)stats->capacity << 8);
    out += ",R:"; out += stats->remaining;
    websocket_send(out);
}

void ExpresslrsMsp::handleHandsetTlmGps(uint8_t * data)
{
    String out = "ELRS_tlm_gps=";
    GpsOta_t * stats = (GpsOta_t*)data;
    out += "lat:"; out += BYTE_SWAP_U32(stats->latitude);
    out += ",lon:"; out += BYTE_SWAP_U32(stats->longitude);
    out += ",spe:"; out += BYTE_SWAP_U16(stats->speed);
    out += ",hea:"; out += BYTE_SWAP_U16(stats->heading) / 10; // convert to degrees
    out += ",alt:"; out += (int)BYTE_SWAP_U16(stats->altitude) - 1000; // 1000m offset
    out += ",sat:"; out += stats->satellites;
    websocket_send(out);
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
    String out = "ELRS_handset_battery=";
    out += batt_voltage;
    out += ",";
    out += eeprom_storage.batt_voltage_scale;
    out += ",";
    out += eeprom_storage.batt_voltage_warning;

    websocket_send(out, num);
}

void ExpresslrsMsp::battery_voltage_parse(const char * input, int num)
{
    const char * temp = strstr(input, ",");
    uint32_t scale = char_u8_to_hex(input);
    if (50 <= scale && scale <= 150) {
        eeprom_storage.batt_voltage_scale = scale;
        eeprom_storage.markDirty();
    }
    if (temp) {
        scale = char_u8_to_hex(&temp[1]);
        if (10 <= scale && scale <= 100) {
            eeprom_storage.batt_voltage_warning = scale;
            eeprom_storage.markDirty();

            batt_voltage_warning_limit = ((scale * BATT_NOMINAL_mV) / 100);
        }
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
    SettingsGet(num);
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

                    handleSettingDomain(NULL);
                    handleSettingRate(NULL);
                    handleSettingPower(NULL);
                    handleSettingTlm(NULL);
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
    char * temp;
#if CONFIG_HANDSET
    /* Handset specific */
    temp = strstr(cmd, "handset_");
    if (temp) {
        // handset_mixer=
        temp = strstr(cmd, "mixer=");
        if (temp) {
            temp = &temp[6];
            handleHandsetMixer(temp, (len - ((uintptr_t)temp - (uintptr_t)cmd)));
            return 0;
        }

        // handset_calibrate=
        temp = strstr(cmd, "calibrate=");
        if (temp) {
            handleHandsetCalibrate(&temp[10]);
            return 0;
        }

        // handset_adjust_[axe]_[min|max]=val
        temp = strstr(cmd, "_adjust_");
        if (temp) {
            handleHandsetAdjust(&temp[8]);
            return 0;
        }

        temp = strstr(cmd, "_refresh");
        if (temp) {
            HandsetConfigGet(num, 1);
            return 0;
        }

        temp = strstr(cmd, "_save");
        if (temp) {
            HandsetConfigSave(num);
            return 0;
        }

        temp = strstr(cmd, "_battery_config=");
        if (temp) {
            battery_voltage_parse(&temp[16], num);
            return 0;
        }
    } else
#endif // CONFIG_HANDSET
    {
        // ExLRS setting commands
        temp = strstr(cmd, "S_rate");
        if (temp) {
            handleSettingRate(&temp[6], num);
            return 0;
        }
        temp = strstr(cmd, "S_power");
        if (temp) {
            handleSettingPower(&temp[7], num);
            return 0;
        }
        temp = strstr(cmd, "S_telemetry");
        if (temp) {
            handleSettingTlm(&temp[11], num);
            return 0;
        }
        temp = strstr(cmd, "S_vtx_freq");
        if (temp) {
            handleVtxFrequency(&temp[10], num);
            return 0;
        }
        temp = strstr(cmd, "S_rf_pwr");
        if (temp) {
            handleSettingRfPwr(&temp[8], num);
            return 0;
        }
        temp = strstr(cmd, "S_rf_module");
        if (temp) {
            handleSettingRfModule(&temp[11], num);
            return 0;
        }
    }
    return -1;
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
            handleVtxFrequency(NULL);
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
        /* Save send requested, send and clear the flag */
        msp_out.reset();
        msp_out.type = MSP_PACKET_V1_CMD;
        msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
        msp_out.function = MSP_EEPROM_WRITE;
        msp_out.payloadSize = 0; // No params
        MSP::sendPacket(&msp_out, _serial);

        msp_send_save_requested_ms = 0;
    }
}
