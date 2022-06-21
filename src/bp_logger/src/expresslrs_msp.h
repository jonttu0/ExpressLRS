#pragma once

#include "platform.h"
#include "msp.h"
#include <stdint.h>


class ExpresslrsMsp
{
public:
    ExpresslrsMsp(CtrlSerial *serial) : _serial(serial) {}
    ~ExpresslrsMsp() {}

    void init(void);

    void syncSettings(int num);

    int parse_data(uint8_t const chr);
    int parse_command(char * cmd, size_t len, int num);

    int handle_received_msp(mspPacket_t &msp_in);

    void loop(void);

private:
    CtrlSerial * _serial;

    MSP _handler;
    mspPacket_t msp_out;

    volatile uint32_t msp_send_save_requested_ms;

    uint8_t settings_rate;
    uint8_t settings_power, settings_power_max;
    uint8_t settings_tlm;
    uint8_t settings_region;
    uint8_t settings_valid;

#if CONFIG_HANDSET
    /* Handset specific data */
    struct gimbal_limit gimbals[TX_NUM_ANALOGS];
    struct mixer mixer[TX_NUM_MIXER];
    uint8_t handset_num_switches;
    uint8_t handset_num_aux;
    uint8_t handset_mixer_ok;
    uint8_t handset_adjust_ok;
#endif

    void SettingsWrite(uint8_t * buff, uint8_t len);
    void handleSettingRate(const char * input, int num = -1);
    void handleSettingPower(const char * input, int num = -1);
    void handleSettingTlm(const char * input, int num = -1);
    void handleSettingRfPwr(const char * input, int num = -1);
    void handleSettingRfModule(const char * input, int num = -1);
    void handleSettingDomain(const char * input, int num = -1);
    void SettingsGet(uint8_t wsnum);
    void handleVtxFrequency(const char * input, int num = -1);
    void sendVtxFrequencyToSerial(uint16_t freq);

#if CONFIG_HANDSET
    void handleHandsetCalibrate(const char * input);
    void handleHandsetCalibrateResp(uint8_t * data, int num = -1);
    void handleHandsetMixer(const char * input, size_t length);
    void handleHandsetMixerResp(uint8_t * data, int num = -1);
    void handleHandsetAdjust(const char * input);
    void handleHandsetAdjustResp(uint8_t * data, int num = -1);
    void HandsetConfigGet(uint8_t wsnum, uint8_t force = 0);
    void HandsetConfigSave(uint8_t wsnum);
    void handleHandsetTlmLnkStats(uint8_t * data);
    void handleHandsetTlmBattery(uint8_t * data);
    void handleHandsetTlmGps(uint8_t * data);

    void battery_voltage_report(int num = -1);
    void battery_voltage_parse(const char * input, int num = -1);
    void batt_voltage_init(void);
    void batt_voltage_measure(void);
#endif
};
