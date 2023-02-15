#pragma once

#include "platform.h"
#include "msp.h"
#include "main.h"
#include <stdint.h>


class ExpresslrsMsp
{
public:
    ExpresslrsMsp(CtrlSerial *serial) : _serial(serial) {}
    ~ExpresslrsMsp() {}

    void init(void);

    void syncSettings(void * client);

    int parse_data(uint8_t const chr);
    int parse_command(char * cmd, size_t len, void * client);
    int parse_command(websoc_bin_hdr_t const * const cmd, size_t len, void * client);

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

    int send_current_values(void * client = NULL);

    void SettingsWrite(uint8_t * buff, uint8_t len);
    void MspWrite(uint8_t * buff, uint8_t len, uint8_t function);

    void handleVtxFrequency(uint16_t freq, void * client = NULL);
    void sendVtxFrequencyToSerial(uint16_t freq, void * client = NULL);
    void sendVtxFrequencyToWebsocket(uint16_t freq);

#if CONFIG_HANDSET
    void handleHandsetCalibrate(uint8_t const * const input);
    void handleHandsetCalibrateResp(uint8_t * data, void * client = NULL);
    void handleHandsetMixer(uint8_t const * const input, size_t length);
    void handleHandsetMixerResp(uint8_t * data, void * client = NULL);
    void handleHandsetAdjust(uint8_t const * const input);
    void handleHandsetAdjustResp(uint8_t * data, void * client = NULL);
    void HandsetConfigGet(void * client, uint8_t force = 0);
    void HandsetConfigSave(void * client);
    void handleHandsetTlmLnkStatsAndBatt(uint8_t * data);
    void handleHandsetTlmGps(uint8_t * data);

    void battery_voltage_report(void * client = NULL);
    void battery_voltage_parse(uint8_t scale, uint8_t warning, void * client = NULL);
    void batt_voltage_init(void);
    void batt_voltage_measure(void);
#endif
};
