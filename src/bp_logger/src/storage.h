#pragma once

#include <stdint.h>

#define LOGGER_STORAGE_VERSION   0x11220007
#define LOGGER_ESPNOW_INIT_KEY   0x4321
#define LOGGER_WIFINETS_INIT_KEY 0x4322

typedef struct {
    uint8_t mac_addr[6];
} espnow_clients_t;

typedef struct {
    char ssid[33];
    char psk[33];
    uint8_t mac[6];
} wifi_networks_t;

typedef struct {
    int8_t index;
    char pilot_name[33];
} laptimer_config_t;

struct storage {
    // --------------- 0x11220001 ---------------------------
    uint32_t versionNumber;
    uint32_t batt_voltage_scale;    // range 50...150
    uint32_t batt_voltage_interval;
    uint32_t batt_voltage_warning;  // range 10...100

    uint16_t vtx_freq;

    /* esp-now clients */
    uint16_t espnow_initialized;
    uint32_t espnow_clients_count;
    espnow_clients_t espnow_clients[16];

    // --------------- 0x11220002 ---------------------------
    /* configured wifi networks*/
    uint32_t wifi_nets_initialized;
    wifi_networks_t wifi_nets[5];

    // --------------- 0x11220003 ---------------------------
    wifi_networks_t laptimer;

    // --------------- 0x11220004 ---------------------------
    laptimer_config_t laptimer_config;

    // --------------- 0x11220005 ---------------------------
    uint32_t laptimer_start_stop_aux;

    // --------------- 0x11220006 ---------------------------
    struct {
        uint16_t row;
        uint16_t column;
    } laptimer_osd_pos;
    uint16_t laptimer_osd_timeout;

    // --------------- 0x11220007 ---------------------------
    uint32_t recording_start_stop_aux;
    uint8_t uid[6];

    void setup();
    void update();

    void load();
    void save();
    void markDirty();

    void initDefaults();

    inline bool wifi_is_valid(void) {
        return (wifi_nets_initialized == LOGGER_WIFINETS_INIT_KEY);
    }
};

extern struct storage eeprom_storage;

static inline bool wifi_is_mac_valid(wifi_networks_t const * const net) {
    bool res = false;
    for (uint8_t iter = 0; !res && iter < sizeof(net->mac); iter++) {
        res = (net->mac[iter] == 0) || (net->mac[iter] == 0xff);
    }
    return !res;
}
static inline bool wifi_is_ssid_valid(wifi_networks_t const * const net) {
    return (!!net->ssid[0]);
}
static inline bool wifi_is_psk_valid(wifi_networks_t const * const net) {
    return (!!net->psk[0]);
}
