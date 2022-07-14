#pragma once

#include <stdint.h>

#define LOGGER_STORAGE_VERSION  0x11220001

#define LOGGER_ESPNOW_INIT_KEY  0x4321

typedef struct {
    uint8_t mac_addr[6];
} espnow_clients_t;


struct storage {
    uint32_t versionNumber;
    uint32_t batt_voltage_scale;    // range 50...150
    uint32_t batt_voltage_interval;
    uint32_t batt_voltage_warning;  // range 10...100

    uint16_t vtx_freq;

    /* esp-now clients */
    uint16_t espnow_initialized;
    uint32_t espnow_clients_count;
    espnow_clients_t espnow_clients[16];

    void setup();
    void update();

    void load();
    void save();
    void markDirty();

    void initDefaults();
};

extern struct storage eeprom_storage;
