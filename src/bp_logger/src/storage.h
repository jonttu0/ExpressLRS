#pragma once

#include <stdint.h>

#define LOGGER_STORAGE_VERSION  0x11220001


struct storage {
    uint32_t versionNumber;
    uint32_t batt_voltage_scale;
    uint32_t batt_voltage_interval;
    uint32_t batt_voltage_warning;

    uint16_t vtx_freq;

    void setup();
    void update();

    void load();
    void save();
    void markDirty();

    void initDefaults();
};

extern struct storage eeprom_storage;
