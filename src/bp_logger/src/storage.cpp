#include "storage.h"
#include "handset.h"

#include <Arduino.h>
#include <EEPROM.h>


struct storage eeprom_storage;

static uint32_t last_save;
static bool isDirty;

void storage::setup()
{
    EEPROM.begin(sizeof(*this));
    this->load();
}

void storage::update()
{
    uint32_t now = millis();
    if (isDirty && (1000 <= (now - last_save))) {
        last_save = now;
        this->save();
    }
}

void storage::load()
{
    EEPROM.get(0, *this);

    if (versionNumber != LOGGER_STORAGE_VERSION)
        initDefaults();
    if (!wifi_is_valid()) {
        memset(wifi_nets, 0, sizeof(wifi_nets));
        wifi_nets_initialized = LOGGER_WIFINETS_INIT_KEY;
#if defined(WIFI_SSID) && defined(WIFI_PSK)
        wifi_networks_t * const ptr = &wifi_nets[0];
        const char ssid[] = WIFI_SSID;
        memcpy(ptr->ssid, ssid, sizeof(ssid));
        const char psk[] = WIFI_PSK;
        memcpy(ptr->psk, psk, sizeof(psk));
#endif
    }
    isDirty = false;
}

void storage::save()
{
    EEPROM.put(0, *this);
    EEPROM.commit();
    isDirty = false;
}

void storage::markDirty()
{
    isDirty = true;
}

void storage::initDefaults()
{
    versionNumber = LOGGER_STORAGE_VERSION;

    batt_voltage_scale = 100;
    batt_voltage_interval = 5000;
    batt_voltage_warning = BATT_WARN_DEFAULT;

    vtx_freq = 0;

    espnow_initialized = 0;
    espnow_clients_count = 0;

    wifi_nets_initialized = 0;

    memset(&laptimer, 0, sizeof(laptimer));

    this->save();
}
