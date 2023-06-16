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

    if (versionNumber != LOGGER_STORAGE_VERSION) {
        initDefaults();
    }
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
    if (versionNumber < 0x11220001) {
        batt_voltage_scale = 100;
        batt_voltage_interval = 5000;
        batt_voltage_warning = BATT_WARN_DEFAULT;

        vtx_freq = 0;

        espnow_initialized = 0;
        espnow_clients_count = 0;
    }
    if (versionNumber < 0x11220002) {
        // mighrate to newer version
        wifi_nets_initialized = 0;
        memset(wifi_nets, 0, sizeof(wifi_nets));
    }
    if (versionNumber < 0x11220003) {
        memset(&laptimer, 0, sizeof(laptimer));
    }
    if (versionNumber < 0x11220004) {
        memset(&laptimer_config, 0, sizeof(laptimer_config));
        laptimer_config.index = -1;
    }
    if (versionNumber < 0x11220005) {
        laptimer_start_stop_aux = UINT32_MAX;
    }
    if (versionNumber < 0x11220006) {
        laptimer_osd_pos.row = 5;
        laptimer_osd_pos.column = 0;
        laptimer_osd_timeout = 3;
    }
    if (versionNumber < 0x11220007) {
        uint8_t default_uid[] = {MY_UID};
        default_uid[0] &= ~0x1; // UID is used as a MAC address
        memcpy(uid, default_uid, sizeof(default_uid));
        recording_start_stop_aux = UINT32_MAX;
    }

    versionNumber = LOGGER_STORAGE_VERSION;
    this->save();
}
