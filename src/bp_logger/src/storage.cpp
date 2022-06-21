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

    if (this->versionNumber != LOGGER_STORAGE_VERSION)
        this->initDefaults();
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

    this->save();
}
