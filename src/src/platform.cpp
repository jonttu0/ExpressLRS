#include "platform.h"
#include <string.h>

struct platform_config_454c5208
{
    uint32_t key;
    uint32_t rf_mode;
    struct rf_s {
        uint32_t mode;
        uint32_t power;
        uint32_t tlm;
    } rf[3]; // rf_mode: 0 = SX127x, 1 = SX128x, 2 = SX128x_FLRC

    /* Handset specific data */
    struct gimbal_limit gimbals[TX_NUM_ANALOGS];
    struct mixer mixer[TX_NUM_MIXER];
};
int8_t platform_config_migrate_454c5208(struct platform_config_454c5208 * old, struct platform_config &config)
{
    uint8_t iter;
    if (!old || old->key != 0x454c5208)
        return -1;

    memset(&config, 0, sizeof(config));

    // Do migration
    config.key = ELRS_EEPROM_KEY;
    config.rf_mode = old->rf_mode;
    for (iter = 0; iter < 3; iter++) {
        config.rf[iter].mode = old->rf[iter].mode;
        config.rf[iter].power = old->rf[iter].power;
        config.rf[iter].tlm = old->rf[iter].tlm;
    }
    memcpy(config.gimbals, old->gimbals, sizeof(config.gimbals));
    memcpy(config.mixer, old->mixer, sizeof(config.mixer));

    // Save migrated values
    return platform_config_save(config);
}

int8_t platform_config_migrate(void *oldptr, struct platform_config &config)
{
    if (!oldptr)
        return -1;
    if (config.key == 0x454c5208) {
        return platform_config_migrate_454c5208(
            (struct platform_config_454c5208*)oldptr, config);
    }
    return -1;
}
