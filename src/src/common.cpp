#include "common.h"
#include "platform.h"
#include "utils.h"
#include "FHSS.h"
#include "crc.h"
#include "rc_channels.h"
#include "targets.h"
#include "helpers.h"
#include "debug_elrs.h"

#if RADIO_SX127x
#include "LoRa_SX127x.h"
#endif
#if RADIO_SX128x
#include "SX1280.h"
#endif

uint8_t current_rate_config;
const expresslrs_mod_settings_t *ExpressLRS_currAirRate;

static expresslrs_mod_settings_t * current_settings;

//
// https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R000000HUhK/6T9Vdb3_ldnElA8drIbPYjs1wBbhlWUXej8ZMXtZXOM
//
#if RADIO_SX128x
#if RADIO_SX128x_FLRC
static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_128x_FLRC[] = {
    /* 500Hz */
    {RADIO_FLRC, SX1280_FLRC_BR_0_325_BW_0_3, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, 2000, 500, TLM_RATIO_1_128, FHSS_2, OSD_MODE_500Hz, 32, 1000, 750, 250000u}, // 0.78ms
};
#endif

static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_128x[] = {
    // NOTE! Preamble len is calculate MANT*2^EXP when MANT is bits [3:0] and EXP is bits [7:4]
#if RADIO_SX128x_BW800
    /* 500Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, 500, TLM_RATIO_1_128, FHSS_2, OSD_MODE_500Hz, 0b01100 /*12*/, 1000, 750, 250000u}, // 1.51ms
    /* 250Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, 250, TLM_RATIO_1_64,  FHSS_1, OSD_MODE_250Hz, 0b01110 /*14*/, 1000, 750, 250000u}, // 3.33ms
    /* 125Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 8000, 125, TLM_RATIO_1_32,  FHSS_1, OSD_MODE_125Hz, 0b11001 /*18*/, 1000, 1200, 500000u}, // 6.82ms
    /* 50Hz */
    {RADIO_LORA, SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 20000, 50, TLM_RATIO_1_16,  FHSS_1, OSD_MODE_50Hz,  0b11010 /*20*/, 1000, 1500, 750000u}, // 13.32ms
#else // 1600MHz BW
    /* 800Hz */
    //{RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 1250, 800, TLM_RATIO_1_128, FHSS_4, OSD_MODE_800Hz, 0b01100 /*12*/, 1000, 1500, 250000u}, // 0.68ms
    /* 500Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_6, 2000, 500, TLM_RATIO_1_128, FHSS_2, OSD_MODE_500Hz, 0b01100 /*12*/, 1000, 1500, 250000u}, // 1.35ms
    /* 250Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 4000, 250, TLM_RATIO_1_64,  FHSS_1, OSD_MODE_250Hz, 0b01110 /*14*/, 1000, 1500, 250000u}, // 3.10ms
    /* 125Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 8000, 125, TLM_RATIO_1_32,  FHSS_1, OSD_MODE_125Hz, 0b11010 /*20*/, 1000, 2000, 500000u},
    /* 50Hz */
    {RADIO_LORA, SX1280_LORA_BW_1600, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 20000, 50, TLM_RATIO_1_16,  FHSS_1, OSD_MODE_50Hz,  0b11010 /*20*/, 1000, 2500, 750000u},
#endif // RADIO_SX128x_BW800
};
#endif /* RADIO_SX128x */

#if RADIO_SX127x
static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig_127x[] = {
    /* 200Hz */
    {RADIO_LORA, BW_500_00_KHZ, SF_6, CR_4_7, 5000, 200, TLM_RATIO_1_64, FHSS_1, OSD_MODE_200Hz, 8, 1000, 1000, 250000u}, // 4.38ms
    /* 100Hz */
    {RADIO_LORA, BW_500_00_KHZ, SF_7, CR_4_8, 10000, 100, TLM_RATIO_1_32, FHSS_1, OSD_MODE_100Hz, 8, 1000, 1500, 500000u}, // 9.28ms
    /* 50Hz */
    {RADIO_LORA, BW_500_00_KHZ, SF_8, CR_4_8, 20000, 50, TLM_RATIO_1_16, FHSS_1, OSD_MODE_50Hz, 8, 1000, 2000, 750000u}, // 18.56ms

#if RATE_ENABLED_25Hz
    {RADIO_LORA, BW_500_00_KHZ, SF_9, CR_4_8, 40000, 25, TLM_RATIO_1_8, FHSS_1, OSD_MODE_25Hz, 10, 6000, 2500, 0},
#if RATE_ENABLED_4Hz
    {RADIO_LORA, BW_500_00_KHZ, SF_12, CR_4_8, 250000, 4, TLM_RATIO_NO_TLM, FHSS_1, OSD_MODE_4Hz, 10, 6000, 2500, 0},
#endif /* RATE_ENABLED_4Hz */
#endif /* RATE_ENABLED_25Hz */
};
#endif /* RADIO_SX127x */


static expresslrs_mod_settings_t* get_air_rate_config(uint8_t type)
{
    switch (type) {
#if RADIO_SX127x
        case RADIO_TYPE_127x:
            return ExpressLRS_AirRateConfig_127x;
#endif
#if RADIO_SX128x
        case RADIO_TYPE_128x:
            return ExpressLRS_AirRateConfig_128x;
#if RADIO_SX128x_FLRC
        case RADIO_TYPE_128x_FLRC:
            return ExpressLRS_AirRateConfig_128x_FLRC;
#endif
#endif
        default:
            return NULL;
    }
}

const expresslrs_mod_settings_t *get_elrs_airRateConfig(uint8_t rate)
{
    if (get_elrs_airRateMax() <= rate || !current_settings)
        return NULL;
    return &current_settings[rate];
}

uint8_t get_elrs_airRateIndex(void * current)
{
    if (!current_settings || !current)
        return 0;
    return ((uintptr_t)current - (uintptr_t)current_settings) / sizeof(expresslrs_mod_settings_t);
}

uint8_t get_elrs_airRateMax(void)
{
#if RADIO_SX127x
    if (current_settings == ExpressLRS_AirRateConfig_127x)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_127x);
#endif
#if RADIO_SX128x
    if (current_settings == ExpressLRS_AirRateConfig_128x)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_128x);
#if RADIO_SX128x_FLRC
    if (current_settings == ExpressLRS_AirRateConfig_128x_FLRC)
        return ARRAY_SIZE(ExpressLRS_AirRateConfig_128x_FLRC);
#endif
#endif
    return 0;
}

uint8_t get_elrs_current_radio_type(void)
{
#if RADIO_SX127x
    if (current_settings == ExpressLRS_AirRateConfig_127x)
        return RADIO_TYPE_127x;
#endif
#if RADIO_SX128x
    if (current_settings == ExpressLRS_AirRateConfig_128x)
        return RADIO_TYPE_128x;
#if RADIO_SX128x_FLRC
    if (current_settings == ExpressLRS_AirRateConfig_128x_FLRC)
        return RADIO_TYPE_128x_FLRC;
#endif
#endif
    return RADIO_TYPE_MAX;
}

#ifndef MY_UID
#error "UID is mandatory!"
#endif

uint8_t getSyncWord(void)
{
    uint8_t UID[6] = {MY_UID};
    return CalcCRC8len(UID, sizeof(UID));
}

uint32_t getSyncWord32(void)
{
    uint8_t UID[6] = {MY_UID};
    return CalcCRC32(UID, sizeof(UID));
}

typedef struct {
    int rst, dio0, dio1, dio2;
    int busy, txen, rxen, pactrl;
    uint32_t nss;
    RadioInterface* radio_if;
    const char * str;
} RadioParameters_t;

#if RADIO_SX127x
SX127xDriver DRAM_FORCE_ATTR Radio127x(OTA_PACKET_SIZE);
#endif
#if RADIO_SX128x
SX1280Driver DRAM_FORCE_ATTR Radio128x(OTA_PACKET_SIZE);
#endif
#if !RADIO_SX127x && !RADIO_SX128x
#error "NO VALID RADIO!"
#endif
RadioInterface DRAM_FORCE_ATTR *Radio;

static RadioParameters_t DRAM_FORCE_ATTR RadioType[] = {
#if RADIO_SX127x
    {GPIO_PIN_RST_127x, GPIO_PIN_DIO0_127x, GPIO_PIN_DIO1_127x, GPIO_PIN_DIO2_127x,
     UNDEF_PIN, GPIO_PIN_TXEN_127x, GPIO_PIN_RXEN_127x, GPIO_PIN_PAEN_127x,
     GPIO_PIN_NSS_127x,
     &Radio127x, "SX127x"},
#endif
#if RADIO_SX128x
    {GPIO_PIN_RST_128x, GPIO_PIN_DIO0_128x, GPIO_PIN_DIO1_128x, GPIO_PIN_DIO2_128x,
     GPIO_PIN_BUSY, GPIO_PIN_TXEN_128x, GPIO_PIN_RXEN_128x, GPIO_PIN_PAEN_128x,
     GPIO_PIN_NSS_128x,
     &Radio128x, "SX128x"},
#endif
};

static_assert(0 < ARRAY_SIZE(RadioType), "INVALID CONFIG");

static RadioParameters_t* get_radio_type_cfg(uint8_t type)
{
#if RADIO_SX127x && RADIO_SX128x
    switch (type) {
        case RADIO_TYPE_127x:
            return &RadioType[RADIO_TYPE_127x];
        case RADIO_TYPE_128x:
        case RADIO_TYPE_128x_FLRC:
            return &RadioType[RADIO_TYPE_128x];
    }
#endif
    return &RadioType[0];
}

uint8_t common_config_get_radio_type(uint8_t mode)
{
    switch (mode) {
#if RADIO_SX127x
        case RADIO_RF_MODE_915_AU_FCC:
        case RADIO_RF_MODE_868_EU:
        case RADIO_RF_MODE_433_AU_EU:
            return RADIO_TYPE_127x;
#endif
#if RADIO_SX128x
        case RADIO_RF_MODE_2400_ISM:
        case RADIO_RF_MODE_2400_ISM_500Hz:
            return RADIO_TYPE_128x;
#if RADIO_SX128x_FLRC
        case RADIO_RF_MODE_2400_ISM_FLRC:
            return RADIO_TYPE_128x_FLRC;
#endif
#endif
        default:
            break;
    };
    return RADIO_TYPE_MAX;
}

RadioInterface* common_config_radio(uint8_t type)
{
    RadioParameters_t * config;
    RadioInterface* radio;
    uint8_t iter;

    if (!current_settings) {
        /* Init both radio first */
        for (iter = 0; iter < ARRAY_SIZE(RadioType); iter++) {
            config = &RadioType[iter];
            radio = config->radio_if;
            radio->SetPins(config->rst, config->dio0, config->dio1, config->dio2,
                           config->busy, config->txen, config->rxen, config->nss,
                           config->pactrl);
            radio->SetSyncWord(getSyncWord());
            radio->SetSyncWordLong(getSyncWord32());
            //radio->End();
        }
    }

    config = get_radio_type_cfg(type);
    radio = config->radio_if;

    if (0 > radio->Begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI)) {
        DEBUG_PRINTF("[ERROR] Radio config failed!\n");
        return NULL;
    }

    DEBUG_PRINTF("RF: %s\n", config->str);
    FHSS_init(type);

    current_settings = get_air_rate_config(type);
    return radio;
}
