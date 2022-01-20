#pragma once

#include "RadioInterface.h"
#include <stdint.h>

//#define RATE_ENABLED_25Hz 1
//#define RATE_ENABLED_4Hz  1

#define BUTTON_RESET_INTERVAL_RX  4000 // Hold button for 4 sec to reboot RX
#define WEB_UPDATE_PRESS_INTERVAL 2000 // hold button for 2 sec to enable webupdate mode

#ifndef RADIO_SX128x_FLRC
#define RADIO_SX128x_FLRC 0
#endif


typedef enum
{
    STATE_search_iteration_done = -2, // e.g. RX could start SW upgrade
    STATE_fw_upgrade = -1,
    STATE_disconnected = 0,
    STATE_lost,
    STATE_tentative,
    STATE_connected,
} connectionState_e;
extern connectionState_e connectionState;


// *******************************************************************'
enum {
    RADIO_RF_MODE_915_AU_FCC = 0,
    RADIO_RF_MODE_868_EU,
    RADIO_RF_MODE_433_AU_EU,
    RADIO_RF_MODE_2400_ISM,
    RADIO_RF_MODE_2400_ISM_500Hz,
    RADIO_RF_MODE_2400_ISM_FLRC,
    RADIO_RF_MODE_MAX, // 0x1f = 32 values!
    ExLRS_RF_MODE_FLRC      = 0x80,
    ExLRS_RF_MODE_HANDSET   = 0x40,
    ExLRS_RF_MODE_DUAL      = 0x80,
    ExLRS_RF_MODE_MASK      = 0x3F,
    RADIO_RF_MODE_INVALID   = 0xFF,
};

enum {
    RADIO_TYPE_127x,
    RADIO_TYPE_128x,
    RADIO_TYPE_128x_FLRC,
    RADIO_TYPE_MAX
};

uint8_t common_config_get_radio_type(uint8_t mode);
RadioInterface* common_config_radio(uint8_t type);

extern RadioInterface DRAM_FORCE_ATTR *Radio;

// *******************************************************************'
enum
{
    RADIO_LORA = 0,
    RADIO_FLRC,
};

enum
{
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7,
    TLM_RATIO_MAX,
    TLM_RATIO_DEFAULT = 0xff,
};

enum
{
    FHSS_1 = 1,
    FHSS_2 = 2,
    FHSS_4 = 4,
    FHSS_8 = 8,
};

enum
{
    OSD_MODE_4Hz = 0,
    OSD_MODE_25Hz,
    OSD_MODE_50Hz,
    OSD_MODE_100Hz,
    OSD_MODE_125Hz,
    OSD_MODE_200Hz,
    OSD_MODE_250Hz,
    OSD_MODE_500Hz,
    OSD_MODE_500Hz_FLRC,
    OSD_MODE_800Hz,
};

typedef enum
{
    RF_DOWNLINK_INFO = 0,
    RF_UPLINK_INFO = 1,
    RF_AIRMODE_PARAMETERS = 2
} expresslrs_tlm_header_e;

#define RATE_DEFAULT         0


typedef struct expresslrs_mod_settings_s
{
    uint8_t pkt_type, bw, sf, cr;
    uint32_t interval;       // interval in us seconds that corresponds to that frequnecy
    uint16_t rate;           // rate in hz
    uint8_t TLMinterval;     // every X packets is a response TLM packet, should be a power of 2
    uint8_t FHSShopInterval; // every X packets we hope to a new frequnecy.
    uint8_t rate_osd_num;
    uint16_t PreambleLen;
    uint16_t connectionLostTimeout;
    uint16_t syncSearchTimeout;
    uint32_t syncInterval;
} expresslrs_mod_settings_t;

extern const expresslrs_mod_settings_t *ExpressLRS_currAirRate;
extern uint8_t current_rate_config;

const expresslrs_mod_settings_t *get_elrs_airRateConfig(uint8_t rate);
uint8_t get_elrs_airRateIndex(void * current);
uint8_t get_elrs_airRateMax(void);
uint8_t get_elrs_current_radio_type(void);
static inline uint8_t get_elrs_airRateOsd(void) {
    return ExpressLRS_currAirRate->rate_osd_num;
}

#define TLMratioEnumToValue(TLM) (256U >> (TLM))

#if defined(RX_MODULE)
void forced_stop(void);
#endif /* RX_MODULE */

uint8_t my_uid_crc8(void);
uint32_t my_uid_crc32(void);

int8_t validate_bl_indentifier(const uint8_t * id);
