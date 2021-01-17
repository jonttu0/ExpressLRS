#ifndef __RC_CHANNELS_H
#define __RC_CHANNELS_H

#include "platform.h"
#include "msp.h"
#include <stdint.h>

#ifndef OTA_PACKET_10B
#define OTA_PACKET_10B      0
#endif

#define OTA_PACKET_DATA     6
#define OTA_PACKET_CRC      2
#define OTA_PACKET_PAYLOAD  (OTA_PACKET_DATA + (OTA_PACKET_10B * 2))
#define OTA_PACKET_SIZE     (OTA_PACKET_PAYLOAD + OTA_PACKET_CRC)

// current and sent switch values
#define N_CONTROLS 4
#ifndef N_SWITCHES
//#define N_SWITCHES 8
#define N_SWITCHES 5
#endif
#define N_CHANNELS (N_CONTROLS + N_SWITCHES)

#ifndef TX_SKIP_SYNC_WHEN_ARMED
#define TX_SKIP_SYNC_WHEN_ARMED 0
#endif

/*************************************************
 *    Data conversion macros
 *************************************************/
/** NOTE
 * CRSF input range is [0...992...1984]
 * CRSF output range is [172...992...1811]
 **/
// OUT to flight controller
#define CRSF_CHANNEL_OUT_VALUE_MIN 172
#define CRSF_CHANNEL_OUT_VALUE_MID 992
#define CRSF_CHANNEL_OUT_VALUE_MAX 1811

// IN comming from handset
#define CRSF_CHANNEL_IN_VALUE_MIN 0
#define CRSF_CHANNEL_IN_VALUE_MID 992
#define CRSF_CHANNEL_IN_VALUE_MAX 1984


#define UINT10_to_CRSF(val) MAP_U16((val), 0, 1024, CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX)
#define CRSF_to_UINT10(val) MAP_U16((val), CRSF_CHANNEL_OUT_VALUE_MIN, CRSF_CHANNEL_OUT_VALUE_MAX, 0, 1023)

// 7 state aka 3b switches use 0...6 as values to represent 7 different values
// 1984 / 6 = 330 => taken down a bit to align result more evenly
// (1811-172) / 6 = 273
#define CRSF_to_SWITCH3b(val) ((val) / 300)
#define SWITCH3b_to_CRSF(val) ((val) * 273 + CRSF_CHANNEL_OUT_VALUE_MIN)

// 3 state aka 2b switches use 0, 1 and 2 as values to represent low, middle and high
// 819 = (1811-172) / 2
#define CRSF_to_SWITCH2b(val) ((val) / 819)
#define SWITCH2b_to_CRSF(val) ((val)*819 + CRSF_CHANNEL_OUT_VALUE_MIN)

#define CRSF_to_BIT(val) (((val) > 1000) ? 1 : 0)
#define BIT_to_CRSF(val) ((val) ? CRSF_CHANNEL_OUT_VALUE_MAX : CRSF_CHANNEL_OUT_VALUE_MIN)

/**
 * Protocol callbacks
*/
typedef void (*MspCallback_t)(uint8_t const *const input);
typedef void (*BattInfoCallback_t)(uint32_t voltage, uint32_t current, uint32_t capa);


// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 10 -> sync packet with hop data
// 11 -> tlm packet (MSP)
enum
{
    UL_PACKET_UNKNOWN = 0b00,
    UL_PACKET_RC_DATA = 0b01,
    UL_PACKET_SYNC = 0b10,
    UL_PACKET_MSP = 0b11,
};

enum
{
    DL_PACKET_FREE1 = 0b00,
    DL_PACKET_TLM_MSP = 0b01,
    DL_PACKET_GPS = 0b10,
    DL_PACKET_TLM_LINK = 0b11,
};

typedef struct ElrsSyncPacket_s {
    uint16_t CRCCaesarCipher;
    uint8_t fhssIndex;
    uint8_t rxtx_counter;
#if RX_UPDATE_AIR_RATE
    uint8_t air_rate: 4;
    uint8_t tlm_interval : 4;
#else
    uint8_t tlm_interval;
#endif
    uint8_t pkt_type;
} ElrsSyncPacket_s;


inline __attribute__((always_inline)) uint8_t
RcChannels_packetTypeGet(uint8_t const *const input)
{
    return input[OTA_PACKET_DATA-1] & 0b11;
}

inline __attribute__((always_inline)) void
RcChannels_packetTypeSet(uint8_t *const output, uint8_t type)
{
    uint8_t val = output[OTA_PACKET_DATA-1];
    val = (val & 0xFC) + (type & 0b11);
    output[OTA_PACKET_DATA-1] = val;
}


/*************************************************************************************
 * RC OTA PACKET
 *************************************************************************************/
typedef struct rc_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED rc_channels_t;

void
RcChannels_processChannels(rc_channels_t const *const channels);
void ICACHE_RAM_ATTR
RcChannels_get_packed_data(uint8_t *const output);
uint8_t ICACHE_RAM_ATTR
RcChannels_get_arm_channel_state(void);

// RX related
void ICACHE_RAM_ATTR
RcChannels_channels_extract(uint8_t const *const input,
                            rc_channels_t &output);

/*************************************************************************************
 * TELEMETRY OTA PACKET
 *************************************************************************************/
uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_ota_send(uint8_t *const output,
                        mspPacket_t &packet,
                        uint8_t tx=1);
uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_ota_receive(uint8_t const *const input,
                           mspPacket_t &packet);

/*************************************************************************************
 * LINK STATISTICS OTA PACKET
 *************************************************************************************/
typedef struct LinkStats_s {
    struct {
        /* NOTE: keep these aligned to crsfLinkStatistics_t */
        uint8_t uplink_RSSI_1;
        uint8_t uplink_RSSI_2;
        uint8_t uplink_Link_quality;
        int8_t uplink_SNR;
        uint8_t active_antenna;
        uint8_t rf_Mode;
        uint8_t uplink_TX_Power;
        uint8_t downlink_RSSI;
        uint8_t downlink_Link_quality;
        int8_t downlink_SNR;
    } link;
    struct {
        /* NOTE: keep these aligned to crsf_sensor_battery_t */
        uint16_t voltage;  // mv * 100
        uint16_t current;  // ma * 100
        uint32_t capacity : 24; // mah
        uint32_t remaining : 8; // %
    } batt;
} PACKED LinkStats_t;

void ICACHE_RAM_ATTR
RcChannels_link_stas_pack(uint8_t *const output,
                          LinkStats_t &input, uint_fast8_t ul_lq);
void ICACHE_RAM_ATTR
RcChannels_link_stas_extract(uint8_t const *const input,
                             LinkStats_t &output,
                             int8_t snr, uint8_t rssi);


/*************************************************************************************
 * GPS OTA PACKET
 *************************************************************************************/
typedef struct GpsOta_s {
    int32_t latitude;
    int32_t longitude;
    uint16_t speed;
    uint16_t heading;
    uint16_t altitude;
    uint8_t satellites;
} PACKED GpsOta_t;

/* Packet received callback */
typedef void (*GpsCallback_t)(GpsOta_t * gps);


#endif /* __RC_CHANNELS_H */
