/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * This file provides utilities for packing and unpacking the data to
 * be sent over the radio link.
 */

#include "OTAvanilla.h"
#include "platform.h"
#include "common.h"
#include "FHSS.h"
#include "crc.h"
#include "debug_elrs.h"
#include "helpers.h"
#include <string.h>

// defines from telemetry_protocol.h
#define ELRS_MSP_BYTES_PER_CALL 5
#define ELRS4_TELEMETRY_SHIFT 2
#define ELRS4_TELEMETRY_BYTES_PER_CALL 5


#if TARGET_HANDSET && OTA_VANILLA_ENABLED


#define OTA4_CRC_CALC_LEN    offsetof(OTA_Packet4_s, crcLow)

// Packet header types (ota.std.type)
#define PACKET_TYPE_RCDATA  0b00
#define PACKET_TYPE_MSPDATA 0b01
#define PACKET_TYPE_TLM     0b11
#define PACKET_TYPE_SYNC    0b10

#define ELRS_CRC14_POLY 0x2E57


typedef struct {
    uint8_t fhssIndex;
    uint8_t nonce;
    uint8_t switchEncMode:1,
            newTlmRatio:3,
            rateIndex:4;
    uint8_t UID3;
    uint8_t UID4;
    uint8_t UID5;
} PACKED OTA_Sync_s;

typedef struct {
    uint8_t uplink_RSSI_1:7,
            antenna:1;
    uint8_t uplink_RSSI_2:7,
            modelMatch:1;
    uint8_t lq:7,
            mspConfirm:1;
    int8_t SNR;
} PACKED OTA_LinkStats_s;

typedef union {
    uint8_t raw[5]; // 4x 10-bit channels, see PackUInt11ToChannels4x10 for encoding
    struct {
        unsigned ch0 : 10;
        unsigned ch1 : 10;
        unsigned ch2 : 10;
        unsigned ch3 : 10;
    } PACKED;
} OTA_Channels_4x10;

typedef struct {
    // The packet type must always be the low two bits of the first byte of the
    // packet to match the same placement in OTA_Packet8_s
    uint8_t type: 2,
            crcHigh: 6;
    union {
        /** PACKET_TYPE_RCDATA **/
        struct {
            OTA_Channels_4x10 ch;
            uint8_t switches:7,
                    ch4:1;
        } rc;
        struct {
            uint32_t packetNum; // LittleEndian
            uint8_t free[2];
        } PACKED dbg_linkstats;
        /** PACKET_TYPE_MSP **/
        struct {
            uint8_t packageIndex;
            uint8_t payload[ELRS_MSP_BYTES_PER_CALL];
        } msp_ul;
        /** PACKET_TYPE_SYNC **/
        OTA_Sync_s sync;
        /** PACKET_TYPE_TLM **/
        struct {
            uint8_t type:ELRS4_TELEMETRY_SHIFT,
                    packageIndex:(8 - ELRS4_TELEMETRY_SHIFT);
            union {
                struct {
                    OTA_LinkStats_s stats;
                    uint8_t free;
                } PACKED ul_link_stats;
                uint8_t payload[ELRS4_TELEMETRY_BYTES_PER_CALL];
            };
        } tlm_dl; // PACKET_TYPE_TLM
    };
    uint8_t crcLow;
} PACKED OTA_Packet4_s;

typedef struct {
    union {
        OTA_Packet4_s std;
        //OTA_Packet8_s full;  // NOT SUPPORTED!
        uint8_t TXdataBuffer[OTA_VANILLA_SIZE];
    };
} PACKED OTA_Packet_s;

static_assert(sizeof(OTA_Packet_s) == OTA_VANILLA_SIZE,
              "Incorrect packet size!");


static OTA_Packet_s ota_packet_buffer;
uint8_t arm_channel_state;

// http://users.ece.cmu.edu/~koopman/crc/c14/0x372b.txt
// Poly: 0x2E57 (implicit+1 = 0x372B), reversed: FALSE, (HD=6 up to 57b)
GenericLutCRC<uint16_t, ELRS_CRC14_POLY, 14> DMA_ATTR crc_vanilla;


enum OtaSwitchMode_e { smWideOr8ch = 0, smHybridOr16ch = 1, sm12ch = 2 };


// Convert CRSF to 0-(cnt-1), constrained between 1000us and 2000us
static inline uint16_t FAST_CODE_1 CRSF_to_N(uint16_t val, uint16_t cnt)
{
    // The span is increased by one to prevent the max val from returning cnt
    if (val <= CRSF_CHANNEL_VALUE_1000)
        return 0;
    if (val >= CRSF_CHANNEL_VALUE_2000)
        return cnt - 1;
    return (val - CRSF_CHANNEL_VALUE_1000) * cnt / (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000 + 1);
}

// Returns 1 if val is greater than CRSF_CHANNEL_VALUE_MID
static inline uint8_t FAST_CODE_1 CRSF_to_BIT(uint16_t val)
{
    return (val >= CRSF_CHANNEL_VALUE_MID) ? 1 : 0;
}



static void FAST_CODE_2
PackChannelDataHybridCommon(OTA_Packet4_s* const ota4, rc_channels_handset_t const *const ChannelDataIn)
{
    arm_channel_state = CRSF_to_BIT(ChannelDataIn->ch[4]);

    ota4->type = PACKET_TYPE_RCDATA;
    ota4->rc.ch.ch0 = ChannelDataIn->gimbal[0];
    ota4->rc.ch.ch1 = ChannelDataIn->gimbal[1];
    ota4->rc.ch.ch2 = ChannelDataIn->gimbal[2];
    ota4->rc.ch.ch3 = ChannelDataIn->gimbal[3];
    ota4->rc.ch4 = arm_channel_state;
}


/*****************************************
 * Hybrid 8
 *****************************************/

/**
 * Hybrid switches packet encoding for sending over the air
 *
 * Analog channels are reduced to 10 bits to allow for switch encoding
 * Switch[0] is sent on every packet.
 * A 3 bit switch index and 3-4 bit value is used to send the remaining switches
 * in a round-robin fashion.
 *
 * Inputs: crsf.ChannelDataIn, crsf.currentSwitches
 * Outputs: Radio.TXdataBuffer, side-effects the sentSwitch value
 */
void FAST_CODE_2
GenerateChannelDataHybrid8(OTA_Packet4_s* const ota4, rc_channels_handset_t const *const ChannelDataIn)
{
    // The next switch index to send, where 0=AUX2 and 6=AUX8
    static uint8_t Hybrid8NextSwitchIndex;

    PackChannelDataHybridCommon(ota4, ChannelDataIn);

    // Actually send switchIndex - 1 in the packet, to shift down 1-7 (0b111) to 0-6 (0b110)
    // If the two high bits are 0b11, the receiver knows it is the last switch and can use
    // that bit to store data
    uint8_t bitclearedSwitchIndex = Hybrid8NextSwitchIndex;
    uint8_t value;
    // AUX8 is High Resolution 16-pos (4-bit)
    if (7 <= N_SWITCHES && bitclearedSwitchIndex == 6) {
        value = CRSF_to_N(ChannelDataIn->aux[6 + 1], 16);
    } else {
        // AUX2-7 are Low Resolution, "7pos" 6+center (3-bit)
        // The output is mapped evenly across 6 output values (0-5)
        // with a special value 7 indicating the middle so it works
        // with switches with a middle position as well as 6-position
        const uint16_t CHANNEL_BIN_COUNT = 6;
        const uint16_t CHANNEL_BIN_SIZE = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / CHANNEL_BIN_COUNT;
        uint16_t ch = ChannelDataIn->aux[bitclearedSwitchIndex + 1];
        // If channel is within 1/4 a BIN of being in the middle use special value 7
        if (ch < (CRSF_CHANNEL_VALUE_MID - CHANNEL_BIN_SIZE / 4)
                || ch > (CRSF_CHANNEL_VALUE_MID + CHANNEL_BIN_SIZE / 4))
            value = CRSF_to_N(ch, CHANNEL_BIN_COUNT);
        else
            value = 7;
    } // If not 16-pos

    ota4->rc.switches =
        //TelemetryStatus << 6 |
        // tell the receiver which switch index this is
        (bitclearedSwitchIndex << 3) |
        // include the switch value
        value;

#if N_SWITCHES <= 7
#define VANILLA_N_SWITCHES N_SWITCHES
#else
#define VANILLA_N_SWITCHES 7
#endif

    // update the sent value
    Hybrid8NextSwitchIndex = (bitclearedSwitchIndex + 1) % VANILLA_N_SWITCHES;
}


/*****************************************
 * PUBLIC FUNCTIONS
 *****************************************/

void FAST_CODE_1
OTA_vanilla_processChannels(rc_channels_handset_t const *const channels)
{
    GenerateChannelDataHybrid8(&ota_packet_buffer.std, channels);
}

void FAST_CODE_1
OTA_vanilla_SyncPacketData(uint8_t * const tx_buff,
                           uint32_t const NonceTX,
                           uint8_t const tlm_interval)
{
    const uint8_t UID[6] = {MY_UID};
    uint8_t rateIndex = current_rate_config + 2;
    if (5 == rateIndex) rateIndex = 6; // Map LoRa 250Hz

    OTA_Packet4_s* const ota4 = (OTA_Packet4_s*)tx_buff;
    ota4->type = PACKET_TYPE_SYNC;
    ota4->sync.fhssIndex = FHSSgetCurrIndex();
    ota4->sync.nonce = NonceTX;
    ota4->sync.switchEncMode = smHybridOr16ch;
    ota4->sync.newTlmRatio = tlm_interval;
    ota4->sync.rateIndex = rateIndex;
    ota4->sync.UID3 = UID[3];
    ota4->sync.UID4 = UID[4];
    ota4->sync.UID5 = UID[5];
}

void FAST_CODE_1
OTA_vanilla_PackChannelData(uint8_t * const tx_buff, uint32_t NonceTX,
                            uint8_t NonceFHSSresult)
{
    (void)NonceTX;
    (void)NonceFHSSresult;
    // Fill TX buffer
    memcpy(tx_buff, ota_packet_buffer.TXdataBuffer, sizeof(ota_packet_buffer.TXdataBuffer));
}

void FAST_CODE_1
OTA_vanilla_calculateCrc(uint8_t * const tx_buff, uint16_t cipher)
{
    OTA_Packet4_s* const ota4 = (OTA_Packet4_s*)tx_buff;
    ota4->crcHigh = 0;
    ota4->crcLow = 0;
    uint16_t const crc = crc_vanilla.calc(tx_buff, OTA4_CRC_CALC_LEN, cipher);
    ota4->crcHigh = (crc >> 8);
    ota4->crcLow  = crc;
}

#endif // TARGET_HANDSET
