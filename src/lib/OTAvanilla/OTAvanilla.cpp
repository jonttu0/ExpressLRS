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


#if TARGET_HANDSET && OTA_VANILLA_ENABLED

// expresslrs packet header types
// 00 -> standard channel data packet
// 01 -> MSP data packet
// 11 -> TLM packet
// 10 -> sync packet
#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define TLM_PACKET 0b11
#define SYNC_PACKET 0b10

// Mask used to XOR the ModelId into the SYNC packet for ModelMatch
#define MODELMATCH_MASK 0x3f



static uint8_t TXdataBuffer[OTA_VANILLA_SIZE];
uint8_t arm_channel_state;

// http://users.ece.cmu.edu/~koopman/crc/c14/0x372b.txt
// Poly: 0x2E57 (implicit+1 = 0x372B), reversed: FALSE, (HD=6 up to 57b)
GenericLutCRC<uint16_t, 0x2E57, 14> DMA_ATTR crc_vanilla;


#define sm1Bit          0
#define smHybrid        1
#define smHybridWide    2

#define getModelID 0
#define OtaGetSwitchMode() smHybrid



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
PackChannelDataHybridCommon(uint8_t* const Buffer, rc_channels_handset_t const *const ChannelDataIn)
{
    Buffer[0] = RC_DATA_PACKET;
    Buffer[1] = ((ChannelDataIn->ch[0]) >> 3);
    Buffer[2] = ((ChannelDataIn->ch[1]) >> 3);
    Buffer[3] = ((ChannelDataIn->ch[2]) >> 3);
    Buffer[4] = ((ChannelDataIn->ch[3]) >> 3);
    Buffer[5] = ((ChannelDataIn->ch[0] & 0b110) << 5) +
                    ((ChannelDataIn->ch[1] & 0b110) << 3) +
                    ((ChannelDataIn->ch[2] & 0b110) << 1) +
                    ((ChannelDataIn->ch[3] & 0b110) >> 1);
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
GenerateChannelDataHybrid8(uint8_t* const Buffer, rc_channels_handset_t const *const ChannelDataIn)
{
    // The next switch index to send, where 0=AUX2 and 6=AUX8
    static uint8_t Hybrid8NextSwitchIndex;

    PackChannelDataHybridCommon(Buffer, ChannelDataIn);

    // Actually send switchIndex - 1 in the packet, to shift down 1-7 (0b111) to 0-6 (0b110)
    // If the two high bits are 0b11, the receiver knows it is the last switch and can use
    // that bit to store data
    uint8_t bitclearedSwitchIndex = Hybrid8NextSwitchIndex;
    uint8_t value;
    // AUX8 is High Resolution 16-pos (4-bit)
    if (bitclearedSwitchIndex == 6)
        value = CRSF_to_N(ChannelDataIn->ch[6 + 1 + 4], 16);
    else
    {
        // AUX2-7 are Low Resolution, "7pos" 6+center (3-bit)
        // The output is mapped evenly across 6 output values (0-5)
        // with a special value 7 indicating the middle so it works
        // with switches with a middle position as well as 6-position
        const uint16_t CHANNEL_BIN_COUNT = 6;
        const uint16_t CHANNEL_BIN_SIZE = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / CHANNEL_BIN_COUNT;
        uint16_t ch = ChannelDataIn->ch[bitclearedSwitchIndex + 1 + 4];
        // If channel is within 1/4 a BIN of being in the middle use special value 7
        if (ch < (CRSF_CHANNEL_VALUE_MID - CHANNEL_BIN_SIZE / 4)
            || ch > (CRSF_CHANNEL_VALUE_MID + CHANNEL_BIN_SIZE / 4))
            value = CRSF_to_N(ch, CHANNEL_BIN_COUNT);
        else
            value = 7;
    } // If not 16-pos

    Buffer[6] =
        //(TelemetryStatus << 7) |
        // switch 0 is one bit sent on every packet - intended for low latency arm/disarm
        (CRSF_to_BIT(ChannelDataIn->ch[4]) << 6) |
        // tell the receiver which switch index this is
        (bitclearedSwitchIndex << 3) |
        // include the switch value
        value;

    // update the sent value
    Hybrid8NextSwitchIndex = (bitclearedSwitchIndex + 1) % 7;
}

void FAST_CODE_2
GenerateChannelDataHybrid8_update(uint8_t* const Buffer, uint8_t TelemetryStatus)
{
    // Just fill current time based data here...
   Buffer[6] |= (TelemetryStatus << 7);
}


/*****************************************
 * PUBLIC FUNCTIONS
 *****************************************/

void FAST_CODE_1
OTA_vanilla_processChannels(rc_channels_handset_t const *const channels)
{
    arm_channel_state = CRSF_to_BIT(channels->ch[4]);
    // TODO: process input data
#if OtaGetSwitchMode() != smHybridWide
    GenerateChannelDataHybrid8(TXdataBuffer, channels);
#else
    #error "smHybridWide is not supported!"
#endif
}

void FAST_CODE_1
OTA_vanilla_SyncPacketData(uint8_t * const tx_buff,
                           uint32_t const NonceTX,
                           uint8_t const tlm_interval)
{
    const uint8_t UID[6] = {MY_UID};

    tx_buff[0] = SYNC_PACKET;
    tx_buff[1] = FHSSgetCurrIndex();
    tx_buff[2] = NonceTX;
    tx_buff[3] = ((current_rate_config & 3) << 6) +
                    ((tlm_interval & 7) << 3) +
                    ((OtaGetSwitchMode() & 3) << 1);
    tx_buff[4] = UID[3];
    tx_buff[5] = UID[4];
    tx_buff[6] = UID[5];
    tx_buff[7] = 0;
}

void FAST_CODE_1
OTA_vanilla_PackChannelData(uint8_t * const tx_buff, uint32_t NonceTX,
                            uint8_t NonceFHSSresult)
{
    (void)NonceTX;
    (void)NonceFHSSresult;
    // Fill TX buffer
    memcpy(tx_buff, TXdataBuffer, sizeof(TXdataBuffer));
    // Update values for this TX frame
    GenerateChannelDataHybrid8_update(tx_buff, 0);
}

void FAST_CODE_1
OTA_vanilla_calculateCrc(uint8_t * const tx_buff, uint16_t cipher)
{
    uint16_t const crc = crc_vanilla.calc(tx_buff, 7, cipher);
    tx_buff[0] &= 0b11;
    tx_buff[0] += (crc >> 6) & 0b11111100;
    tx_buff[7]  = crc & 0xFF;
}

#endif // TARGET_HANDSET
