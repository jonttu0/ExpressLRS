/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * This file provides utilities for packing and unpacking the data to
 * be sent over the radio link.
 */

#ifndef H_OTA
#define H_OTA

#include "msp.h"            // just to make pio happy
#include "RadioInterface.h" // just to make pio happy
#include "rc_channels.h"
#include <stdint.h>


#if TARGET_HANDSET && OTA_VANILLA_ENABLED

#define OTA_VANILLA_SIZE 8

#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_2000 1792
#define CRSF_CHANNEL_VALUE_MAX  1811

static inline uint32_t __attribute__((always_inline))
OTA_vanilla_get_channelMinValue(void)
{
    return CRSF_CHANNEL_VALUE_1000;
}

static inline uint32_t __attribute__((always_inline))
OTA_vanilla_get_channelMaxValue(void)
{
    return CRSF_CHANNEL_VALUE_2000;
}

static inline uint8_t __attribute__((always_inline))
OTA_vanilla_getArmChannelState(void)
{
    extern uint8_t arm_channel_state;
    return arm_channel_state;
}

void OTA_vanilla_processChannels(rc_channels_handset_t const *const channels);

void OTA_vanilla_SyncPacketData(uint8_t * const TXdataBuffer, uint32_t NonceTX,
                                uint8_t tlm_interval);
void OTA_vanilla_PackChannelData(uint8_t * const TXdataBuffer, uint32_t NonceTX,
                                 uint8_t NonceFHSSresult);

void OTA_vanilla_calculateCrc(uint8_t * const TXdataBuffer, uint16_t cipher);

#endif // TARGET_HANDSET

#endif // H_OTA
