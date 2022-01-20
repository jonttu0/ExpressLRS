#include "FHSS.h"
#include "debug_elrs.h"
#include "common.h"
#include "utils.h"
#include "crc.h"
#include "targets.h"


#define FHSS_SEQ_TABLE_SIZE 256

// Actual sequence of hops as indexes into the frequency list
static uint32_t DRAM_FORCE_ATTR FHSS_freq_base;
static uint32_t DRAM_FORCE_ATTR FHSS_bandwidth;
static uint32_t DRAM_FORCE_ATTR FHSS_band_count;
static uint32_t DRAM_FORCE_ATTR FHSS_sync_channel;
static uint32_t DRAM_FORCE_ATTR FHSS_sequence_len;
static uint8_t  DRAM_FORCE_ATTR FHSS_sequence_lut[FHSS_SEQ_TABLE_SIZE];

// Runtime variables
static volatile uint32_t DRAM_ATTR FHSSindex;
static volatile int32_t  DRAM_ATTR FreqCorrection;


static void FHSSupdateFrequencies(uint8_t mode);
static void FHSSrandomiseSequence(uint32_t nbr_fhss_freqs);
static void FHSSprintSequence(const uint8_t fhss_sequence_len)
{
#ifdef DEBUG_SERIAL
    // output FHSS sequence
    uint8_t iter;
    DEBUG_PRINTF("FHSS Sequence:");
    for (iter = 0; iter < fhss_sequence_len; iter++) {
        if ((iter % 10) == 0)
            DEBUG_PRINTF("\n");
        DEBUG_PRINTF("%2u ", FHSS_sequence_lut[iter]);
    }
    DEBUG_PRINTF("\n");
#else
    (void)fhss_sequence_len;
#endif
}

void FHSS_init(uint8_t const mode)
{
#if 0
    if (RADIO_SX127x && mode == RADIO_TYPE_127x) {
#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915)
        DEBUG_PRINTF("Setting 915MHz Mode\n");
#elif defined Regulatory_Domain_EU_868 || defined Regulatory_Domain_EU_868_R9
        DEBUG_PRINTF("Setting 868MHz Mode\n");
#elif defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
        DEBUG_PRINTF("Setting 433MHz Mode\n");
#endif
    } else if (RADIO_SX128x && mode == RADIO_TYPE_128x) {
        DEBUG_PRINTF("Setting ISM 2400 Mode\n");
    }
#endif
    FHSSupdateFrequencies(mode);
    FHSSrandomiseSequence(FHSS_band_count);
    FHSSprintSequence(FHSS_band_count);
}

void FAST_CODE_1 FHSSfreqCorrectionReset(void)
{
    FreqCorrection = 0;
}

void FAST_CODE_1 FHSSfreqCorrectionSet(int32_t const error)
{
    FreqCorrection += error;
}

void FAST_CODE_1 FHSSsetCurrIndex(uint32_t const value)
{ // set the current index of the FHSS pointer
    FHSSindex = value % FHSS_sequence_len;
}

uint32_t FAST_CODE_1 FHSSgetCurrIndex()
{ // get the current index of the FHSS pointer
    return FHSSindex;
}

void FAST_CODE_1 FHSSincCurrIndex()
{
#if !STAY_ON_INIT_CHANNEL
    FHSSindex = (FHSSindex + 1) % FHSS_sequence_len;
#endif
}

uint8_t FAST_CODE_1 FHSScurrSequenceIndex()
{
    return (FHSS_sequence_lut[FHSSindex]);
}

uint8_t FAST_CODE_1 FHSScurrSequenceIndexIsSyncChannel()
{
    return (FHSS_sequence_lut[FHSSindex] == FHSS_sync_channel);
}

uint32_t FAST_CODE_1 FHSSgetCurrFreq()
{
    uint32_t freq = FHSS_freq_base;
    freq += (FHSS_bandwidth * FHSS_sequence_lut[FHSSindex]);
    return (freq - FreqCorrection);
}

uint32_t FAST_CODE_1 FHSSgetNextFreq()
{
    FHSSincCurrIndex();
    return FHSSgetCurrFreq();
}


static void FHSSupdateFrequencies(uint8_t const mode)
{
    /* Fill/calc FHSS frequencies base on requested mode */
    switch (mode) {
        case RADIO_TYPE_127x: {
#if defined(Regulatory_Domain_AU_433)
            FHSS_freq_base = 433420000;
            FHSS_bandwidth = 500000;
            FHSS_band_count = 3;
#elif defined(Regulatory_Domain_EU_433)
            FHSS_freq_base = 433100000; // 433100000, 433925000, 434450000
            FHSS_bandwidth = ; // ???
            FHSS_band_count = 3;
#elif defined(Regulatory_Domain_AU_915)
            FHSS_freq_base = 915500000;
            FHSS_bandwidth = 600000;
            FHSS_band_count = 20;
#elif defined(Regulatory_Domain_FCC_915)
            FHSS_freq_base = 903500000;
            FHSS_bandwidth = 600000;
            FHSS_band_count = 40;
#elif defined(Regulatory_Domain_EU_868)
            FHSS_freq_base = 863275000;
            FHSS_bandwidth = 525000;
            FHSS_band_count = 18;
#elif defined(Regulatory_Domain_EU_868_R9)
            FHSS_freq_base = 859504989;
            FHSS_bandwidth = 500000;
            FHSS_band_count = 26;
#endif
            break;
        }
        case RADIO_TYPE_128x: {
#if defined(Regulatory_Domain_ISM_2400_800kHz)
            FHSS_freq_base = 2400400000;
            FHSS_bandwidth = 850000;
            FHSS_band_count = 94;
#else
            FHSS_freq_base = 2400400000;
            FHSS_bandwidth = 1650000;
            FHSS_band_count = 49;
#endif
            break;
        }
        case RADIO_TYPE_128x_FLRC: {
            FHSS_freq_base = 2400400000;
            FHSS_bandwidth = 330000;
            FHSS_band_count = 242;
            break;
        }
    }

    DEBUG_PRINTF("FHSS freq base:%u, bw:%u, count:%u\n",
                 FHSS_freq_base, FHSS_bandwidth, FHSS_band_count);
}


/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.
*/
static void
FHSSrandomiseSequence(const uint32_t nbr_fhss_freqs)
{
    uint8_t const sync_channel = nbr_fhss_freqs / 2;
    // Number of hops in the FHSS_sequence_lut list before circling back around,
    // even multiple of the number of frequencies
    const uint32_t fhss_sequence_len =
            (FHSS_SEQ_TABLE_SIZE / nbr_fhss_freqs) * nbr_fhss_freqs;
    uint8_t temp, iter, offset, rand;

    DEBUG_PRINTF("FHSS sequence len: %u, sync channel: %u\n",
                 fhss_sequence_len, sync_channel);

    rngSeed(my_uid_crc32());

    // initialize the sequence array
    for (iter = 0; iter < fhss_sequence_len; iter++) {
        if ((iter % nbr_fhss_freqs) == 0) {
            FHSS_sequence_lut[iter] = sync_channel;
        } else if ((iter % nbr_fhss_freqs) == sync_channel) {
            FHSS_sequence_lut[iter] = 0;
        } else {
            FHSS_sequence_lut[iter] = iter % nbr_fhss_freqs;
        }
    }

    // Randomize sequence
    for (iter = 0; iter < fhss_sequence_len; iter++) {
        // if it's not the sync channel
        if ((iter % nbr_fhss_freqs) != 0) {
            // offset to start of current block
            offset = (iter / nbr_fhss_freqs) * nbr_fhss_freqs;
            // random number between 1 and nbr_fhss_freqs
            rand = rngN(nbr_fhss_freqs - 1) + 1;
            // switch this entry and another random entry in the same block
            temp = FHSS_sequence_lut[iter];
            FHSS_sequence_lut[iter] = FHSS_sequence_lut[offset + rand];
            FHSS_sequence_lut[offset + rand] = temp;
        }
    }

    // Update global params
    FHSS_sequence_len = fhss_sequence_len;
    FHSS_sync_channel = sync_channel;
}
