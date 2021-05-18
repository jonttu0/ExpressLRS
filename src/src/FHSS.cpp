#include "FHSS.h"
#include "debug_elrs.h"
#include "common.h"
#include "utils.h"
#include "crc.h"
#include "targets.h"

#if RADIO_SX127x
#include "fhss_freqs_127x.h"
#endif
#if RADIO_SX128x
#include "fhss_freqs_128x.h"
#if RADIO_SX128x_FLRC
#include "fhss_freqs_128x_flrc.h"
#endif
#endif

static uint32_t DRAM_FORCE_ATTR FHSSstep;
static uint32_t DRAM_FORCE_ATTR FHSSsequenceLen;
static uint8_t * DRAM_FORCE_ATTR FHSSsequence;
static uint32_t * DRAM_FORCE_ATTR FHSSfreqs;


volatile uint32_t DRAM_ATTR FHSSptr;
volatile int_fast32_t DRAM_ATTR FreqCorrection;

void FHSS_init(uint8_t mode)
{
#if RADIO_SX127x
    if (mode == RADIO_TYPE_127x) {
        FHSSsequence = SX127x::FHSSsequence;
        FHSSsequenceLen = sizeof(SX127x::FHSSsequence);
        FHSSfreqs = SX127x::FHSSfreqs;
        FHSSstep = SX127x::FHSS_MY_STEP;
    }
#endif
#if RADIO_SX128x
    if (mode == RADIO_TYPE_128x) {
        FHSSsequence = SX128x::FHSSsequence;
        FHSSsequenceLen = sizeof(SX128x::FHSSsequence);
        FHSSfreqs = SX128x::FHSSfreqs;
        FHSSstep = SX128x::FHSS_MY_STEP;
    }

#if RADIO_SX128x_FLRC
    if (mode == RADIO_TYPE_128x_FLRC) {
        FHSSsequence = SX128x_FLRC::FHSSsequence;
        FHSSsequenceLen = sizeof(SX128x_FLRC::FHSSsequence);
        FHSSfreqs = SX128x_FLRC::FHSSfreqs;
        FHSSstep = SX128x_FLRC::FHSS_MY_STEP;
    }
#endif
#endif
    //FHSSrandomiseFHSSsequence();
    DEBUG_PRINTF("FHSS len %hu, step %hu\n",
        FHSSsequenceLen, FHSSstep);
}

void FAST_CODE_1 FHSSfreqCorrectionReset(void)
{
    FreqCorrection = 0;
}

void FAST_CODE_1 FHSSfreqCorrectionSet(int32_t error)
{
    FreqCorrection += error;
}

void FAST_CODE_1 FHSSsetCurrIndex(uint32_t value)
{ // set the current index of the FHSS pointer
    FHSSptr = value % FHSSsequenceLen;
}

uint32_t FAST_CODE_1 FHSSgetCurrIndex()
{ // get the current index of the FHSS pointer
    return FHSSptr;
}

void FAST_CODE_1 FHSSincCurrIndex()
{
#if !STAY_ON_INIT_CHANNEL
    FHSSptr = (FHSSptr + FHSSstep) % FHSSsequenceLen;
#endif
}

uint8_t FAST_CODE_1 FHSSgetCurrSequenceIndex()
{
    return FHSSsequence[FHSSptr];
}

uint32_t FAST_CODE_1 GetInitialFreq()
{
    return FHSSfreqs[0] - FreqCorrection;
}

uint32_t FAST_CODE_1 FHSSgetCurrFreq()
{
    return FHSSfreqs[FHSSsequence[FHSSptr]] - FreqCorrection;
}

uint32_t FAST_CODE_1 FHSSgetNextFreq()
{
    FHSSincCurrIndex();
    return FHSSgetCurrFreq();
}



#if 0

// Set all of the flags in the array to true, except for the first one
// which corresponds to the sync channel and is never available for normal
// allocation.
void resetIsAvailable(uint8_t *const array, uint32_t size)
{
    // channel 0 is the sync channel and is never considered available
    array[0] = 0;

    // all other entires to 1
    for (uint32_t i = 1; i < size; i++)
        array[i] = 1;
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pesudorandom

Approach:
  Initialise an array of flags indicating which channels have not yet been assigned and a counter of how many channels are available
  Iterate over the FHSSsequence array using index
    if index is a multiple of SYNC_INTERVAL assign the sync channel index (0)
    otherwise, generate a random number between 0 and the number of channels left to be assigned
    find the index of the nth remaining channel
    if the index is a repeat, generate a new random number
    if the index is not a repeat, assing it to the FHSSsequence array, clear the availability flag and decrement the available count
    if there are no available channels left, reset the flags array and the count
*/
void FHSSrandomiseFHSSsequence(uint8_t mode)
{
    const uint32_t nbr_fhss_seq = (sizeof(FHSSfreqs) / sizeof(FHSSfreqs[0]));

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

    DEBUG_PRINTF("Number of FHSS frequencies = %u\n", nbr_fhss_seq);

    uint8_t UID[6] = {MY_UID};
    uint32_t macSeed = CalcCRC32(UID, sizeof(UID));
    rngSeed(macSeed);

    uint8_t isAvailable[nbr_fhss_seq];

    resetIsAvailable(isAvailable, sizeof(isAvailable));

    // Fill the FHSSsequence with channel indices
    // The 0 index is special - the 'sync' channel. The sync channel appears every
    // syncInterval hops. The other channels are randomly distributed between the
    // sync channels
    const int SYNC_INTERVAL = 20;

    int nLeft = nbr_fhss_seq - 1; // how many channels are left to be allocated. Does not include the sync channel
    unsigned int prev = 0;           // needed to prevent repeats of the same index

    // for each slot in the sequence table
    for (uint32_t i = 0; i < FHSSsequenceLen; i++)
    {
        if (i % SYNC_INTERVAL == 0)
        {
            // assign sync channel 0
            FHSSsequence[i] = 0;
            prev = 0;
        }
        else
        {
            // pick one of the available channels. May need to loop to avoid repeats
            unsigned int index;
            do
            {
                int c = rngN(nLeft); // returnc 0<c<nLeft
                // find the c'th entry in the isAvailable array
                // can skip 0 as that's the sync channel and is never available for normal allocation
                index = 1;
                int found = 0;
                while (index < nbr_fhss_seq)
                {
                    if (isAvailable[index])
                    {
                        if (found == c)
                            break;
                        found++;
                    }
                    index++;
                }
                if (index == nbr_fhss_seq)
                {
                    // This should never happen
                    DEBUG_PRINTF("FAILED to find the available entry!\n");
                    // What to do? We don't want to hang as that will stop us getting to the wifi hotspot
                    // Use the sync channel
                    index = 0;
                    break;
                }
            } while (index == prev); // can't use index if it repeats the previous value

            FHSSsequence[i] = index; // assign the value to the sequence array
            isAvailable[index] = 0;  // clear the flag
            prev = index;            // remember for next iteration
            nLeft--;                 // reduce the count of available channels
            if (nLeft == 0)
            {
                // we've assigned all of the channels, so reset for next cycle
                resetIsAvailable(isAvailable, sizeof(isAvailable));
                nLeft = nbr_fhss_seq - 1;
            }
        }

        DEBUG_PRINTF("%u ", FHSSsequence[i]);
        if ((i + 1) % 10 == 0)
        {
            DEBUG_PRINTF("\n");
        }
    } // for each element in FHSSsequence

    DEBUG_PRINTF("\n");
}
#endif
