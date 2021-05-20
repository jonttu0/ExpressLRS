#ifndef __FHSS_H
#define __FHSS_H

#include "platform.h"
#include <stdint.h>

//#define FHSS_SYNC_CHANNEL    0
//#define STAY_ON_INIT_CHANNEL 1

#ifndef FHSS_SYNC_CHANNEL
#define FHSS_SYNC_CHANNEL 0
#endif
#ifndef FHSS_INDEX_INIT
#define FHSS_INDEX_INIT 0
#endif

#define FreqCorrectionMax 100000
#define FreqCorrectionMin -100000
#define FreqCorrectionStep 61 //min freq step is ~ 61hz

void FHSSfreqCorrectionReset(void);
void FHSSfreqCorrectionSet(int32_t error);

void FAST_CODE_1 FHSS_init(uint8_t mode);

#define FHSSresetCurrIndex() FHSSsetCurrIndex(FHSS_INDEX_INIT)
void FAST_CODE_1 FHSSsetCurrIndex(uint32_t value);
uint32_t FAST_CODE_1 FHSSgetCurrIndex();
void FAST_CODE_1 FHSSincCurrIndex();
uint8_t FAST_CODE_1 FHSSgetCurrSequenceIndex();

uint32_t FAST_CODE_1 GetInitialFreq();
uint32_t FAST_CODE_1 FHSSgetCurrFreq();
uint32_t FAST_CODE_1 FHSSgetNextFreq();

void FHSSrandomiseFHSSsequence(uint8_t mode);

#endif // __FHSS_H
