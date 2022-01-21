#ifndef __FHSS_H
#define __FHSS_H

#include "platform.h"
#include <stdint.h>

//#define STAY_ON_INIT_CHANNEL 1

#ifndef FHSS_INDEX_INIT
#define FHSS_INDEX_INIT 0
#endif


void FHSSfreqCorrectionReset(void);
int32_t FHSSfreqCorrectionApply(int32_t error);

void FHSS_init(uint8_t mode);

#define FHSSresetCurrIndex() FHSSsetCurrIndex(FHSS_INDEX_INIT)
void FAST_CODE_1 FHSSsetCurrIndex(uint32_t value);
uint32_t FAST_CODE_1 FHSSgetCurrIndex();
void FAST_CODE_1 FHSSincCurrIndex();
uint8_t FAST_CODE_1 FHSScurrSequenceIndex();
uint8_t FAST_CODE_1 FHSScurrSequenceIndexIsSyncChannel();

uint32_t FAST_CODE_1 FHSSgetCurrFreq();
uint32_t FAST_CODE_1 FHSSgetNextFreq();

#endif // __FHSS_H
