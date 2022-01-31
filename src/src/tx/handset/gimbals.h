#pragma once

#include <stdint.h>


#define NUM_ANALOGS   4


void gimbals_init(void);
void gimbals_timer_adjust(uint32_t us);
void gimbals_get(uint16_t * const out, uint16_t channelMin, uint16_t channelMax);

uint8_t gimbals_calibrate(uint8_t * data);
uint8_t gimbals_adjust_min(uint16_t val, uint8_t idx);
uint8_t gimbals_adjust_mid(uint16_t val, uint8_t idx);
uint8_t gimbals_adjust_max(uint16_t val, uint8_t idx);
