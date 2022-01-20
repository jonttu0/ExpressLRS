#pragma once

#include <stdint.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define LIMIT(mi, x, ma) MIN(MAX(mi, x), ma)


void rngSeed(uint32_t newSeed);
// returns 0 <= x < max where max < 256
uint8_t rngN(uint32_t max);
uint8_t rng8(void);
uint8_t rng5(void);

unsigned int
volatile_memcpy(volatile void *d, volatile void *s, unsigned int n);
