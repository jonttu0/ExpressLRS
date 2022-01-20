#include "utils.h"
#include "platform.h"

static uint32_t seed = 0;

// returns values between 0 and 0x7FFF
// NB rngN depends on this output range, so if we change the
// behaviour rngN will need updating
static uint16_t rng(void)
{
    constexpr uint32_t m = 2147483648;
    constexpr uint32_t a = 214013;
    constexpr uint32_t c = 2531011;
    seed = (a * seed + c) % m;
    return seed >> 16;
}

void rngSeed(const uint32_t newSeed)
{
    seed = newSeed;
}

uint8_t rngN(const uint32_t max)
{
    return rng() % max;
}

uint8_t rng8(void)
{
    return rng() & 0xff;
}

uint8_t rng5(void)
{
    return rng() & 0x1F;
}

unsigned int FAST_CODE_1
volatile_memcpy(volatile void *d, volatile void *s, unsigned int n)
{
    volatile unsigned char *dst = (unsigned char *)d;
    volatile unsigned char *src = (unsigned char *)s;
    unsigned int iter;
    for (iter = 0; iter < n; iter++) {
        *dst++ = *src++;
    }
    return iter;
}
