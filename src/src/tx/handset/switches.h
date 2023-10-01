#pragma once

#include <stdint.h>

#define NUM_SWITCHES   6

void switches_init(void);
void switches_collect(uint16_t * const out);
uint8_t switches_get_available(void);
