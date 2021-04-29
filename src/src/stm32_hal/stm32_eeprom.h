#pragma once

#include "stm32_def.h"

uint8_t eeprom_read(uint8_t * out, uint32_t len);
uint8_t eeprom_write(uint8_t * data, uint32_t len);
