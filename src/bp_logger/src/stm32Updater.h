#pragma once

#include <stdint.h>

#if CONFIG_STM_UPDATER
// taken and adapted from https://github.com/mengguang/esp8266_stm32_isp

#define FLASH_START 0x08000000
#ifndef FLASH_SIZE
#define FLASH_SIZE 0x10000 // 64kB
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 0x400 // 1kB
#endif
#ifndef FLASH_OFFSET
#define FLASH_OFFSET 0x2000 // skip bootloader
#endif
#define BEGIN_ADDRESS (FLASH_START + FLASH_OFFSET)


void reset_stm32_to_isp_mode();

void reset_stm32_to_app_mode();

void stm32flasher_hardware_init();

void debug_log();

uint8_t esp8266_spifs_write_file(const char *filename, uint32_t const begin_addr);

#endif // CONFIG_STM_UPDATER
