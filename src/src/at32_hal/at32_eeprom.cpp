/**
 ******************************************************************************
 * @file    stm32_eeprom.c
 * @brief   Provides emulated eeprom from flash
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "at32_eeprom.h"
#include "debug_elrs.h"
#include <string.h>

/* Linker is used to define start and end addresses */
extern __IO uint32_t __EEPROM_START;
extern __IO uint32_t __EEPROM_END;

#define EEPROM_START_ADDR (((uint8_t *)&__EEPROM_START))
#define EEPROM_SIZE       (uint32_t)((uint8_t *)&__EEPROM_END - (uint8_t *)&__EEPROM_START)

/**
 * @brief  Function reads a bytes from the eeprom buffer
 *
 * @param  out[out]  pointer where data will be read
 * @param  len[in]   how much data will be read
 *
 * @retval 0 fail
 * @retval 1 success
 */
uint8_t eeprom_read(uint8_t * out, uint32_t len)
{
    if (!out || !len || EEPROM_SIZE < len)
        return 0;
    memcpy(out, EEPROM_START_ADDR, len);
    return 1;
}

/**
 * @brief  This function writes the buffer content into the flash
 *
 * @param  input[in] data to be written
 * @param  len[in]   how much data will be written
 *
 * @retval 0 fail
 * @retval 1 success
 */
uint8_t eeprom_write(uint8_t * input, uint32_t len)
{
    if (!input || !len || EEPROM_SIZE < len)
        return 0;

#if EEPROM_RETRAM_MODE

    memcpy(EEPROM_START_ADDR, input, len);

#else /* !EEPROM_RETRAM_MODE */

    uint32_t offset = 0;
    uint32_t address = (uint32_t)EEPROM_START_ADDR;
    uint32_t const address_end = address + len - 1;

    /* Check whether the save is needed or not */
    if (memcmp(input, (void *)address, len) == 0) {
        /* Content is same */
        return 1;
    }

    uint32_t data = 0;
    extern const uint32_t __FLASH_PAGE_SIZE;
    uint32_t const page_size = (uint32_t)&__FLASH_PAGE_SIZE;

    /* Start flash access */
    flash_unlock();

    /* ERASING page(s) */
    size_t const page_cnt = (EEPROM_SIZE) / page_size;
    for (uint8_t iter = 0; iter < page_cnt; iter++) {
        if (flash_sector_erase(address + iter * page_size) != FLASH_OPERATE_DONE) {
            DEBUG_PRINTF("FLASH ERASE FAILED!\n");
        }
    }

    /* WRITE data */
    while (address <= address_end) {
        /* 32bit write */
        memcpy(&data, input + offset, sizeof(uint32_t));
        if (flash_word_program(address, data) == FLASH_OPERATE_DONE) {
            address += 4;
            offset += 4;
        } else {
            DEBUG_PRINTF("FLASH WRITE FAILED!\n");
        }
    }

    /* Close flash access */
    flash_lock();

#endif /* EEPROM_RETRAM_MODE */

    return (address_end <= address);
}
