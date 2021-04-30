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

#include "stm32_eeprom.h"
#include "debug_elrs.h"
#include <string.h>

#ifndef FLASH_BASE
#define FLASH_BASE ((uint32_t)0x08000000u)
#endif

#if defined(STM32MP1xx)
/* Note for STM32MP1xx devices:
 * Those devices do not have non-volatile memory. The emulation is done
 * in RETRAM. Therefore data will be preserved *only* when VBAT is supplied
 * (e.g. A coin battery is connected to CN3 on STM32MP157A_DK1) and
 * the coprocessor is waken up from STANBY mode.
 * The data won't be preserved from cold boot, even if VBAT is connected.
 * See: https://community.st.com/s/question/0D50X0000B44pHUSQY/doesnt-the-mcu-coprocessor-have-nonvolatile-memory
 */
#define EEPROM_RETRAM_MODE 1

/* 4kB is the same size as EEPROM size of ATMega2560. */
#ifndef EEPROM_RETRAM_MODE_SIZE
#define EEPROM_RETRAM_MODE_SIZE ((uint32_t)(4*1024))
#endif

/* RETRAM start address is 0x00000000 (retset entry) and end address is
 * 0x00020000 (64kB in total). The by default, ldscript.ld for STM32MP1xx
 * does not define address between 0x00000298 (end of ISR Vector) and 0x00020000.
 * So it is okay to use in this address range. Make sure ldscript.ld does not
 * overrap the following address range.
 */
#ifndef EEPROM_RETRAM_START_ADDRESS
#define EEPROM_RETRAM_START_ADDRESS (0x00000400UL)
#endif

#define EEPROM_SIZE (EEPROM_RETRAM_MODE_SIZE)

#define EEPROM_START_ADDR ((uint8_t*)(EEPROM_RETRAM_START_ADDRESS))

#else // !defined(STM32MP1xx)

/* Linker is used to define start and end addresses */
extern __IO uint32_t __EEPROM_START;
extern __IO uint32_t __EEPROM_END;

#define EEPROM_START_ADDR (((uint8_t*)&__EEPROM_START))
#define EEPROM_SIZE       (uint32_t)((uint8_t*)&__EEPROM_END - (uint8_t*)&__EEPROM_START)

#endif // defined(STM32MP1xx)


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

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t offset = 0;
  uint32_t address = (uint32_t)EEPROM_START_ADDR;
  uint32_t address_end = address + EEPROM_SIZE - 1;

  /* Check whether the save is needed or not */
  if (memcmp(input, EEPROM_START_ADDR, len) == 0) {
    /* Content is same */
    return 1;
  }

#if defined (STM32F0xx) || defined (STM32F1xx) || defined (STM32F3xx) || \
    defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L0xx) || \
    defined (STM32L1xx) || defined (STM32L4xx) || defined (STM32WBxx)
  uint64_t data = 0;
  uint32_t pageError = 0;
  extern const uint32_t __FLASH_PAGE_SIZE;

  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
#ifdef FLASH_BANK_1
  erase_init.Banks = FLASH_BANK_1;
#endif
#if defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || defined (STM32WBxx)
  EraseInitStruct.Page = (address - FLASH_BASE) / __FLASH_PAGE_SIZE;
#else
  EraseInitStruct.PageAddress = address;
#endif
  EraseInitStruct.NbPages = (EEPROM_SIZE) / __FLASH_PAGE_SIZE;

  if (HAL_FLASH_Unlock() == HAL_OK) {

#if defined(STM32L0xx)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | \
                           FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);

#elif defined(STM32L1xx)
#if defined(FLASH_SR_RDERR)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR);
#else
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
#endif

#elif defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || defined (STM32WBxx)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
#else
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
#endif

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK) {
      while (address <= address_end) {
#if defined (STM32G0xx) || defined (STM32G4xx) || defined (STM32L4xx) || defined (STM32WBxx)
        /* 64bit write */
        data = *((uint64_t *)((uint8_t *)input + offset));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) == HAL_OK) {
          address += 8;
          offset += 8;
#else
        /* 32bit write */
        memcpy(&data, input + offset, sizeof(uint32_t));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
          address += 4;
          offset += 4;
#endif
        } else {
          address = address_end + 1;
        }
      }
    }
    HAL_FLASH_Lock();
  }
#else
  uint32_t SectorError = 0;
#if defined(STM32H7xx)
  uint64_t data[4] = {0x0000};
#else
  uint32_t data = 0;
#endif
  extern const uint32_t __EEPROM_SECTOR;
  extern const uint32_t __EEPROM_SECTOR_CNT;

  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
#if defined(STM32H7xx)
  EraseInitStruct.Banks = FLASH_BANK_1;
#endif
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = (uint32_t)&__EEPROM_SECTOR;
  EraseInitStruct.NbSectors = (uint32_t)&__EEPROM_SECTOR_CNT;

  HAL_FLASH_Unlock();

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
    while (address <= address_end) {
#if defined(STM32H7xx)
      /* 256 bits */
      memcpy(&data, input + offset, 8 * sizeof(uint32_t));
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data) == HAL_OK) {
        address += 32;
        offset += 32;
      }
#else
      memcpy(&data, input + offset, sizeof(uint32_t));
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
        address += 4;
        offset += 4;
      }
#endif
      else {
        DEBUG_PRINTF("FLASH WRITE FAILED!\n");
        address = address_end + 100;
      }
    }
  } else {
    DEBUG_PRINTF("FLASH ERASE FAILED!\n");
  }
  HAL_FLASH_Lock();
#endif

#endif /* EEPROM_RETRAM_MODE */

  return (address < EEPROM_SIZE);
}
