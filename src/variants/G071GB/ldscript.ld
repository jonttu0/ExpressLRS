
/* Linker script for STM32G071GB Device with 128KByte FLASH, 36KByte SRAM */

/* Page is 2k */
__FLASH_PAGE_SIZE = 0x800;

/* Specify the memory areas */
MEMORY
{
  RAM_CODE (rx)   : ORIGIN = 0x20000000, LENGTH = 14K
  RAM_DATA (rwx)  : ORIGIN = 0x20000000 + LENGTH(RAM_CODE), LENGTH = RAM_SIZE - LENGTH(RAM_CODE)
  FLASH (rx)      : ORIGIN = 0x8000000 + FLASH_APP_OFFSET, LENGTH = FLASH_SIZE - FLASH_APP_OFFSET - __FLASH_PAGE_SIZE
  STORAGE (rx)    : ORIGIN = ORIGIN(FLASH) + LENGTH(FLASH), LENGTH = __FLASH_PAGE_SIZE
}

REGION_ALIAS("DMA_DATA", RAM_DATA)
REGION_ALIAS("FLASH_RAM_CODE", FLASH)
REGION_ALIAS("FLASH_STORAGE", STORAGE)

INCLUDE "variants/ldscript_gen.ld"
