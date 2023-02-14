#ifndef PLATFORM_H_
#define PLATFORM_H_

#ifdef ARDUINO_ARCH_ESP32

#include <esp_attr.h>
#include <sys/cdefs.h>

#define FAST_CODE_1 IRAM_ATTR
#define FAST_CODE_2 IRAM_ATTR

#define DRAM_FORCE_ATTR DRAM_ATTR
#else
#include <c_types.h>

#define FAST_CODE_1 ICACHE_RAM_ATTR
#define FAST_CODE_2 ICACHE_RAM_ATTR

#define DRAM_ATTR
#define DMA_ATTR WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#define DRAM_FORCE_ATTR
#endif


#endif /* PLATFORM_H_ */
