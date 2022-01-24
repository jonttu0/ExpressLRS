#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <c_types.h>

#define FAST_CODE_1 ICACHE_RAM_ATTR
#define FAST_CODE_2 ICACHE_RAM_ATTR

#define DRAM_ATTR
#define DMA_ATTR WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#define DRAM_FORCE_ATTR

#endif /* PLATFORM_H_ */
