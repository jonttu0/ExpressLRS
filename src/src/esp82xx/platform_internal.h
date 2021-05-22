#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <Arduino.h>
#include <c_types.h>

#define likely(x) (x)
#define unlikely(x) (x)

#define FAST_CODE_1 IRAM_ATTR
#define FAST_CODE_2 IRAM_ATTR

#define DRAM_ATTR
#define DRAM_FORCE_ATTR DRAM_ATTR
#define DMA_ATTR WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))

#define _DISABLE_IRQ() noInterrupts()
#define _ENABLE_IRQ() interrupts()
#define _SAVE_IRQ() xt_rsil(0)
#define _RESTORE_IRQ(_x) xt_wsr_ps(_x)

#endif /* PLATFORM_H_ */
