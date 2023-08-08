#ifndef _PLATFORM_INTERNAL_H_
#define _PLATFORM_INTERNAL_H_

#include "irq.h"

#ifndef __section
#define __section(S) __attribute__((section(S)))
#endif
#define ATTR_NO_INIT __section(".noinit")


#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#if RAM_CODE_IGNORE
#define FAST_CODE_1
#define FAST_CODE_2
#else // !RAM_CODE_IGNORE
/* FAST_CODE_1 is always linked into RAM */
#define FAST_CODE_1  __section(".ram_code")
/* FAST_CODE_2 is linked into RAM only if enough space */
#if RAM_CODE_LIMITED
#define FAST_CODE_2
#else
#define FAST_CODE_2 __section(".ram_code")
#endif
#endif // RAM_CODE_IGNORE

#define DRAM_ATTR
#define DMA_RAM_ATTR        __section(".dma_data")
#define DRAM_FORCE_ATTR     __section(".data")
#define DMA_ATTR            WORD_ALIGNED_ATTR DMA_RAM_ATTR
#define WORD_ALIGNED_ATTR   __attribute__((aligned(32)))

#define _DISABLE_IRQ()      irq_disable()
#define _ENABLE_IRQ()       irq_enable()
#define _SAVE_IRQ()         irq_save()
#define _RESTORE_IRQ(_x)    irq_restore(_x)

#endif /* _PLATFORM_INTERNAL_H_ */
