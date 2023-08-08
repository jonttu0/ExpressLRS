#ifndef _HAL_INC__
#define _HAL_INC__

#ifndef USE_HAL_DRIVER
#define USE_HAL_DRIVER
#endif

#define RCC_ENCODE(periph) (rcc_reg_t)(CRM_##periph##_PERIPH_CLOCK)

#if defined(AT32F415)
    #include "at32f415.h"
#elif defined(AT32F403A_407)
    #include "at32f403a_407.h"
#else
    #error "AT32YYxx chip series is not defined in boards.txt."
#endif

#endif /* _HAL_INC__ */
