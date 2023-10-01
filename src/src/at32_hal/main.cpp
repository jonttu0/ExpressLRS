#include "Arduino.h"
#include "at32_def.h"


#ifdef DWT_BASE
static void dwt_access(bool ena)
{
#if (__CORTEX_M == 0x07U)
    /*
    * Define DWT LSR mask which is (currentuly) not defined by the CMSIS.
    * Same as ITM LSR one.
    */
#if !defined DWT_LSR_Present_Msk
#define DWT_LSR_Present_Msk ITM_LSR_Present_Msk
#endif
#if !defined DWT_LSR_Access_Msk
#define DWT_LSR_Access_Msk ITM_LSR_Access_Msk
#endif
    uint32_t lsr = DWT->LSR;

    if ((lsr & DWT_LSR_Present_Msk) != 0) {
        if (ena) {
            if ((lsr & DWT_LSR_Access_Msk) != 0) { //locked
                DWT->LAR = 0xC5ACCE55;
            }
        } else {
            if ((lsr & DWT_LSR_Access_Msk) == 0) { //unlocked
                DWT->LAR = 0;
            }
        }
    }
#else /* __CORTEX_M */
    UNUSED(ena);
#endif /* __CORTEX_M */
}

static uint32_t dwt_init(void)
{

    /* Enable use of DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Unlock */
    dwt_access(true);

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

    /* 3 NO OPERATION instructions */
    __asm volatile(" nop      \n\t"
                   " nop      \n\t"
                   " nop      \n\t");

    /* Check if clock cycle counter has started */
    return (DWT->CYCCNT) ? 0 : 1;
}
#endif /* DWT_BASE */

static void init(void)
{
    /* Deinit HAL just in case => reset all peripherals */
    //HAL_DeInit();

    /* Init DWT if present */
#ifdef DWT_BASE
    if (dwt_init()) {
        Error_Handler();
    }
#endif

    /* Configure the system clock */
    system_clock_config();

    ms_timer_init();

    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_reset(CRM_IOMUX_PERIPH_RESET, TRUE);
    crm_periph_reset(CRM_IOMUX_PERIPH_RESET, FALSE);
    crm_periph_reset(CRM_EXINT_PERIPH_RESET, TRUE);
    crm_periph_reset(CRM_EXINT_PERIPH_RESET, FALSE);
}

void copy_functions_to_ram(void)
{
    /* Load functions into RAM */
    extern uint8_t ram_code_start;
    extern uint8_t ram_code_end;
    extern uint8_t ram_code;
    memcpy(&ram_code_start, &ram_code, (size_t) (&ram_code_end - &ram_code_start));
}

void init_dma_ram(void)
{
  /* Zero DMA memory */
  extern uint8_t _sdma;
  extern uint8_t _edma;
  memset(&_sdma, 0, (size_t) (&_edma - &_sdma));
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need HAL may fail.
__attribute__((constructor)) void premain(void)
{
#if !USE_DEFAULT_LD_SCRIPT
    copy_functions_to_ram();
    init_dma_ram();

    /* Reset vector location which is set wrongly by SystemInit */
    extern uint32_t isr_vector_table_base;
    SCB->VTOR = (__IO uint32_t) &isr_vector_table_base;
    (void)SCB->VTOR;
#endif

    hw_init();

    // Required by FreeRTOS, see http://www.freertos.org/RTOS-Cortex-M3-M4.html
#ifdef NVIC_PRIORITYGROUP_4
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
#endif
#if (__CORTEX_M == 0x07U)
    // Defined in CMSIS core_cm7.h
#ifndef I_CACHE_DISABLED
    SCB_EnableICache();
#endif
#ifndef D_CACHE_DISABLED
    SCB_EnableDCache();
#endif
#endif

    init();
}

extern void initVariant(void) __attribute__((weak));

/*
 * \brief Main entry point of Arduino application
 */
int main(void)
{
    initVariant();
    setup();
    for (;;) {
        loop();
    }

    return 0;
}
