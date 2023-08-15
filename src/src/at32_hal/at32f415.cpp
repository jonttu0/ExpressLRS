#ifndef HEXT_VALUE
// Must be defined here to avoid an issue with precompiler:
//   error: missing binary operator before token "8000000"
#define HEXT_VALUE 8000000U
#endif

#include "Arduino.h"
#include "at32_def.h"
#include "priorities.h"
#include "platform.h"

#ifdef AT32F415

#include <at32f415.h>
#include <at32f415_dma.h>


void* dma_get(uint32_t const periph, uint8_t const type, uint8_t const index)
{
#ifdef UART4_BASE
    if (periph == UART4_BASE)
        return DMA2;
#endif
    return DMA1;
}

void* dma_channel_get(uint32_t const periph, uint8_t const type, uint8_t const index)
{
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? DMA1_CHANNEL5 : DMA1_CHANNEL4;
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? DMA1_CHANNEL6 : DMA1_CHANNEL7;
#ifdef USART3_BASE
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? DMA1_CHANNEL3 : DMA1_CHANNEL2;
#endif
#ifdef UART4_BASE
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? DMA2_CHANNEL3 : DMA2_CHANNEL5;
#endif
    return 0;
}

uint8_t dma_channel_idx_get(uint32_t const periph, uint8_t const type, uint8_t const index)
{
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? 5 : 4;
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? 6 : 7;
#ifdef USART3_BASE
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? 3 : 2;
#endif
#ifdef UART4_BASE
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? 3 : 5;
#endif
    return UINT8_MAX;
}

uint32_t dma_irq_get(uint32_t const periph, uint8_t const type, uint8_t const index)
{
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? DMA1_Channel5_IRQn : DMA1_Channel4_IRQn;
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? DMA1_Channel6_IRQn : DMA1_Channel7_IRQn;
#ifdef USART3_BASE
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? DMA1_Channel3_IRQn : DMA1_Channel2_IRQn;
#endif
#ifdef UART4_BASE
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? DMA2_Channel3_IRQn : DMA2_Channel4_5_IRQn;
#endif
    return 0xff;
}

void dma_request_config(uint32_t const periph, uint8_t const type, uint8_t const index)
{
    /*LL_DMA_SetPeriphRequest(
        (DMA_TypeDef *)dma_get(periph, type, index),
        dma_channel_get(periph, type, index),
        LL_DMA_REQUEST_2);
        */
}

uint32_t uart_peripheral_get(uint32_t const pin)
{
    switch (pin) {
        case GPIO('A', 10):
        case GPIO('A', 9):
        case GPIO('B', 7):
        case GPIO('B', 6):
            return (uint32_t)USART1_BASE;
        case GPIO('A', 3):
        case GPIO('A', 2):
            return (uint32_t)USART2_BASE;
#ifdef USART3_BASE
        case GPIO('A', 6):
        case GPIO('A', 7):
        case GPIO('B', 10):
        case GPIO('B', 11):
        case GPIO('C', 10):
        case GPIO('C', 11):
            return (uint32_t)USART3_BASE;
#endif
#ifdef UART4_BASE
        case GPIO('F', 4):
        case GPIO('F', 5):
            return (uint32_t)UART4_BASE;
#endif
#ifdef UART5_BASE
        case GPIO('C', 12):
        case GPIO('D', 2):
            return (uint32_t)UART5_BASE;
#endif
    }
    return 0;
}

uint8_t uart_pin_is_tx(int32_t const pin)
{
    switch (pin) {
        case GPIO('A', 2):
        case GPIO('A', 9):
        case GPIO('B', 6):
#ifdef USART3_BASE
        case GPIO('A', 7):
        case GPIO('B', 10):
        case GPIO('C', 10):
#endif
#ifdef UART4_BASE
        case GPIO('F', 4):
#endif
#ifdef UART5_BASE
        case GPIO('C', 12):
#endif
            return 1;
    }
    return 0;
}

void uart_config_afio(uint32_t const periph, uint32_t const rx_pin, uint32_t const tx_pin)
{
    if (rx_pin != tx_pin && rx_pin != (uint32_t)-1)
        gpio_peripheral(rx_pin, GPIO_AF, 1);
    gpio_peripheral(tx_pin, GPIO_AF, 0);

    if (periph == USART1_BASE) {
        if ((rx_pin == GPIO('B', 7)) || (tx_pin == GPIO('B', 6)))
            gpio_pin_remap_config(USART1_MUX, TRUE);
#ifdef USART3_BASE
    } else if (periph == USART3_BASE) {
        if ((rx_pin == GPIO('C', 11)) || (tx_pin == GPIO('C', 10)))
            gpio_pin_remap_config(USART3_MUX_01, TRUE);
        else if ((rx_pin == GPIO('A', 6)) || (tx_pin == GPIO('A', 7)))
            gpio_pin_remap_config(USART3_MUX_10, TRUE);
#endif
#ifdef UART4_BASE
    } else if (periph == UART4_BASE) {
        if ((rx_pin == GPIO('F', 5)) || (tx_pin == GPIO('F', 4)))
            gpio_pin_remap_config(UART4_GMUX_0001, TRUE);
    }
#endif
}

// Enable a peripheral clock
void enable_pclock(uint32_t const periph_base)
{
    if (is_enabled_pclock(periph_base))
        return;

    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
        CRM->apb1en |= (1 << pos);
        CRM->apb1en;
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        CRM->apb2en |= (1 << pos);
        CRM->apb2en;
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        CRM->ahben |= (1 << pos);
        CRM->ahben;
    }
}

// Check if a peripheral clock has been enabled
uint32_t is_enabled_pclock(uint32_t const periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
        return CRM->apb1en & (1 << pos);
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        return CRM->apb2en & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        return CRM->ahben & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t const periph_base)
{
    return CONFIG_CLOCK_FREQ / 2;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(gpio_type *regs)
{
    uint32_t const io = ((uintptr_t)regs - GPIOA_BASE) / 0x400;
    crm_periph_clock_enable((crm_periph_clock_type)(CRM_GPIOA_PERIPH_CLOCK + io), TRUE);

    //uint32_t rcc_pos = ((uint32_t)regs - APB2PERIPH_BASE) / 0x400;
    //CRM->apb2en |= 1 << rcc_pos;
    //(void)CRM->apb2en;

    // Enable EXTI cloks as well for GPIO ISR handling
    //enable_pclock(EXINT_BASE);
}

// Set the mode and extended function of a pin
void gpio_peripheral(uint32_t const gpio, uint32_t const mode, int const pullup)
{
    gpio_type *regs = digital_regs[GPIO2PORT(gpio)];
    gpio_init_type init = {0};
    if (!regs)
        return;

    // Enable GPIO clock
    gpio_clock_enable(regs);

    init.gpio_pins = GPIO2BIT(gpio);
    init.gpio_pull = (!pullup) ? GPIO_PULL_NONE : ((pullup < 0) ? GPIO_PULL_DOWN : GPIO_PULL_UP);
    init.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    init.gpio_out_type = (mode & GPIO_OPEN_DRAIN) ? GPIO_OUTPUT_OPEN_DRAIN : GPIO_OUTPUT_PUSH_PULL;
    if (mode == GPIO_INPUT) {
        init.gpio_mode = GPIO_MODE_INPUT;
    } else if (mode == GPIO_OUTPUT || mode == (GPIO_OUTPUT + GPIO_OPEN_DRAIN)) {
        init.gpio_mode = GPIO_MODE_OUTPUT;
    } else if (mode == GPIO_ANALOG) {
        init.gpio_mode = GPIO_MODE_ANALOG;
    } else {
        init.gpio_mode = GPIO_MODE_MUX;
    }
    gpio_init(regs, &init);
}

// Return the current time (in absolute clock ticks).
uint32_t FAST_CODE_1 timer_read_time(void)
{
    return DWT->CYCCNT;
}

uint32_t FAST_CODE_1 micros(void)
{
    return clockCyclesToMicroseconds(timer_read_time());
}

void FAST_CODE_1 delayMicroseconds(uint32_t usecs)
{
    //uint32_t end = timer_read_time() + microsecondsToClockCycles(usecs);
    //while (timer_is_before(timer_read_time(), end))
    //    ;
    usecs = microsecondsToClockCycles(usecs);
    uint32_t const start = timer_read_time();
    while ((timer_read_time() - start) < usecs);
}

void ms_timer_init(void)
{
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    /*
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    */

    // Enable SysTick
    NVIC_SetPriority(SysTick_IRQn, ISR_PRIO_TICKS);
    SysTick->LOAD = (uint32_t)(system_core_clock / 1000UL) - 1;
    SysTick->VAL = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}


void system_clock_config(void)
{
    /* enable pwc periph clock */
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

    /* config ldo voltage */
    pwc_pvm_level_select(PWC_PVM_VOLTAGE_2V3);

    /* set the flash clock divider. 4 for 150MHz */
    flash_psr_set(FLASH_WAIT_CYCLE_4);

    /* reset crm */
    crm_reset();

#if USE_INTERNAL_XO
    /* enable hick */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

    /* wait till hick is ready */
    while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET) {
    }

    /* config pll clock resource */
    crm_pll_config2(CRM_PLL_SOURCE_HICK, 75, 1, CRM_PLL_FR_2);  // 8MHz * 75 / (1 * 4)

#else
#if HEXT_VALUE != 8000000U
    #error "Invalid EXT XO value!"
#endif

    /* enable hext */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

    /* wait till hext is ready */
    while (crm_hext_stable_wait() == ERROR) {
    }

    /* config pll clock resource */
    crm_pll_config2(CRM_PLL_SOURCE_HEXT, 75, 1, CRM_PLL_FR_4);  // 8MHz * 75 / (1 * 4)
#endif

    /* enable pll */
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

    /* wait till pll is ready */
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
    }

    /* config ahbclk. Max 150MHz */
    crm_ahb_div_set(CRM_AHB_DIV_1);

    /* config apb2clk. Max 75MHz */
    crm_apb2_div_set(CRM_APB2_DIV_2);

    /* config apb1clk. Max 75MHz */
    crm_apb1_div_set(CRM_APB1_DIV_2);

    /* enable auto step mode */
    crm_auto_step_mode_enable(TRUE);

    /* select pll as system clock source */
    crm_sysclk_switch(CRM_SCLK_PLL);

    /* wait till pll is used as system clock source */
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
    }

    /* disable auto step mode */
    crm_auto_step_mode_enable(FALSE);

    /* update system_core_clock global variable */
    system_core_clock_update();
}

void hw_init(void)
{
    /* Configure Flash prefetch */
    FLASH->psr_bit.pft_enf = 1;
    FLASH->psr_bit.pft_en = 1;
}

extern "C" {
void NMI_Handler(void) {}
void HardFault_Handler(void) {Error_Handler();}
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
// void SysTick_Handler(void) {Error_Handler();}
void WWDT_IRQHandler(void) {Error_Handler();}
void PVM_IRQHandler(void) {Error_Handler();}
void TAMPER_IRQHandler(void) {Error_Handler();}
void ERTC_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void CRM_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 EXINT0_IRQHandler(void) {gpio_exti_irq_handler(0);}
void FAST_CODE_1 EXINT1_IRQHandler(void) {gpio_exti_irq_handler(1);}
void FAST_CODE_1 EXINT2_IRQHandler(void) {gpio_exti_irq_handler(2);}
void FAST_CODE_1 EXINT3_IRQHandler(void) {gpio_exti_irq_handler(3);}
void FAST_CODE_1 EXINT4_IRQHandler(void) {gpio_exti_irq_handler(4);}
void FAST_CODE_1 EXINT9_5_IRQHandler(void)
{
    uint8_t pin;
    for (pin = 5; pin <= 9; pin++) {
        gpio_exti_irq_handler(pin);
    }
}
void FAST_CODE_1 EXINT15_10_IRQHandler(void)
{
    uint8_t pin;
    for (pin = 10; pin <= 15; pin++) {
        gpio_exti_irq_handler(pin);
    }
}

void DMA1_Channel1_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA1_Channel2_IRQHandler(void) {
#ifdef USART3_BASE
    uart_dma_irq_handler(2);
#else
    Error_Handler();
#endif
}
void DMA1_Channel3_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA1_Channel4_IRQHandler(void) {uart_dma_irq_handler(0);} // USART1
void DMA1_Channel5_IRQHandler(void) {Error_Handler();}
void DMA1_Channel6_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA1_Channel7_IRQHandler(void) {uart_dma_irq_handler(1);} // USART2

void DMA2_Channel1_IRQHandler(void) {Error_Handler();}
void DMA2_Channel2_IRQHandler(void) {Error_Handler();}
void DMA2_Channel3_IRQHandler(void) {Error_Handler();}
void DMA2_Channel4_5_IRQHandler(void) {Error_Handler();}
void DMA2_Channel6_7_IRQHandler(void) {Error_Handler();}

void ADC1_IRQHandler(void) {Error_Handler();}

void TMR1_OVF_TMR10_IRQHandler(void) {Error_Handler();}
//void TMR2_GLOBAL_IRQHandler(void) {Error_Handler();}
void TMR3_GLOBAL_IRQHandler(void) {Error_Handler();}
void TMR4_GLOBAL_IRQHandler(void) {Error_Handler();}
void TMR5_GLOBAL_IRQHandler(void) {Error_Handler();}

void I2C1_EVT_IRQHandler(void) {Error_Handler();}
void I2C1_ERR_IRQHandler(void) {Error_Handler();}
void I2C2_EVT_IRQHandler(void) {Error_Handler();}
void I2C2_ERR_IRQHandler(void) {Error_Handler();}

void SPI1_IRQHandler(void) {Error_Handler();}
void SPI2_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 USART1_IRQHandler(void) {uart_irq_handler(0);}
void FAST_CODE_1 USART2_IRQHandler(void) {uart_irq_handler(1);}
void FAST_CODE_1 USART3_IRQHandler(void) {uart_irq_handler(2);}
void FAST_CODE_1 UART4_IRQHandler(void) {uart_irq_handler(3);}
void FAST_CODE_1 UART5_IRQHandler(void) {uart_irq_handler(4);}

} // extern

#endif
