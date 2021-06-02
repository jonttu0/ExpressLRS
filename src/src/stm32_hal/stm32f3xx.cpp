#include "Arduino.h"
#include "stm32_def.h"
#include "priorities.h"
#include "platform_internal.h"

#ifdef STM32F3xx

#include <stm32f3xx_ll_dma.h>


uint32_t dma_get(uint32_t periph, uint8_t type, uint8_t index)
{
#ifdef UART5_BASE
    if (periph == UART5_BASE)
        return 0;
#endif
#if defined(UART4_BASE) && defined(DMA2_BASE)
    else if (periph == UART4_BASE)
        return DMA2_BASE;
#endif
    return DMA1_BASE;
}

uint32_t dma_channel_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_5 : LL_DMA_CHANNEL_4;
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_6 : LL_DMA_CHANNEL_7;
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_3 : LL_DMA_CHANNEL_2;
#if defined(UART4_BASE) && defined(DMA2_BASE)
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_3 : LL_DMA_CHANNEL_5;
#endif
    return 0xff;
}

uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? DMA1_Channel5_IRQn : DMA1_Channel4_IRQn;
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? DMA1_Channel6_IRQn : DMA1_Channel7_IRQn;
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? DMA1_Channel3_IRQn : DMA1_Channel2_IRQn;
#if defined(UART4_BASE) && defined(DMA2_BASE)
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? DMA2_Channel3_IRQn : DMA2_Channel5_IRQn;
#endif
    return 0xff;
}

void dma_request_config(uint32_t periph, uint8_t type, uint8_t index)
{
    /* not needed */
}

uint32_t uart_peripheral_get(uint32_t pin)
{
    switch (pin) {
        case GPIO('A', 9):
        case GPIO('A', 10):
        case GPIO('B', 6):
        case GPIO('B', 7):
        case GPIO('C', 4):
        case GPIO('C', 5):
            return (uint32_t)USART1_BASE;
        case GPIO('A', 2):
        case GPIO('A', 3):
        case GPIO('A', 14):
        case GPIO('A', 15):
        case GPIO('B', 3):
        case GPIO('B', 4):
        case GPIO('D', 5):
        case GPIO('D', 6):
            return (uint32_t)USART2_BASE;
        case GPIO('B', 10):
        case GPIO('B', 11):
        case GPIO('C', 10):
        case GPIO('C', 11):
        case GPIO('D', 8):
        case GPIO('D', 9):
            return (uint32_t)USART3_BASE;
    }
    return 0;
}

uint8_t uart_pin_is_tx(int32_t const pin)
{
    switch (pin) {
        case GPIO('A', 2):
        case GPIO('A', 9):
        case GPIO('A', 14):
        case GPIO('B', 3):
        case GPIO('B', 6):
        case GPIO('B', 9):
        case GPIO('B', 10):
        case GPIO('C', 4):
        case GPIO('C', 6):
        case GPIO('C', 10):
        return 1;
    }
    return 0;
}

uint8_t uart_tx_pin_is_rx(int32_t const pin)
{
    switch (pin) {
        case GPIO('A', 3):
        case GPIO('A', 10):
        case GPIO('A', 15):
        case GPIO('B', 4):
        case GPIO('B', 7):
        case GPIO('B', 8):
        case GPIO('B', 11):
        case GPIO('C', 5):
        case GPIO('C', 11):
            return 1;
    }
    return 0;
}

void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin)
{
    uint32_t afio = GPIO_FUNCTION(7);
#ifdef UART4_BASE
    if (periph == UART4_BASE)
        afio = GPIO_FUNCTION(5);
#endif
#ifdef UART5_BASE
    if (periph == UART5_BASE)
        afio = GPIO_FUNCTION(5);
#endif
    if (rx_pin != tx_pin && rx_pin != (uint32_t)-1)
        gpio_peripheral(rx_pin, afio, 1);
    gpio_peripheral(tx_pin, afio, 0);
}

// Enable a peripheral clock
void enable_pclock(uint32_t periph_base)
{
    if (is_enabled_pclock(periph_base))
        return;

    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
        RCC->APB1ENR |= (1 << pos);
        RCC->APB1ENR;
    } else if (periph_base < AHB1PERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        RCC->APB2ENR |= (1 << pos);
        RCC->APB2ENR;
    } else {
        uint32_t pos = (periph_base - AHB1PERIPH_BASE) / 0x400;
        RCC->AHBENR |= (1 << pos);
        RCC->AHBENR;
    }
}

// Check if a peripheral clock has been enabled
uint32_t is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
        return RCC->APB1ENR & (1 << pos);
    } else if (periph_base < AHB1PERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        return RCC->APB2ENR & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHB1PERIPH_BASE) / 0x400;
        return RCC->AHBENR & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return CONFIG_CLOCK_FREQ / 2;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - AHB2PERIPH_BASE) / 0x400;
    RCC->AHBENR |= 1 << (17 + rcc_pos);
    (void)RCC->AHBENR;
}

// Set the mode and extended function of a pin
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];
    GPIO_InitTypeDef init = {0};
    if (!regs)
        return;

    // Enable GPIO clock
    gpio_clock_enable(regs);

    init.Pin = GPIO2BIT(gpio);
    init.Speed = GPIO_SPEED_FREQ_MEDIUM;
    init.Pull = (!pullup) ? GPIO_NOPULL : ((pullup < 0) ? GPIO_PULLDOWN : GPIO_PULLUP);
    if (mode == GPIO_INPUT) {
        init.Mode = GPIO_MODE_INPUT;
    } else if (mode == GPIO_OUTPUT) {
        init.Mode = GPIO_MODE_OUTPUT_PP;
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        init.Mode = GPIO_MODE_OUTPUT_OD;
    } else if (mode == GPIO_ANALOG) {
        init.Mode = GPIO_MODE_ANALOG;
    } else {
        init.Alternate = (mode >> 4);
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        if (mode & GPIO_OPEN_DRAIN) {
            init.Mode = GPIO_MODE_AF_OD;
        } else {
            init.Mode = GPIO_MODE_AF_PP;
        }
    }
    HAL_GPIO_Init(regs, &init);
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

void timer_init(void)
{
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    /*
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    */

    // Enable SysTick
    NVIC_SetPriority(SysTick_IRQn, ISR_PRIO_TICKS);
    SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000UL) - 1;
    SysTick->VAL = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}


void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Initializes the CPU, AHB and APB busses clocks */
#if !USE_INTERNAL_XO && defined(HSE_VALUE)
#define CPU_CLK_MAX 72000000LU
#if (CPU_CLK_MAX / HSE_VALUE) < 2
#error "Invalid HSE VALUE!"
#endif
    // CPU_CLK to 72MHz
    uint32_t PLLMUL = CPU_CLK_MAX / HSE_VALUE;
    if (16 < PLLMUL)
        PLLMUL = 16; // limit to max
    PLLMUL = (PLLMUL - 2) << RCC_CFGR_PLLMUL_Pos;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = PLLMUL;
#else // USE_INTERNAL_XO
#if HSI_VALUE != 8000000
#error "Wrong config! HSI VALUE is 8MHz!"
#endif
    // CPU_CLK to 64MHz (max)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // HSI = 8MHz
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; // 8MHz / DIV2 = 4MHz
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // 16 * 4MHz = 64MHz
#endif // !USE_INTERNAL_XO
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    /* Flash latency = (CpuClock / 24) - 1  = 2WS */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
#if defined(RCC_USART1CLKSOURCE_PCLK2)
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
#else
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
#endif
#ifdef RCC_PERIPHCLK_USART2
    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();

    /* Enable SYSCFG Clock */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

void hw_init(void)
{
    /* Configure Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

extern "C" {
void NMI_Handler(void) {}
void HardFault_Handler(void) {Error_Handler();}
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
// void SysTick_Handler(void) {Error_Handler();}
void WWDG_IRQHandler(void) {Error_Handler();}
void PVD_IRQHandler(void) {Error_Handler();}
void RTC_WKUP_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void RCC_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 EXTI0_IRQHandler(void) {GPIO_EXTI_IRQHandler(0);}
void FAST_CODE_1 EXTI1_IRQHandler(void) {GPIO_EXTI_IRQHandler(1);}
void FAST_CODE_1 EXTI2_TSC_IRQHandler(void) {GPIO_EXTI_IRQHandler(2);}
void FAST_CODE_1 EXTI3_IRQHandler(void) {GPIO_EXTI_IRQHandler(3);}
void FAST_CODE_1 EXTI4_IRQHandler(void) {GPIO_EXTI_IRQHandler(4);}
void FAST_CODE_1 EXTI9_5_IRQHandler(void)
{
    uint8_t pin;
    for (pin = 5; pin <= 9; pin++) {
        GPIO_EXTI_IRQHandler(pin);
    }
}
void FAST_CODE_1 EXTI15_10_IRQHandler(void)
{
    uint8_t pin;
    for (pin = 10; pin <= 15; pin++) {
        GPIO_EXTI_IRQHandler(pin);
    }
}

void DMA1_Channel1_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA1_Channel2_IRQHandler(void) {USARTx_DMA_handler(2);} // USART3 TX
void DMA1_Channel3_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA1_Channel4_IRQHandler(void) {USARTx_DMA_handler(0);} // USART1 TX
void DMA1_Channel5_IRQHandler(void) {Error_Handler();}
void DMA1_Channel6_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA1_Channel7_IRQHandler(void) {USARTx_DMA_handler(1);} // USART2 TX

#if defined(DMA2_BASE)
void DMA2_Channel1_IRQHandler(void) {Error_Handler();}
void DMA2_Channel2_IRQHandler(void) {Error_Handler();}
void DMA2_Channel3_IRQHandler(void) {Error_Handler();}
void DMA2_Channel4_IRQHandler(void) {Error_Handler();}
#if defined(UART4_BASE)
void FAST_CODE_1 DMA2_Channel5_IRQHandler(void) {USARTx_DMA_handler(3);} // USART4 TX
#else
void DMA2_Channel5_IRQHandler(void) {Error_Handler();}
#endif
#endif // DMA2_BASE

//void TIM2_IRQHandler(void) {Error_Handler();}
void TIM3_IRQHandler(void) {Error_Handler();}
void TIM4_IRQHandler(void) {Error_Handler();}
void TIM6_DAC_IRQHandler(void) {Error_Handler();}
void TIM7_IRQHandler(void) {Error_Handler();}

void I2C1_EV_IRQHandler(void) {Error_Handler();}
void I2C1_ER_IRQHandler(void) {Error_Handler();}
void I2C2_EV_IRQHandler(void) {Error_Handler();}
void I2C2_ER_IRQHandler(void) {Error_Handler();}

void SPI1_IRQHandler(void) {Error_Handler();}
void SPI2_IRQHandler(void) {Error_Handler();}
void SPI3_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 USART1_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(0);
}
void FAST_CODE_1 USART2_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(1);
}
void FAST_CODE_1 USART3_IRQHandler(void)
{
    FAST_CODE_1 USART_IDLE_IRQ_handler(2);
}
#ifdef UART4_BASE
void FAST_CODE_1 UART4_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(3);
}
#endif
#ifdef UART5_BASE
void FAST_CODE_1 UART5_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(4);
}
#endif
} // extern

#endif /* STM32F3xx */
