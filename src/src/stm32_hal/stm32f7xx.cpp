#include "Arduino.h"
#include "stm32_def.h"
#include "priorities.h"
#include "platform.h"

#ifdef STM32F7xx

#include <stm32f7xx_ll_dma.h>

// https://www.mouser.fi/datasheet/2/389/stm32f722ic-1851054.pdf
// https://www.st.com/resource/en/reference_manual/dm00305990-stm32f72xxx-and-stm32f73xxx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf



uint32_t dma_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE || periph == USART6_BASE)
        return DMA2_BASE;
    return DMA1_BASE;
}

uint32_t dma_channel_get(uint32_t periph, uint8_t type, uint8_t index)
{
    // Return stream instead
    /* DMA2 */
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_STREAM_2 : LL_DMA_STREAM_7;
    else if (periph == USART6_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_STREAM_1 : LL_DMA_STREAM_6;
    /* DMA1 */
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_STREAM_5 : LL_DMA_STREAM_6;
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_STREAM_1 : LL_DMA_STREAM_3;
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_STREAM_2 : LL_DMA_STREAM_4;
    else if (periph == UART5_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_STREAM_0 : LL_DMA_STREAM_7;
    return 0xff;
}

uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index)
{
    /* DMA2 */
    if (periph == USART1_BASE)
        return (type == DMA_USART_RX) ? DMA2_Stream2_IRQn : DMA2_Stream7_IRQn;
    else if (periph == USART6_BASE)
        return (type == DMA_USART_RX) ? DMA2_Stream1_IRQn : DMA2_Stream6_IRQn;
    /* DMA1 */
    else if (periph == USART2_BASE)
        return (type == DMA_USART_RX) ? DMA1_Stream5_IRQn : DMA1_Stream6_IRQn;
    else if (periph == USART3_BASE)
        return (type == DMA_USART_RX) ? DMA1_Stream1_IRQn : DMA1_Stream3_IRQn;
    else if (periph == UART4_BASE)
        return (type == DMA_USART_RX) ? DMA1_Stream2_IRQn : DMA1_Stream4_IRQn;
    else if (periph == UART5_BASE)
        return (type == DMA_USART_RX) ? DMA1_Stream0_IRQn : DMA1_Stream7_IRQn;
    return 0xff;
}

void dma_request_config(uint32_t periph, uint8_t type, uint8_t index)
{
    // Link channel (peripheral) to stream
    uint32_t channel = LL_DMA_CHANNEL_4;
    if (periph == USART6_BASE)
        channel = LL_DMA_CHANNEL_5;
    LL_DMA_SetChannelSelection(
        (DMA_TypeDef *)dma_get(periph, type, index),
        dma_channel_get(periph, type, index),
        channel);
}

uint32_t uart_peripheral_get(uint32_t pin)
{
    switch (pin) {
        case GPIO('A', 9):
        case GPIO('A', 10):
        case GPIO('B', 6):
        case GPIO('B', 7):
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
        case GPIO('A', 0):
        case GPIO('A', 1):
            return (uint32_t)UART4_BASE;
        case GPIO('C', 12):
        case GPIO('D', 2):
            return (uint32_t)UART5_BASE;
        case GPIO('C', 6):
        case GPIO('C', 7):
            return (uint32_t)USART6_BASE;
    }
    return 0;
}

uint8_t uart_pin_is_tx(int32_t const pin)
{
    switch (pin) {
        case GPIO('A', 0):
        case GPIO('A', 2):
        case GPIO('A', 9):
        case GPIO('A', 14):
        case GPIO('B', 3):
        case GPIO('B', 6):
        case GPIO('B', 10):
        case GPIO('C', 6):
        case GPIO('C', 10):
        case GPIO('C', 12):
        case GPIO('D', 5):
        case GPIO('D', 8):
        return 1;
    }
    return 0;
}

void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin)
{
    uint32_t afio = GPIO_FUNCTION(7);
    if (periph == UART4_BASE || periph == UART5_BASE || periph == USART6_BASE)
        afio = GPIO_FUNCTION(8);
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
        if (pos == 8) { // ADC
            if (periph_base == ADC2_BASE) pos = 9;
            else if (periph_base == ADC3_BASE) pos = 10;
        }
        RCC->APB2ENR |= (1 << pos);
        RCC->APB2ENR;
    } else if (periph_base < AHB2PERIPH_BASE) {
        uint32_t pos = (periph_base - AHB1PERIPH_BASE) / 0x400;
        if (18 < pos) pos -= 3;
        RCC->AHB1ENR |= (1 << pos);
        RCC->AHB1ENR;
    } else  {
        uint32_t pos = (periph_base - AHB2PERIPH_BASE) / 0x400;
        RCC->AHB2ENR |= (1 << pos);
        RCC->AHB2ENR;
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
        if (pos == 8) { // ADC
            if (periph_base == ADC2_BASE) pos = 9;
            else if (periph_base == ADC3_BASE) pos = 10;
        }
        return RCC->APB2ENR & (1 << pos);
    } else if (periph_base < AHB2PERIPH_BASE) {
        uint32_t pos = (periph_base - AHB1PERIPH_BASE) / 0x400;
        if (18 < pos) pos -= 3;
        return RCC->AHB1ENR & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHB2PERIPH_BASE) / 0x400;
        return RCC->AHB2ENR & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return CONFIG_CLOCK_FREQ / 4;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - AHB1PERIPH_BASE) / 0x400;
    RCC->AHB1ENR |= (1 << rcc_pos);
    (void)RCC->AHB1ENR;
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
    SCB_EnableICache();
    //SCB_DisableICache();
    //SCB_EnableDCache();
    SCB_DisableDCache();

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* PLL f out = ((f_in) * (PLLN / PLLM)) / PLLP */
#define PLL_N     432
#define PLL_P     RCC_PLLP_DIV2
#define PLL_Q     9

    /* Initializes the CPU, AHB and APB busses clocks to be 216MHz */
#if !USE_INTERNAL_XO && defined(HSE_VALUE)
#if HSE_VALUE < 1000000U
#error "Wrong config! HSE VALUE min is 1MHz!"
#elif HSE_VALUE > 63000000U
#error "Wrong config! HSE VALUE max is 63MHz!"
#endif

#define PLL_M (HSE_VALUE / 1000000U)

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = PLL_M;
    RCC_OscInitStruct.PLL.PLLN = PLL_N;
    RCC_OscInitStruct.PLL.PLLP = PLL_P;
    RCC_OscInitStruct.PLL.PLLQ = PLL_Q;
#else // USE_INTERNAL_XO
#if HSI_VALUE != 16000000U
#error "Wrong config! HSI VALUE is 16MHz!"
#endif

#define PLL_M (HSI_VALUE / 1000000U)

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = PLL_M;
    RCC_OscInitStruct.PLL.PLLN = PLL_N;
    RCC_OscInitStruct.PLL.PLLP = PLL_P;
    RCC_OscInitStruct.PLL.PLLQ = PLL_Q;
#endif // !USE_INTERNAL_XO
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Activate the OverDrive to reach the 216 MHz Frequency */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // HCLK
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  // PCLK1, must be <= 54MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;  // PCLK2, must be <= 108MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                                         |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                                         |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                                         |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8
                                         |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3
                                         |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C4;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    PeriphClkInit.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    PeriphClkInit.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
    PeriphClkInit.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();

    /* Enable SYSCFG Clock */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // The timer prescaler
    //   * APBx prescaler is 1,2 or 4, then TIMxCLK = HCLK, otherwise TIMxCLK = 4x PCLKx
    //__HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_ACTIVATED);
    //   * APBx prescaler is 1, then TIMxCLK = PCLKx, otherwise TIMxCLK = 2x PCLKx
    __HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_DESACTIVATED);
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
void RTC_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void RCC_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 EXTI0_IRQHandler(void) {GPIO_EXTI_IRQHandler(0);}
void FAST_CODE_1 EXTI1_IRQHandler(void) {GPIO_EXTI_IRQHandler(1);}
void FAST_CODE_1 EXTI2_IRQHandler(void) {GPIO_EXTI_IRQHandler(2);}
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

void FAST_CODE_1 DMA1_Stream0_IRQHandler(void) {USARTx_DMA_handler(4);} // UART5 RX
void FAST_CODE_1 DMA1_Stream1_IRQHandler(void) {USARTx_DMA_handler(2);} // USART3 RX
void FAST_CODE_1 DMA1_Stream2_IRQHandler(void) {USARTx_DMA_handler(3);} // UART4 RX
void FAST_CODE_1 DMA1_Stream3_IRQHandler(void) {USARTx_DMA_handler(2);} // USART3 TX
void FAST_CODE_1 DMA1_Stream4_IRQHandler(void) {USARTx_DMA_handler(3);} // UART4 TX
void FAST_CODE_1 DMA1_Stream5_IRQHandler(void) {USARTx_DMA_handler(1);} // USART2 RX
void FAST_CODE_1 DMA1_Stream6_IRQHandler(void) {USARTx_DMA_handler(1);} // USART2 TX
void FAST_CODE_1 DMA1_Stream7_IRQHandler(void) {USARTx_DMA_handler(4);} // UART5 TX

//void FAST_CODE_1 DMA2_Stream0_IRQHandler(void) {Error_Handler();} // ADC1 DMA
void FAST_CODE_1 DMA2_Stream1_IRQHandler(void) {USARTx_DMA_handler(5);} // USART6 RX
void FAST_CODE_1 DMA2_Stream2_IRQHandler(void) {USARTx_DMA_handler(0);} // USART1 RX
void DMA2_Stream3_IRQHandler(void) {Error_Handler();} // ADC2 DMA
void DMA2_Stream4_IRQHandler(void) {Error_Handler();}
void DMA2_Stream5_IRQHandler(void) {Error_Handler();}
void FAST_CODE_1 DMA2_Stream6_IRQHandler(void) {USARTx_DMA_handler(5);} // USART6 TX
void FAST_CODE_1 DMA2_Stream7_IRQHandler(void) {USARTx_DMA_handler(0);} // USART1 TX

void TIM1_BRK_TIM9_IRQHandler(void) {Error_Handler();}
void TIM1_UP_TIM10_IRQHandler(void) {Error_Handler();}
void TIM1_TRG_COM_TIM11_IRQHandler(void) {Error_Handler();}
void TIM1_CC_IRQHandler(void) {Error_Handler();}
//void TIM2_IRQHandler(void) {Error_Handler();}
void TIM3_IRQHandler(void) {Error_Handler();}
void TIM4_IRQHandler(void) {Error_Handler();}
void TIM5_IRQHandler(void) {Error_Handler();}
//void TIM6_DAC_IRQHandler(void) {Error_Handler();}
void TIM7_IRQHandler(void) {Error_Handler();}
void TIM8_BRK_TIM12_IRQHandler(void) {Error_Handler();}
void TIM8_UP_TIM13_IRQHandler(void) {Error_Handler();}
void TIM8_TRG_COM_TIM14_IRQHandler(void) {Error_Handler();}
void TIM8_CC_IRQHandler(void) {Error_Handler();}

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
    USART_IDLE_IRQ_handler(2);
}
void FAST_CODE_1 USART6_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(5);
}
void FAST_CODE_1 UART4_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(3);
}
void FAST_CODE_1 UART5_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(4);
}
void FAST_CODE_1 UART7_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(6);
}
void FAST_CODE_1 UART8_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(7);
}

} // extern

#endif /* STM32F7xx */
