#include "Arduino.h"
#include "stm32_def.h"
#include "priorities.h"
#include "platform_internal.h"

#ifdef STM32F1xx

#include <stm32f1xx_ll_dma.h>

uint32_t dma_get(uint32_t periph, uint8_t type, uint8_t index)
{
    return DMA1_BASE;
}

uint32_t dma_channel_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE) {
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_5 : LL_DMA_CHANNEL_4;
    } else if (periph == USART2_BASE) {
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_7 : LL_DMA_CHANNEL_6;
    } else if (periph == USART3_BASE) {
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_3 : LL_DMA_CHANNEL_2;
    }
    return 0xff;
}

uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE) {
        return (type == DMA_USART_RX) ? DMA1_Channel5_IRQn : DMA1_Channel4_IRQn;
    } else if (periph == USART2_BASE) {
        return (type == DMA_USART_RX) ? DMA1_Channel7_IRQn : DMA1_Channel6_IRQn;
    } else if (periph == USART3_BASE) {
        return (type == DMA_USART_RX) ? DMA1_Channel3_IRQn : DMA1_Channel2_IRQn;
    }
    return 0xff;
}

void dma_request_config(uint32_t periph, uint8_t type, uint8_t index)
{
}

uint32_t uart_peripheral_get(uint32_t pin)
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
        case GPIO('D', 9):
        case GPIO('D', 8):
        case GPIO('B', 11):
        case GPIO('B', 10):
            return (uint32_t)USART3_BASE;
    }
    return 0;
}

uint8_t uart_pin_is_tx(int32_t const pin)
{
    switch (pin) {
        case GPIO('A', 2):
        case GPIO('A', 9):
        case GPIO('B', 6):
        case GPIO('B', 10):
        case GPIO('D', 8):
        return 1;
    }
    return 0;
}

void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin)
{
    (void)periph;
    if (rx_pin != tx_pin && rx_pin != (uint32_t)-1)
        gpio_peripheral(rx_pin, GPIO_FUNCTION(7), 1);
    gpio_peripheral(tx_pin, GPIO_FUNCTION(7), 0);
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
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        RCC->APB2ENR |= (1 << pos);
        RCC->APB2ENR;
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
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
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        return RCC->APB2ENR & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        return RCC->AHBENR & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return (CONFIG_CLOCK_FREQ / 2);
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= 1 << rcc_pos;
    RCC->APB2ENR;
}

// Set the mode and extended function of a pin
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];
    if (!regs)
        return;

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t pos = gpio % 16, shift = (pos % 8) * 4, msk = 0xf << shift, cfg;
    if (mode == GPIO_INPUT) {
        cfg = pullup ? 0x8 : 0x4;
    } else if (mode == GPIO_OUTPUT) {
        cfg = 0x1; // push-pull, 0b00 | max speed 2 MHz, 0b01
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        cfg = 0x5; // Open-drain, 0b01 | max speed 2 MHz, 0b01
    } else if (mode == GPIO_ANALOG) {
        cfg = 0x0;
    } else {
        // Alternate function
        if (mode & GPIO_OPEN_DRAIN)
            // output open-drain mode, 10MHz
            cfg = 0xd;
            // output open-drain mode, 50MHz
            //cfg = 0xF;
        else if (pullup > 0)
            // input pins use GPIO_INPUT mode on the stm32f1
            cfg = 0x8;
        else
            // output push-pull mode, 10MHz
            cfg = 0x9;
            // output push-pull mode, 50MHz
            //cfg = 0xB;
    }
    if (pos & 0x8)
        regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
    else
        regs->CRL = (regs->CRL & ~msk) | (cfg << shift);

    if (pullup > 0)
        regs->BSRR = 1 << pos;
    else if (pullup < 0)
        regs->BSRR = 1 << (pos + 16);

    if (gpio == GPIO('A', 13) || gpio == GPIO('A', 14))
        // Disable SWD to free PA13, PA14
        //AFIO->MAPR = AFIO_MAPR_SWJ_CFG_DISABLE;
        __HAL_AFIO_REMAP_SWJ_DISABLE();
    else if ((gpio == GPIO('A', 15)) || (gpio == GPIO('B', 3)) || (gpio == GPIO('B', 4)))
        // Disable JTAG-DP
        __HAL_AFIO_REMAP_SWJ_NOJTAG();
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
    #if 0
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    #endif

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
    SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000UL) - 1;
    SysTick->VAL = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}


void system_clock_config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

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
    PLLMUL = (PLLMUL - 2) << RCC_CFGR_PLLMULL_Pos;

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
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // 8MHz / 2 = 4MHz
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // 4MHz * 16 = 64MHz
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

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();

    __HAL_RCC_AFIO_CLK_ENABLE();
}

void hw_init(void)
{
/* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0)
#if defined(STM32F101x6) || defined(STM32F101xB) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F102x6) || defined(STM32F102xB) || \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)

  /* Prefetch buffer is not available on value line devices */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
#endif /* PREFETCH_ENABLE */
}

extern "C" {

void NMI_Handler(void) {}
void HardFault_Handler(void) {Error_Handler();}
void MemManage_Handler(void) {Error_Handler();}
void BusFault_Handler(void) {Error_Handler();}
void UsageFault_Handler(void) {Error_Handler();}
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
// void SysTick_Handler(void) {Error_Handler();}
void WWDG_IRQHandler(void) {Error_Handler();}
void PVD_IRQHandler(void) {Error_Handler();}
void TAMPER_IRQHandler(void) {Error_Handler();}
void RTC_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void RCC_IRQHandler(void) {Error_Handler();}
void DMA1_Channel1_IRQHandler(void) {Error_Handler();}
//void DMA1_Channel2_IRQHandler(void) {Error_Handler();}
void DMA1_Channel3_IRQHandler(void) {Error_Handler();}
//void DMA1_Channel4_IRQHandler(void) {Error_Handler();}
void DMA1_Channel5_IRQHandler(void) {Error_Handler();}
//void DMA1_Channel6_IRQHandler(void) {Error_Handler();}
void DMA1_Channel7_IRQHandler(void) {Error_Handler();}
void ADC1_2_IRQHandler(void) {Error_Handler();}
void USB_HP_CAN1_TX_IRQHandler(void) {Error_Handler();}
void USB_LP_CAN1_RX0_IRQHandler(void) {Error_Handler();}
void CAN1_RX1_IRQHandler(void) {Error_Handler();}
void CAN1_SCE_IRQHandler(void) {Error_Handler();}
void TIM1_BRK_IRQHandler(void) {Error_Handler();}
void TIM1_UP_IRQHandler(void) {Error_Handler();}
void TIM1_TRG_COM_IRQHandler(void) {Error_Handler();}
void TIM1_CC_IRQHandler(void) {Error_Handler();}
//void TIM2_IRQHandler(void) {Error_Handler();}
void TIM3_IRQHandler(void) {Error_Handler();}
void TIM4_IRQHandler(void) {Error_Handler();}
void I2C1_EV_IRQHandler(void) {Error_Handler();}
void I2C1_ER_IRQHandler(void) {Error_Handler();}
void I2C2_EV_IRQHandler(void) {Error_Handler();}
void I2C2_ER_IRQHandler(void) {Error_Handler();}
void SPI1_IRQHandler(void) {Error_Handler();}
void SPI2_IRQHandler(void) {Error_Handler();}
void RTC_Alarm_IRQHandler(void) {Error_Handler();}
void USBWakeUp_IRQHandler(void) {Error_Handler();}


/* USART1 TX DMA */
void FAST_CODE_1 DMA1_Channel4_IRQHandler(void)
{
    USARTx_DMA_handler(0);
}

/* USART2 TX DMA */
void FAST_CODE_1 DMA1_Channel6_IRQHandler(void)
{
    USARTx_DMA_handler(1);
}

/* USART3 TX DMA */
void FAST_CODE_1 DMA1_Channel2_IRQHandler(void)
{
    USARTx_DMA_handler(2);
}

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

/**
 * @brief This function handles external line 0 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI0_IRQHandler(void)
{
    GPIO_EXTI_IRQHandler(0);
}

/**
 * @brief This function handles external line 1 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI1_IRQHandler(void)
{
    GPIO_EXTI_IRQHandler(1);
}

/**
 * @brief This function handles external line 2 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI2_IRQHandler(void)
{
    GPIO_EXTI_IRQHandler(2);
}

/**
 * @brief This function handles external line 3 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI3_IRQHandler(void)
{
    GPIO_EXTI_IRQHandler(3);
}

/**
 * @brief This function handles external line 4 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI4_IRQHandler(void)
{
    GPIO_EXTI_IRQHandler(4);
}

/**
 * @brief This function handles external line 5 to 9 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI9_5_IRQHandler(void)
{
    uint8_t pin;
    for (pin = 5; pin <= 9; pin++)
    {
        GPIO_EXTI_IRQHandler(pin);
    }
}

/**
 * @brief This function handles external line 10 to 15 interrupt request.
 * @param  None
 * @retval None
 */
void FAST_CODE_1 EXTI15_10_IRQHandler(void)
{
    uint8_t pin;
    for (pin = 10; pin <= 15; pin++)
    {
        GPIO_EXTI_IRQHandler(pin);
    }
}

} // extern "C"

#endif /* STM32F1xx */
