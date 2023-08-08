#include "Arduino.h"
#include "stm32_def.h"
#include "priorities.h"
#include "platform.h"

#ifdef STM32G0xx

#include <stm32g0xx_ll_dma.h>
#include <stm32g0xx_ll_cortex.h>

#define APB2PERIPH_BASE (APBPERIPH_BASE + 0x00010000UL)

uint32_t dma_get(uint32_t periph, uint8_t type, uint8_t index)
{
    return DMA1_BASE;
}

uint32_t dma_channel_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE || periph == USART2_BASE || periph == USART3_BASE)
        return LL_DMA_CHANNEL_1;
    return 0xff;
}

uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE || periph == USART2_BASE || periph == USART3_BASE)
        return DMA1_Channel1_IRQn;
    return 0xff;
}

void dma_request_config(uint32_t periph, uint8_t type, uint8_t index)
{
    uint32_t req;
    switch (periph) {
        case USART1_BASE:
            req = (type == DMA_USART_RX) ? LL_DMAMUX_REQ_USART1_RX : LL_DMAMUX_REQ_USART1_TX;
            break;
        case USART2_BASE:
            req = (type == DMA_USART_RX) ? LL_DMAMUX_REQ_USART2_RX : LL_DMAMUX_REQ_USART2_TX;
            break;
        case USART3_BASE:
            req = (type == DMA_USART_RX) ? LL_DMAMUX_REQ_USART3_RX : LL_DMAMUX_REQ_USART3_TX;
            break;
        default:
            return;
    }
    LL_DMA_SetPeriphRequest(
        (DMA_TypeDef *)dma_get(periph, type, index),
        dma_channel_get(periph, type, index),
        req);
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
        case GPIO('A', 14):
        case GPIO('A', 15):
            return (uint32_t)USART2_BASE;
        case GPIO('A', 5): // TX
        case GPIO('B', 0): // RX
        case GPIO('B', 2): // TX
        case GPIO('B', 8): // TX
        case GPIO('B', 9): // RX
        case GPIO('B', 10): // TX
        case GPIO('B', 11): // RX
            return (uint32_t)USART3_BASE;
    }
    return 0;
}

uint8_t uart_pin_is_tx(int32_t const pin)
{
    switch (pin) {
        case GPIO('A', 2):
        case GPIO('A', 5):
        case GPIO('A', 9):
        case GPIO('A', 14):
        case GPIO('A', 15):
        case GPIO('B', 2):
        case GPIO('B', 6):
        case GPIO('B', 8):
        case GPIO('B', 10):
        return 1;
    }
    return 0;
}

void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin)
{
    uint32_t afio = 1;
    if (periph == USART3_BASE)
        afio = 4;
    else if ((rx_pin == GPIO('B', 7)) || (tx_pin == GPIO('B', 6)))
        // USART1 has alternative 0 config when pins are PB6 and PB7
        afio = 0;
    if (rx_pin != tx_pin && rx_pin != (uint32_t)-1)
        gpio_peripheral(rx_pin, GPIO_FUNCTION(afio), 1);
    gpio_peripheral(tx_pin, GPIO_FUNCTION(afio), 0);
}

// Enable a peripheral clock
void enable_pclock(uint32_t const periph_base)
{
    if (is_enabled_pclock(periph_base))
        return;

    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        RCC->APBENR1 |= (1 << pos);
        RCC->APBENR1;
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        RCC->APBENR2 |= (1 << pos);
        RCC->APBENR2;
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        RCC->AHBENR |= (1 << pos);
        RCC->AHBENR;
    }
}

// Check if a peripheral clock has been enabled
uint32_t is_enabled_pclock(uint32_t const periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        return RCC->APBENR1 & (1 << pos);
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        return RCC->APBENR2 & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        return RCC->AHBENR & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    //(void)periph_base;
    //return CONFIG_CLOCK_FREQ / 2;
    return (uint32_t)(__LL_RCC_CALC_PCLK1_FREQ(HAL_RCC_GetHCLKFreq(), LL_RCC_GetAPB1Prescaler()));
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - IOPORT_BASE) / 0x400;
    RCC->IOPENR |= 1 << rcc_pos;
    (void)RCC->IOPENR;
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
        // push-pull, 0b00 | max speed 2 MHz, 0b01
        init.Mode = GPIO_MODE_OUTPUT_PP;
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        // Open-drain, 0b01 | max speed 2 MHz, 0b01
        init.Mode = GPIO_MODE_OUTPUT_OD;
    } else if (mode == GPIO_ANALOG) {
        init.Mode = GPIO_MODE_ANALOG;
    } else {
        init.Alternate = (mode >> 4);
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        // Alternate config
        if (mode & GPIO_OPEN_DRAIN) {
            // output open-drain mode, 10MHz
            init.Mode = GPIO_MODE_AF_OD;
        } else {
            // output push-pull mode, 10MHz
            init.Mode = GPIO_MODE_AF_PP;
        }
    }
    HAL_GPIO_Init(regs, &init);
}

#define TIMC_1 TIM15
#define TIMC_2 TIM16 // TIMC_1 UG triggers TIMC_2

// Return the current time (in absolute clock ticks).
uint32_t FAST_CODE_1 timer_read_time(void)
{
    uint32_t us = TIMC_2->CNT;
    us <<= 16;
    us += TIMC_1->CNT;
    return us;
}

uint32_t FAST_CODE_1 micros(void)
{
    return timer_read_time();
}

void FAST_CODE_1 delayMicroseconds(uint32_t const usecs)
{
    uint32_t const start = micros();
    while ((uint32_t)(micros() - start) < usecs);
}

void ms_timer_init(void)
{
    /* STM32L0x1 does not have DWT so use timer for us timer */
    enable_pclock((uint32_t)TIMC_1);
    enable_pclock((uint32_t)TIMC_2);
    TIMC_1->CR1 = 0; // Disable
    TIMC_2->CR1 = 0; // Disable

    /* Configure slave timer (upper 16bits) */
    TIMC_2->PSC = 0;
    TIMC_2->ARR = (1 << 16) - 1;
    TIMC_2->CNT = 0;
    TIMC_2->EGR = TIM_EGR_UG;
    /* TS  > 0 = ITR0
     * SMS > '111' = External clock mode 1 */
    TIMC_2->SMCR = TIM_SMCR_SMS_Msk;
    TIMC_2->CR1 = TIM_CR1_CEN;

    /* Configure master timer (lower 16bits) for 1us */
    TIMC_1->PSC = (CONFIG_CLOCK_FREQ / 1000000) - 1;
    TIMC_1->ARR = (1 << 16) - 1;
    TIMC_1->CNT = 0;
    TIMC_1->EGR = TIM_EGR_UG;
    TIMC_1->CR2 |= TIM_CR2_MMS_1; // Update trigger TRGO
    TIMC_1->CR1 = TIM_CR1_CEN | TIM_CR1_URS;

    // Enable SysTick
    NVIC_SetPriority(SysTick_IRQn, ISR_PRIO_TICKS);
    SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000UL) - 1;
    SysTick->VAL = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    return HAL_OK;
}

void system_clock_config(void)
{
    /* PLL f_out = ((f_in) * (PLLN / PLLM)) / PLLP
     * where:
     *  f_in = XO freq
     *  f_out = 64MHz
     *  PLLN = variable
     *  PLLM = 1
     *  PLLP = 2
    */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* Enable Powers */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
     */
    HAL_SYSCFG_StrobeDBattpinsConfig(SYSCFG_CFGR1_UCPD1_STROBE | SYSCFG_CFGR1_UCPD2_STROBE);

    /* Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the CPU, AHB and APB busses clocks to be 64MHz */
#if !USE_INTERNAL_XO && defined(HSE_VALUE)
#if HSE_VALUE == 16000000U
#define PLL_N   8  // 16MHz
#elif HSE_VALUE == 8000000U
#define PLL_N  16  // 8MHz
#else
#error "HSE_VALUE is not supported at the moment!"
#endif

    /** Enables the Clock Security System
     */
    HAL_RCC_EnableCSS();

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#else // USE_INTERNAL_XO
#if HSI_VALUE != 16000000U
#error "Wrong config! HSI VALUE is 16MHz!"
#endif
#define PLL_N  8

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
#endif

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = PLL_N;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    /* Flash latency = (CpuClock / 16) - 1  = 2WS */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM15 |
        RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 |
        RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
    PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLKSOURCE_PCLK1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();
}

void hw_init(void)
{
    /* Configure Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    /* Enable code cache */
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
}

extern "C" {
void NMI_Handler(void) {}
void HardFault_Handler(void) {Error_Handler();}
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
// void SysTick_Handler(void) {Error_Handler();}
void WWDG_IRQHandler(void) {Error_Handler();}
void PVD_IRQHandler(void) {Error_Handler();}
void RTC_TAMP_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void RCC_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 EXTI0_1_IRQHandler(void) {
    GPIO_EXTI_IRQHandler(0);
    GPIO_EXTI_IRQHandler(1);
}
void FAST_CODE_1 EXTI2_3_IRQHandler(void) {
    GPIO_EXTI_IRQHandler(2);
    GPIO_EXTI_IRQHandler(3);
}
void FAST_CODE_1 EXTI4_15_IRQHandler(void) {
    uint8_t pin;
    for (pin = 4; pin <= 15; pin++)
        GPIO_EXTI_IRQHandler(pin);
}

void UCPD1_2_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 DMA1_Channel1_IRQHandler(void) {
    uint8_t uart;
    for (uart = 0; uart < 3; uart++)
        USARTx_DMA_handler(uart); // USART TX/RX
}
void DMA1_Channel2_3_IRQHandler(void) {Error_Handler();}
void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void) {Error_Handler();}

void ADC1_COMP_IRQHandler(void) {Error_Handler();}
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {Error_Handler();}
void TIM1_CC_IRQHandler(void) {Error_Handler();}
//void TIM2_IRQHandler(void) {Error_Handler();} // hwtimer
void TIM3_IRQHandler(void) {Error_Handler();}
void TIM6_DAC_LPTIM1_IRQHandler(void) {Error_Handler();}
void TIM7_LPTIM2_IRQHandler(void) {Error_Handler();}
void TIM14_IRQHandler(void) {Error_Handler();}
void TIM15_IRQHandler(void) {Error_Handler();}
void TIM16_IRQHandler(void) {Error_Handler();}
void TIM17_IRQHandler(void) {Error_Handler();}
void I2C1_IRQHandler(void) {Error_Handler();}
void I2C2_IRQHandler(void) {Error_Handler();}
void SPI1_IRQHandler(void) {Error_Handler();}
void SPI2_IRQHandler(void) {Error_Handler();}

void FAST_CODE_1 USART1_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(0);
}
void FAST_CODE_1 USART2_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(1);
}
void FAST_CODE_1 USART3_4_LPUART1_IRQHandler(void) {
    USART_IDLE_IRQ_handler(2);
    //USART_IDLE_IRQ_handler(3);
}

} // extern

#endif /* STM32G0xx */
