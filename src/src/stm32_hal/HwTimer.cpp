#include "HwTimer.h"
#include "internal.h"
#include "irq.h"
#include "priorities.h"
#include <Arduino.h>

#if defined(STM32L4xx)
#define SWIER SWIER1
#endif

#define TIMER_IS_2US 1

HwTimer TxTimer;

/****************************************************************
 * Low level timer code
 ****************************************************************/

#define TIMx TIM2
#define TIMx_IRQn TIM2_IRQn
#define TIMx_IRQx_FUNC TIM2_IRQHandler

static FORCED_INLINE uint32_t timer_counter_get(void)
{
    return TIMx->CNT;
}

static FORCED_INLINE void timer_counter_set(uint32_t cnt)
{
    TIMx->CNT = cnt >> TIMER_IS_2US;
}

static FORCED_INLINE void timer_set(uint32_t next)
{
    TIMx->ARR = next >> TIMER_IS_2US;
}

/****************************************************************
 * HW Timer setup and irqs
 ****************************************************************/

extern "C"
{
    // Hardware timer IRQ handler - dispatch software timers
    void FAST_CODE_1 TIMx_IRQx_FUNC(void)
    {
        uint16_t SR = TIMx->SR;
        if (SR & TIM_SR_UIF) {
            TIMx->SR = SR & ~(TIM_SR_UIF);
            TxTimer.callback();
        }
    }
}

void timer_enable(void)
{
    TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_DIR;
    TIMx->DIER = TIM_DIER_UIE;
    TIMx->SR &= ~(TIM_SR_UIF);
}

void timer_disable(void)
{
    TIMx->CR1 = 0;
    TIMx->DIER = 0;
    TIMx->SR &= ~(TIM_SR_UIF);
}

static void timer_init(void)
{
    enable_pclock((uint32_t)TIMx);
    timer_disable();
    // Set clock prescaler to 1us or 2us
    // Note: PSC == 1 clock is APB1 x1 (36MHz) else x2 (72MHz)
    TIMx->PSC = (2 * get_pclock_frequency((uint32_t)TIMx) / (1000000 >> TIMER_IS_2US)) - 1;
    TIMx->ARR = (1U << 16) - 1; // Init to max
    TIMx->CNT = 0;
    //TIMx->RCR = 0;
    TIMx->EGR = TIM_EGR_UG;
    //TIMx->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIMx_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_TIM, 0));
    NVIC_EnableIRQ(TIMx_IRQn);
}

/****************************************************************
 * Public
 ****************************************************************/
void HwTimer::init()
{
    timer_init();
}

void HwTimer::start()
{
    running = 1;
    reset(0);
    timer_enable();
}

void HwTimer::stop()
{
    running = 0;
    timer_disable();
}

void HwTimer::pause()
{
    stop();
}

void FAST_CODE_1 HwTimer::reset(int32_t offset)
{
    if (running) {
        /* Reset counter and set next alarm time */
        timer_counter_set(HWtimerInterval - offset);
        timer_set(HWtimerInterval - offset);
    }
}

void FAST_CODE_1 HwTimer::setTime(uint32_t time)
{
    if (!time)
        time = HWtimerInterval;
    timer_set(time);
}

void FAST_CODE_1 HwTimer::triggerSoon(void)
{
#if 1
    TIMx->SR  &= ~(TIM_SR_UIF); // Clear pending ISR
    timer_counter_set(TIMER_SOON);
#else
    /* Generate soft trigger to run ISR asap */
    EXTI->SWIER |= (0x1 << 3);
#endif
}
