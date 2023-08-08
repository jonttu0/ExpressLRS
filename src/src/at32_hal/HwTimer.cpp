#include "HwTimer.h"
#include "internal.h"
#include "irq.h"
#include "priorities.h"
#include <Arduino.h>

#define TIMER_IS_2US 1

HwTimer TxTimer;

/****************************************************************
 * Low level timer code
 ****************************************************************/

#define TIMx TMR2
#define TIMx_IRQn TMR2_GLOBAL_IRQn
#define TIMx_IRQx_FUNC TMR2_GLOBAL_IRQHandler

static FORCED_INLINE uint32_t timer_counter_get(void)
{
    // tmr_counter_value_get();
    return TIMx->cval;
}

static FORCED_INLINE void timer_counter_set(uint32_t const cnt)
{
    // tmr_counter_value_set();
    TIMx->cval = cnt >> TIMER_IS_2US;
}

static FORCED_INLINE void timer_set(uint32_t const next)
{
    //tmr_period_value_set();
    TIMx->pr = next >> TIMER_IS_2US;
}

/****************************************************************
 * HW Timer setup and irqs
 ****************************************************************/

extern "C"
{
    // Hardware timer IRQ handler - dispatch software timers
    void FAST_CODE_1 TIMx_IRQx_FUNC(void)
    {
        TxTimer.callback();
    }
}

void timer_enable(void)
{
    tmr_counter_enable(TIMx, TRUE);
}

void timer_disable(void)
{
    tmr_counter_enable(TIMx, FALSE);
    TIMx->cval = 0;
}

static void timer_init(void)
{
    enable_pclock((uint32_t)TIMx);
    tmr_reset(TIMx);
    timer_disable();
    tmr_32_bit_function_enable(TIMx, TRUE);
    // Set clock prescaler to 1us or 2us
    uint32_t const tmr_div = 2 * get_pclock_frequency((uint32_t)TIMx) / (1000000U >> TIMER_IS_2US) - 1;
    tmr_base_init(TIMx, 0xffffffff, tmr_div);
    tmr_clock_source_div_set(TIMx, TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(TIMx, TMR_COUNT_DOWN);
    tmr_period_buffer_enable(TIMx, TRUE);

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
#if TIMER_TOCK_EN
    tock = 1; // start with tick (= main)
#endif
    reset(0);
    timer_enable();
}

void HwTimer::stop()
{
    timer_disable();
}

void HwTimer::pause()
{
    stop();
}

void FAST_CODE_1 HwTimer::reset(int32_t const offset)
{
    /* Reset counter and set next alarm time */
    timer_set(HWtimerInterval - offset);
    timer_counter_set(HWtimerInterval - offset);
}

void FAST_CODE_1 HwTimer::setTime(uint32_t time)
{
    if (!time)
        time = HWtimerInterval;
    timer_set(time);
    timer_counter_set(time);
}

void FAST_CODE_1 HwTimer::triggerSoon(void)
{
#if 0
    timer_counter_set(TIMER_SOON);
#else
    /* Generate soft trigger to run ISR asap */
    TIMx->swevt_bit.ovfswtr |= 1;
#endif
}
