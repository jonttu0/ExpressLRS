#include "HwTimer.h"
#include "targets.h"
#include "debug_elrs.h"
#include <user_interface.h>
#include <Arduino.h>

HwTimer TxTimer;

void IRAM_ATTR MyTimCallback(void)
{
    TxTimer.callback();
}

void HwTimer::init()
{
    noInterrupts();
    timer1_attachInterrupt(MyTimCallback);
    running = false;
    interrupts();
}

void IRAM_ATTR HwTimer::start()
{
    if (running)
        return;
    running = true;
#if TIMER_TOCK_EN
    tock = 1; // start with tick (= main)
#endif
    reset(0);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); //5MHz ticks
}

void IRAM_ATTR HwTimer::stop()
{
    running = false;
    timer1_disable();
}

void IRAM_ATTR HwTimer::pause()
{
    stop();
}

void IRAM_ATTR HwTimer::reset(int32_t const offset)
{
    setTime(HWtimerInterval - offset);
}

void IRAM_ATTR HwTimer::setTime(uint32_t time)
{
    if (!time)
        time = HWtimerInterval;
    timer1_write(time * 5);
}

void IRAM_ATTR HwTimer::triggerSoon(void)
{
    timer1_write(TIMER_SOON * 5);
}
