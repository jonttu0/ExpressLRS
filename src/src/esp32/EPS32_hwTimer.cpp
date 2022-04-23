#include "HwTimer.h"
#include "targets.h"
#include <FreeRTOS.h>
#include <esp32-hal-timer.h>
#include "debug_elrs.h"

HwTimer TxTimer;

static hw_timer_t *timer = NULL;
static portMUX_TYPE isrMutex = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR TimerTask_ISRhandler(void)
{
    portENTER_CRITICAL(&isrMutex);
    TxTimer.callback();
    portEXIT_CRITICAL(&isrMutex);
}

void HwTimer::init()
{
    if (!timer)
    {
        timer = timerBegin(0, (APB_CLK_FREQ / 1000000), true); // us timer
        timerAttachInterrupt(timer, &TimerTask_ISRhandler, true);
    }
    stop();
    setTime(HWtimerInterval);
    timerAlarmEnable(timer);
}

void IRAM_ATTR HwTimer::start()
{
    if (running)
        return;
    if (!timer)
        init();
    running = true;
#if TIMER_TOCK_EN
    tock = 1; // start with tick (= main)
#endif
    reset(0);
    timerStart(timer);
}

void IRAM_ATTR HwTimer::stop()
{
    running = false;

    if (timer)
        timerStop(timer);
}

void IRAM_ATTR HwTimer::pause()
{
    running = false;
    if (timer)
        timerAlarmDisable(timer);
}

void IRAM_ATTR HwTimer::reset(int32_t offset)
{
    if (timer && running)
    {
        timerWrite(timer, 0);
        setTime(HWtimerInterval - offset);
    }
}

void IRAM_ATTR HwTimer::setTime(uint32_t time)
{
    if (timer)
    {
        if (!time)
            time = HWtimerInterval;
        timerAlarmWrite(timer, time, true);
    }
}

void IRAM_ATTR HwTimer::triggerSoon(void)
{
    uint32_t interval = timerAlarmRead(timer);
    interval -= TIMER_SOON;
    timerWrite(timer, interval);
}
