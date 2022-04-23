#pragma once

#include "platform.h"
#include <stdint.h>

#define TimerIntervalUSDefault 20000

#define TIMER_SOON         40 // 40us
#define TIMER_OFFSET       300 //250

#define TIMER_TOCK_EN      0


class HwTimer
{
public:
    HwTimer();
    void init();
    void start();
    void reset(int32_t offset = 0);
    void pause();
    void stop();
    inline void FAST_CODE_1 updateInterval(uint32_t const newTimerInterval)
    {
#if TIMER_TOCK_EN
        HWtimerInterval = newTimerInterval / 2;
#else
        HWtimerInterval = newTimerInterval;
#endif
    }
    inline bool FAST_CODE_1 isRunning(void) const
    {
        return running;
    }

    void callback();

    void (*callbackTickPre)(uint32_t us);
    void (*callbackTick)(uint32_t us);
#if TIMER_TOCK_EN
    void (*callbackTock)(uint32_t us);
#endif

    void setTime(uint32_t time = 0);

    void triggerSoon(void);

private:
    uint32_t HWtimerInterval;
    bool running;
#if TIMER_TOCK_EN
    bool tock;
#endif
};

extern HwTimer TxTimer;
