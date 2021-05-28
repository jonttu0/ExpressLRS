#pragma once

#include "platform.h"
#include <stdint.h>

#define TimerIntervalUSDefault 20000

#define TIMER_SOON         40 // 40us
#define TIMER_OFFSET       300 //250


class HwTimer
{
public:
    HwTimer();
    void init();
    void start();
    void reset(int32_t offset = 0);
    void pause();
    void stop();
    inline void FAST_CODE_1 updateInterval(uint32_t newTimerInterval)
    {
        HWtimerInterval = newTimerInterval;
    }
    inline bool FAST_CODE_1 isRunning(void) const
    {
        return running;
    }

    void callback();

    void (*callbackTickPre)(uint32_t us);
    void (*callbackTick)(uint32_t us);
    //void (*callbackTock)(uint32_t us);

    void setTime(uint32_t time = 0);

    void triggerSoon(void);

private:
    uint32_t HWtimerInterval;
    bool running;
};

extern HwTimer TxTimer;
