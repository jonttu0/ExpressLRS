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
    void updateInterval(uint32_t newTimerInterval);
    inline bool FAST_CODE_1 isRunning(void)
    {
        return running;
    }

    void callback();

    void (*callbackTockPre)(uint32_t us);
    void (*callbackTock)(uint32_t us);

    void setTime(uint32_t time = 0);

    void triggerSoon(void);

private:
    uint32_t HWtimerInterval;
    bool running = false;
};

extern HwTimer TxTimer;
