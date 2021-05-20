#pragma once

#include "platform.h"
#include <stdint.h>

#define TimerIntervalUSDefault 20000

#define TIMER_SOON 40 // 40us

#if (PLATFORM_STM32 && !defined(ARDUINO)) || PLATFORM_ESP8266
#define USE_TIMER_KICK  1
#endif

#define TIMER_OFFSET_KICK  100
#define TIMER_OFFSET       300 //250
#define TIMER_OFFSET_LIMIT 50

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
