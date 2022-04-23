#include "HwTimer.h"
#include "debug_elrs.h"
#include "platform.h"

/* HW specific code can be found from <mcu type>/ folder */

static void FAST_CODE_1 nullCallback(uint32_t) {};

HwTimer::HwTimer()
{
    callbackTickPre = nullCallback;
    callbackTick = nullCallback;
#if TIMER_TOCK_EN
    callbackTock = nullCallback;
#endif

    HWtimerInterval = TimerIntervalUSDefault;
    running = false;
#if TIMER_TOCK_EN
    tock = 1; // start with tick (= main)
#endif
}

void FAST_CODE_1 HwTimer::callback()
{
    uint32_t const us = micros();
#if TIMER_TOCK_EN
    tock ^= 1;
    if (tock) {
        callbackTock(us);
        return;
    }
#endif
    callbackTickPre(us);
    callbackTick(us);
}
