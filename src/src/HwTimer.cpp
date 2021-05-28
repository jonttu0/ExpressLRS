#include "HwTimer.h"
#include "debug_elrs.h"
#include "platform.h"

/* HW specific code can be found from <mcu type>/ folder */

static void FAST_CODE_1 nullCallback(uint32_t) {};

HwTimer::HwTimer()
{
    callbackTickPre = nullCallback;
    callbackTick = nullCallback;
    //callbackTock = nullCallback;

    HWtimerInterval = TimerIntervalUSDefault;
    running = false;
}

void FAST_CODE_1 HwTimer::callback()
{
    uint32_t us = micros();
    callbackTickPre(us);
    callbackTick(us);
}
