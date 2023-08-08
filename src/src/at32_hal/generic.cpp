#include "Arduino.h"
#include "at32_def.h"
#include "debug_elrs.h"

// -------------------------------------------
// Generic time methods

volatile uint32_t _ms_cntr;

extern "C" void  FAST_CODE_1 SysTick_Handler(void)
{
    ++_ms_cntr;
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
uint8_t timer_is_before(uint16_t time1, uint16_t time2)
{
    return (int16_t)(time1 - time2) < 0;
}
uint8_t timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}

uint32_t FAST_CODE_1 millis(void)
{
    //uint32_t ms = read_u32(&_ms_cntr);
    //return ms;
    return _ms_cntr;
}

void FAST_CODE_1 delay(uint32_t const ms)
{
    uint32_t const start = millis();
    //ms += millis();
    //while (timer_is_before(millis(), ms))
    //    ;
    while((millis() - start) < ms);
}

// -------------------------------------------
// Error handlers

void shutdown(const char * reason)
{
    DEBUG_PRINTF("F**k! %s\n", reason);
    uint8_t state = 0;
    while(1) {
        delay(50);
        platform_set_led(state);
        state ^= 1;
    }
}

void _Error_Handler(const char * error, int line)
{
    DEBUG_PRINTF("%s : %d\n", error, line);
    shutdown("_Error_Handler");
    (void)line;
}

// -------------------------------------------
