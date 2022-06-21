#include "buzzer.h"
#include <Arduino.h>

#ifdef BUZZER_PIN

void buzzer_init(void)
{
    pinMode(BUZZER_PIN, OUTPUT);
}


void buzzer_beep(int note, int duration, int wait)
{
#if BUZZER_PASSIVE
    tone(BUZZER_PIN, note, duration);
    if (wait)
        delay(duration);
#else // BUZZER_ACTIVE
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
#endif /* BUZZER_PASSIVE */
}

#endif /* BUZZER_PIN */
