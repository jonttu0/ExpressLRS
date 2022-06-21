#pragma once

#ifdef BUZZER_PIN
void buzzer_init(void);
void buzzer_beep(int note, int duration, int wait = 1);
#else
#define buzzer_init()
#define buzzer_beep(...)
#endif
