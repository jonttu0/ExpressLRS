#pragma once

#include <stdint.h>

enum led_state_e {
  LED_OFF       = 0x000000,
  LED_INIT      = 0xebd234,
  LED_WIFI_STA  = 0x00ff00,
  LED_WIFI_AP   = 0x0000ff,

  LED_FREQ_900  = 0xfc00b5,
  LED_FREQ_2400 = 0x1d00fc,

  LED_WARNING   = 0xffff00,
  LED_ERROR     = 0xff0000,

  LED_UNKNOWN   = 0xffffff,
};

void led_init(void);
void led_set(uint32_t state = LED_UNKNOWN);

void led_brightness_set(uint8_t brightness, uint8_t show = 0);
uint8_t led_brightness_get(void);
