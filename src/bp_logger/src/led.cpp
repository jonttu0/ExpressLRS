#include "led.h"

#ifdef WS2812_PIN
#include <Adafruit_NeoPixel.h>
#define PIXEL_FORMAT    (NEO_GRB + NEO_KHZ800)

static Adafruit_NeoPixel led_rgb;
static uint32_t led_rgb_state;
#endif

void led_init(void)
{
#ifdef WS2812_PIN
  led_rgb.setPin(WS2812_PIN);
  led_rgb.updateType(PIXEL_FORMAT);
  led_rgb.begin();
  led_rgb.updateLength(1);
  led_rgb.setBrightness(255);
  led_rgb.fill();
  led_rgb.show();
#endif
}

void led_set(uint32_t state)
{
#ifdef WS2812_PIN
  if (state == LED_UNKNOWN)
      state = led_rgb_state;
  uint32_t led_rgb_color = (state == LED_OFF) ? 0 : led_rgb.Color(
    (uint8_t)(state >> 16), (uint8_t)(state >> 8), (uint8_t)state);
  led_rgb.fill(led_rgb_color);
  led_rgb.show();
  led_rgb_state = state;
#endif
}

void led_brightness_set(uint8_t brightness, uint8_t show)
{
#ifdef WS2812_PIN
    led_rgb.setBrightness(brightness);
    if (show)
        led_rgb.show();
#endif
}

uint8_t led_brightness_get(void)
{
#ifdef WS2812_PIN
    return led_rgb.getBrightness();
#else
    return 0;
#endif
}
