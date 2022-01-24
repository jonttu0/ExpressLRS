#include "gpio.h"
#include <Arduino.h>

struct gpio_out IRAM_ATTR gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (160 <= pin)
        return {.pin = GPIO_PIN_IVALID};
    struct gpio_out g = {.pin = pin};
    gpio_out_reset(g, val);
    return g;
}

void IRAM_ATTR gpio_out_reset(struct gpio_out g, uint32_t val)
{
    pinMode(g.pin, OUTPUT);
    digitalWrite(g.pin, val);
}

void IRAM_ATTR gpio_out_toggle_noirq(struct gpio_out g)
{
    digitalWrite(g.pin, !digitalRead(g.pin));
}

void IRAM_ATTR gpio_out_toggle(struct gpio_out g)
{
    digitalWrite(g.pin, !digitalRead(g.pin));
}

void IRAM_ATTR gpio_out_write(struct gpio_out g, uint32_t val)
{
    digitalWrite(g.pin, val);
}


struct gpio_in IRAM_ATTR gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (160 <= pin)
        return {.pin = GPIO_PIN_IVALID};
    struct gpio_in g = {.pin = pin};
    gpio_in_reset(g, pull_up);
    return g;
}

void IRAM_ATTR gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    pinMode(g.pin, (pull_up) ? INPUT_PULLUP : INPUT);
}

uint8_t IRAM_ATTR gpio_in_read(struct gpio_in g)
{
    return digitalRead(g.pin);
}

void IRAM_ATTR gpio_in_isr(struct gpio_in g, isr_cb_t func, uint8_t type)
{
    attachInterrupt(digitalPinToInterrupt(g.pin), func, type);
}

void IRAM_ATTR gpio_in_isr_remove(struct gpio_in g)
{
    detachInterrupt(digitalPinToInterrupt(g.pin));
}

void IRAM_ATTR gpio_in_isr_clear_pending(struct gpio_in g)
{
    if (gpio_in_valid(g)) {
        uint32_t pin = g.pin;
        if (pin < 32)
            //Clear intr for gpio0 - gpio31
            GPIO.status_w1tc = (0x1 << pin);
        else
            //Clear intr for gpio32 - gpio39
            GPIO.status1_w1tc.val = (0x1 << (pin - 32));
    }
}

struct gpio_adc gpio_adc_setup(uint32_t pin)
{
    analogSetPinAttenuation(pin, ADC_0db);
    struct gpio_adc g = {.pin = pin, .att = ADC_0db};
    return g;
}

uint32_t gpio_adc_read(struct gpio_adc g)
{
    //analogSetPinAttenuation(g.pin, g.att);
    //return analogReadMilliVolts(g.pin);
    return analogRead(g.pin);
}
