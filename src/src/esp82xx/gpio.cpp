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

void gpio_in_isr_clear_pending(struct gpio_in g)
{
    if (gpio_in_valid(g)) {
        int pin = digitalPinToInterrupt(g.pin);
        if (pin < 0)
            return;
        GPIEC = (1 << pin); //Clear Interrupt
    }
}
