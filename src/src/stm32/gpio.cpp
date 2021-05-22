#include "gpio.h"
#include "platform_internal.h"
#include <Arduino.h>

#if defined(STM32L4xx)
#define PR PR1
#endif

struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (160 <= pin)
        return {.pin = GPIO_PIN_IVALID};
    struct gpio_out g = {.pin = digitalPinToPinName(pin)};
    gpio_out_reset(g, val);
    return g;
}

void gpio_out_reset(struct gpio_out g, uint32_t val)
{
    pinMode(pinNametoDigitalPin(g.pin), OUTPUT);
    digitalWriteFast(g.pin, val);
}

void gpio_out_toggle_noirq(struct gpio_out g)
{
    digitalWriteFast(g.pin, !!!digitalReadFast(g.pin));
}

void gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t irq = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(irq);
}

void gpio_out_write(struct gpio_out g, uint32_t val)
{
    digitalWriteFast(g.pin, val);
}


struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (160 <= pin)
        return {.pin = GPIO_PIN_IVALID};
    struct gpio_in g = {.pin = digitalPinToPinName(pin)};
    gpio_in_reset(g, pull_up);
    return g;
}

void gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    pinMode(pinNametoDigitalPin(g.pin),
            (pull_up > 0) ? INPUT_PULLUP :
                ((pull_up < 0) ? INPUT_PULLDOWN : INPUT));
}

uint8_t gpio_in_read(struct gpio_in g)
{
    return digitalReadFast(g.pin);
}

void gpio_in_isr(struct gpio_in g, isr_cb_t func, uint8_t type)
{
    uint32_t pin = digitalPinToInterrupt(pinNametoDigitalPin(g.pin));
    attachInterrupt(pin, (callback_function_t)func, type);
}

void gpio_in_isr_remove(struct gpio_in g)
{
    detachInterrupt(digitalPinToInterrupt(pinNametoDigitalPin(g.pin)));
}

void gpio_in_isr_clear_pending(struct gpio_in g)
{
    // Clear pending IRQ
    if (gpio_in_valid(g)) {
        EXTI->PR = STM_GPIO_PIN(g.pin);
    }
}
