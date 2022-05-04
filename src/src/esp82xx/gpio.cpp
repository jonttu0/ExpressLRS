#include "gpio.h"
#include <Arduino.h>
#include <user_interface.h>

static inline uint8_t IRAM_ATTR _gpio_read(uint8_t const pin)
{
    if (pin < 16) {
        return !!(GPI & (1 << pin));
    }
    return GP16I & 0x1;
}


struct gpio_out IRAM_ATTR gpio_out_setup(uint32_t const pin, uint32_t const val)
{
    if (16 < pin)
        return {.pin = GPIO_PIN_IVALID};

    struct gpio_out g = {.pin = pin};
    gpio_out_reset(g, val);
    return g;
}

void IRAM_ATTR gpio_out_reset(struct gpio_out const g, uint32_t const val)
{
    pinMode(g.pin, OUTPUT);
    gpio_out_write(g, val);
}

void IRAM_ATTR gpio_out_toggle_noirq(struct gpio_out const g)
{
    gpio_out_write(g, !_gpio_read(g.pin));
}

void IRAM_ATTR gpio_out_toggle(struct gpio_out const g)
{
    gpio_out_toggle_noirq(g);
}

void IRAM_ATTR gpio_out_write(struct gpio_out const g, uint32_t const val)
{
    if (g.pin < 16) {
        if (val) GPOS = (1 << g.pin);
        else     GPOC = (1 << g.pin);
    } else {
        if (val) GP16O |= 1;
        else     GP16O &= ~1;
    }
}


struct gpio_in IRAM_ATTR gpio_in_setup(uint32_t const pin, int32_t const pull_up)
{
    if (16 < pin)
        return {.pin = GPIO_PIN_IVALID};
    struct gpio_in g = {.pin = pin};
    gpio_in_reset(g, pull_up);
    return g;
}

void IRAM_ATTR gpio_in_reset(struct gpio_in const g, int32_t const pull_up)
{
    pinMode(g.pin, (pull_up) ? INPUT_PULLUP : INPUT);
}

uint8_t IRAM_ATTR gpio_in_read(struct gpio_in const g)
{
    return _gpio_read(g.pin);
}

void IRAM_ATTR gpio_in_isr(struct gpio_in const g, isr_cb_t func, uint8_t const type)
{
    attachInterrupt(digitalPinToInterrupt(g.pin), func, type);
}

void IRAM_ATTR gpio_in_isr_remove(struct gpio_in const g)
{
    detachInterrupt(digitalPinToInterrupt(g.pin));
}

void IRAM_ATTR gpio_in_isr_clear_pending(struct gpio_in const g)
{
    if (gpio_in_valid(g)) {
        int const pin = digitalPinToInterrupt(g.pin);
        if (0 <= pin)
            GPIEC = (1 << pin); //Clear Interrupt
    }
}

struct gpio_adc gpio_adc_setup(uint32_t const pin)
{
    if (pin == 17 || pin == 0)
        return {.pin = pin};
    return {.pin = GPIO_PIN_IVALID};
}

uint32_t gpio_adc_read(struct gpio_adc const g)
{
    if (g.pin != GPIO_PIN_IVALID)
        return system_adc_read();
    return 0;
}
