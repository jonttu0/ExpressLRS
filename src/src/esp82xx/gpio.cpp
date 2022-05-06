#include "gpio.h"
#include "platform.h"
#include <Arduino.h>
#include <user_interface.h>
#include <interrupts.h>

#define USE_ARDUINO_ISR 0

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

#if !USE_ARDUINO_ISR

#define CLZ(x) __builtin_clz(x)
#define CTZ(x) (31 - CLZ((x) & -(x)))

typedef struct {
    uint8_t mode;
    isr_cb_t fn;
} interrupt_handler_t;

static interrupt_handler_t DRAM_ATTR exti_handlers[16];
static uint32_t DRAM_ATTR interrupt_mask;

static void set_interrupt_handlers(uint8_t const pin, isr_cb_t const userFunc, uint8_t const mode)
{
    interrupt_handler_t * const handler = &exti_handlers[pin];
    handler->mode = mode;
    handler->fn = userFunc;
}

static void IRAM_ATTR _exti_handler(void *arg, void *frame)
{
    (void) arg;
    (void) frame;
    uint32_t status = GPIE;
    GPIEC = status;             // Clear interrupts
    uint32_t const levels = GPI;
    status &= interrupt_mask;    // Mask enabled EXTIs
    if (status == 0)
        return;
    ETS_GPIO_INTR_DISABLE();
#if 0
    int iter = 0;
    while (status) {
        while (!(status & (1 << iter)))
            iter++;
        status &= ~(1 << iter);
        interrupt_handler_t const * const handler = &exti_handlers[iter];
        if (handler->fn &&
            (handler->mode == CHANGE || (handler->mode & 1) == !!(levels & (1 << iter)))) {
            // to make ISR compatible to Arduino AVR model where interrupts are disabled
            // we disable them before we call the client ISR
            esp8266::InterruptLock irqLock; // stop other interrupts
            handler->fn();
        }
    }
#else
    while (status) {
        uint32_t lsb = CTZ(status);
        interrupt_handler_t const * const handler = &exti_handlers[lsb];
        lsb = 0x1 << lsb;
        status &= ~lsb;
        if (handler->fn &&
            (handler->mode == CHANGE || (handler->mode & RISING) == !!(levels & lsb))) {
            // to make ISR compatible to Arduino AVR model where interrupts are disabled
            // we disable them before we call the client ISR
            esp8266::InterruptLock irqLock; // stop other interrupts
            handler->fn();
        }
    }
#endif
    ETS_GPIO_INTR_ENABLE();
}

void attachInterrupt(uint8_t const pin, isr_cb_t const func, int const mode)
{
    // https://github.com/esp8266/esp8266-wiki/wiki/Memory-Map
    if ((uint32_t)func >= 0x40200000) {
        // ISR not in IRAM
        ::printf((PGM_P)F("ISR not in IRAM!\r\n"));
        abort();
    }
    if (pin < 16) {
        ETS_GPIO_INTR_DISABLE();
        set_interrupt_handlers(pin, func, mode);
        interrupt_mask |= (1 << pin);
        GPC(pin) &= ~(0xF << GPCI);         // INT mode disabled
        GPIEC = (1 << pin);                 // Clear Interrupt for this pin
        GPC(pin) |= ((mode & 0xF) << GPCI); // INT mode "mode"
        ETS_GPIO_INTR_ATTACH(_exti_handler, &interrupt_mask);
        ETS_GPIO_INTR_ENABLE();
    }
}

void detachInterrupt(uint8_t pin)
{
    if (pin < 16) {
        ETS_GPIO_INTR_DISABLE();
        GPC(pin) &= ~(0xF << GPCI);//INT mode disabled
        GPIEC = (1 << pin); //Clear Interrupt for this pin
        interrupt_mask &= ~(1 << pin);
		set_interrupt_handlers(pin, NULL, 0);
        if (interrupt_mask) {
            ETS_GPIO_INTR_ENABLE();
        }
    }
}
#endif

void IRAM_ATTR gpio_in_isr(struct gpio_in const g, isr_cb_t func, uint8_t const mode)
{
    attachInterrupt(digitalPinToInterrupt(g.pin), func, mode);
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
