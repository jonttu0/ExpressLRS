#include "gpio.h"
#include "platform.h"
#include "helpers.h"
#include <Arduino.h>
#include <user_interface.h>
#include <interrupts.h>


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

// ***** GPIO ISR handler *****
//  ignore Arduino framework

#define CLZ(x) __builtin_clz(x)
#define CTZ(x) (31 - CLZ((x) & -(x)))

typedef struct {
    uint8_t mode;
    isr_cb_t fn;
} exti_handler_t;

static exti_handler_t DRAM_ATTR exti_handlers[EXTERNAL_NUM_INTERRUPTS];
static uint32_t DRAM_ATTR interrupt_mask;

static void IRAM_ATTR set_interrupt_handlers(uint8_t const pin, isr_cb_t const userFunc, uint8_t const mode)
{
    exti_handler_t * const handler = &exti_handlers[pin];
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
    status &= interrupt_mask;   // Mask enabled EXTIs
    if (status == 0)
        return;
    ETS_GPIO_INTR_DISABLE();
    while (status) {
        uint32_t lsb = CTZ(status);
        exti_handler_t const * const handler = &exti_handlers[lsb];
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
    ETS_GPIO_INTR_ENABLE();
}

void IRAM_ATTR attachInterrupt(uint8_t const pin, isr_cb_t const func, int const mode)
{
    // ISR funcs must be in IRAM
    //  See https://github.com/esp8266/esp8266-wiki/wiki/Memory-Map

    if (pin < ARRAY_SIZE(exti_handlers) && (uint32_t)func < 0x40200000) {
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

void IRAM_ATTR detachInterrupt(uint8_t const pin)
{
    if (pin < ARRAY_SIZE(exti_handlers)) {
        ETS_GPIO_INTR_DISABLE();
        GPC(pin) &= ~(0xF << GPCI);     // INT mode disabled
        GPIEC = (1 << pin);             // Clear Interrupt for this pin
        interrupt_mask &= ~(1 << pin);
		set_interrupt_handlers(pin, NULL, 0);
        if (interrupt_mask) {
            ETS_GPIO_INTR_ENABLE();
        }
    }
}

void IRAM_ATTR gpio_in_isr(struct gpio_in const g, isr_cb_t func, uint8_t const mode)
{
    attachInterrupt(g.pin, func, mode);
}

void IRAM_ATTR gpio_in_isr_remove(struct gpio_in const g)
{
    detachInterrupt(g.pin);
}

void IRAM_ATTR gpio_in_isr_clear_pending(struct gpio_in const g)
{
    if (g.pin < ARRAY_SIZE(exti_handlers)) {
        GPIEC = (1 << g.pin); // Clear Interrupt
    }
}

struct gpio_adc gpio_adc_setup(uint32_t const pin)
{
    if (pin == 17 || pin == 0)
        return {.pin = pin};
    return {.pin = GPIO_PIN_IVALID};
}

uint32_t IRAM_ATTR gpio_adc_read(struct gpio_adc const g)
{
    if (g.pin != GPIO_PIN_IVALID)
        // NOTE! system_adc_read and system_adc_read_fast are both in FLASH :/
        return system_adc_read();
    return 0;
}
