// Code to setup clocks and gpio on stm32f1
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// https://github.com/KevinOConnor/klipper
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "gpio.h" // gpio_out_setup
#include "internal.h"
#include "helpers.h"
#include "irq.h"
#include "at32_def.h"
#include "priorities.h"
#include "platform.h"
#include <string.h> // ffs

gpio_type * __section(".data") digital_regs[] = {
    ['A' - 'A'] = GPIOA,
    GPIOB,
    GPIOC,
#ifdef GPIOD
    ['D' - 'A'] = GPIOD,
#else
    NULL,
#endif
#ifdef GPIOE
    ['E' - 'A'] = GPIOE,
#else
    NULL,
#endif
#ifdef GPIOF
    ['F' - 'A'] = GPIOF,
#else
    NULL,
#endif
#ifdef GPIOG
    ['G' - 'A'] = GPIOG,
#else
    NULL,
#endif
#ifdef GPIOH
    ['H' - 'A'] = GPIOH,
#else
    NULL,
#endif
#ifdef GPIOI
    ['I' - 'A'] = GPIOI,
#else
    NULL,
#endif
};

// Convert a register and bit location back to an integer pin identifier
static int
regs_to_pin(gpio_type *regs, uint32_t bit)
{
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return GPIO('A' + i, ffs(bit) - 1);
    return 0;
}

struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (GPIO('J', 0) <= pin)
        return {.regs = NULL, .bit = 0};
    gpio_type *regs = digital_regs[GPIO2PORT(pin)];
    if (!regs)
        Error_Handler();
    struct gpio_out g = {.regs = regs, .bit = GPIO2BIT(pin)};
    gpio_out_reset(g, val);
    return g;
}

void gpio_out_reset(struct gpio_out g, uint32_t val)
{
    gpio_type *regs = (gpio_type *)g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_OUTPUT, 0);
    gpio_out_write(g, val);
    irq_restore(flag);
}

void FAST_CODE_1
gpio_out_toggle_noirq(struct gpio_out g)
{
    gpio_type *regs = (gpio_type *)g.regs;
    if (regs->odt & g.bit)
        regs->clr = g.bit;
    else
        regs->scr = g.bit;
}

void FAST_CODE_1
gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

void FAST_CODE_1
gpio_out_write(struct gpio_out g, uint32_t val)
{
    gpio_type *regs = (gpio_type *)g.regs;
    if (val)
        regs->scr = g.bit;
    else
        regs->clr = g.bit;
}

uint8_t FAST_CODE_1
gpio_out_read(struct gpio_out g)
{
    gpio_type *regs = (gpio_type *)g.regs;
    return !!(regs->odt & g.bit);
}


struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (GPIO('J', 0) <= pin)
        return {.regs = NULL, .bit = 0};

    gpio_type *regs = digital_regs[GPIO2PORT(pin)];
    if (!regs)
        Error_Handler();
    struct gpio_in g = {.regs = regs, .bit = GPIO2BIT(pin)};
    gpio_in_reset(g, pull_up);
    return g;
}

void gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    gpio_type *regs = (gpio_type *)g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint8_t FAST_CODE_1
gpio_in_read(struct gpio_in g)
{
    gpio_type *regs = (gpio_type *)g.regs;
    return !!(regs->idt & g.bit);
}

typedef struct
{
    IRQn_Type irqnb;
    isr_cb_t callback;
} gpio_irq_conf_str;

/* Private Variables */
static gpio_irq_conf_str DRAM_FORCE_ATTR gpio_irq_conf[GPIO_NUM_PINS] = {
#if defined(AT32F415)
    {.irqnb = EXINT0_IRQn, .callback = NULL},     //GPIO_PIN_0
    {.irqnb = EXINT1_IRQn, .callback = NULL},     //GPIO_PIN_1
    {.irqnb = EXINT2_IRQn, .callback = NULL},     //GPIO_PIN_2
    {.irqnb = EXINT3_IRQn, .callback = NULL},     //GPIO_PIN_3
    {.irqnb = EXINT4_IRQn, .callback = NULL},     //GPIO_PIN_4
    {.irqnb = EXINT9_5_IRQn, .callback = NULL},   //GPIO_PIN_5
    {.irqnb = EXINT9_5_IRQn, .callback = NULL},   //GPIO_PIN_6
    {.irqnb = EXINT9_5_IRQn, .callback = NULL},   //GPIO_PIN_7
    {.irqnb = EXINT9_5_IRQn, .callback = NULL},   //GPIO_PIN_8
    {.irqnb = EXINT9_5_IRQn, .callback = NULL},   //GPIO_PIN_9
    {.irqnb = EXINT15_10_IRQn, .callback = NULL}, //GPIO_PIN_10
    {.irqnb = EXINT15_10_IRQn, .callback = NULL}, //GPIO_PIN_11
    {.irqnb = EXINT15_10_IRQn, .callback = NULL}, //GPIO_PIN_12
    {.irqnb = EXINT15_10_IRQn, .callback = NULL}, //GPIO_PIN_13
    {.irqnb = EXINT15_10_IRQn, .callback = NULL}, //GPIO_PIN_14
    {.irqnb = EXINT15_10_IRQn, .callback = NULL}  //GPIO_PIN_15
#elif defined(AT32F403A_407)
// TODO!
#else
#error "GPIO IRQ config is missing!"
#endif
};

void gpio_in_isr(struct gpio_in g, isr_cb_t callback, uint8_t it_mode)
{
    gpio_type * regs = (gpio_type*)g.regs;
    uint32_t const pin = regs_to_pin(regs, g.bit);
    if (!regs)
        Error_Handler();

    uint32_t const port_idx = GPIO2PORT(pin);
    uint32_t const index = GPIO2IDX(pin);

    gpio_exint_line_config((gpio_port_source_type)port_idx, (gpio_pins_source_type)index);

    exint_init_type extintcfg;
    extintcfg.line_enable = TRUE;
    extintcfg.line_select = g.bit;
    if (it_mode == FALLING)
        extintcfg.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    else if (it_mode == RISING)
        extintcfg.line_polarity = EXINT_TRIGGER_RISING_EDGE;
    else
        extintcfg.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
    extintcfg.line_mode = EXINT_LINE_INTERRUPUT,
    exint_init(&extintcfg);

    write_u32(&gpio_irq_conf[index].callback, (uint32_t)callback);

    // Enable and set EXTI Interrupt
    NVIC_SetPriority(
        gpio_irq_conf[index].irqnb,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_EXTI, 0));
    NVIC_EnableIRQ(gpio_irq_conf[index].irqnb);
    exint_interrupt_enable(g.bit, TRUE);
    // Clear pending IRQ
    EXINT->intsts = g.bit;
}

void gpio_in_isr_remove(struct gpio_in const g)
{
    if (!gpio_in_valid(g))
        Error_Handler();
    uint32_t index = ffs(g.bit) - 1;
    irqstatus_t irq = irq_save();
    //NVIC_DisableIRQ(gpio_irq_conf[index].irqnb);
    //exint_interrupt_enable(g.bit, FALSE);
    write_u32(&gpio_irq_conf[index].callback, 0);
    irq_restore(irq);
}

void FAST_CODE_1
gpio_in_isr_clear_pending(struct gpio_in const g)
{
    if (gpio_in_valid(g))
        // Clear pending IRQ
        EXINT->intsts = g.bit;
}

/*********************/

void FAST_CODE_1
GPIO_EXTI_IRQHandler(uint16_t const pin)
{
    /* EXTI line interrupt detected */
    uint32_t const line = 0x1 << pin;
    if (EXINT->intsts & line) {
        if (gpio_irq_conf[pin].callback != NULL) {
            gpio_irq_conf[pin].callback();
        }
        EXINT->intsts = line;
    }
}

/*********************/

struct gpio_adc gpio_adc_setup(uint32_t pin)
{
    return {.adc = NULL, .chan = 0};
}

uint32_t gpio_adc_read(struct gpio_adc g)
{
    return 0;
}
