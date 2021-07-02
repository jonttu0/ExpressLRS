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
#include "stm32_def.h"
#include "priorities.h"
#include "platform.h"
#include <string.h> // ffs

#if defined(STM32L4xx) || defined(STM32G0xx)
#define IMR IMR1
#define EMR EMR1
#define RTSR RTSR1
#define FTSR FTSR1
#if defined(STM32G0xx)
/* TODO: Should merge these 2 */
//#define PR FPR1
#define PR RPR1
#else
#define PR PR1
#endif
#endif

GPIO_TypeDef * __section(".data") digital_regs[] = {
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
regs_to_pin(GPIO_TypeDef *regs, uint32_t bit)
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
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    if (!regs)
        Error_Handler();
    struct gpio_out g = {.regs = regs, .bit = GPIO2BIT(pin)};
    gpio_out_reset(g, val);
    return g;
}

void gpio_out_reset(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_OUTPUT, 0);
    gpio_out_write(g, val);
    irq_restore(flag);
}

void FAST_CODE_1
gpio_out_toggle_noirq(struct gpio_out g)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)g.regs;
    regs->ODR ^= g.bit;
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
    GPIO_TypeDef *regs = (GPIO_TypeDef *)g.regs;
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << 16;
}

uint8_t FAST_CODE_1
gpio_out_read(struct gpio_out g)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)g.regs;
    return !!(regs->ODR & g.bit);
}


struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (GPIO('J', 0) <= pin)
        return {.regs = NULL, .bit = 0};

    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    if (!regs)
        Error_Handler();
    struct gpio_in g = {.regs = regs, .bit = GPIO2BIT(pin)};
    gpio_in_reset(g, pull_up);
    return g;
}

void gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint8_t FAST_CODE_1
gpio_in_read(struct gpio_in g)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)g.regs;
    return !!(regs->IDR & g.bit);
}

typedef struct
{
    IRQn_Type irqnb;
    isr_cb_t callback;
} gpio_irq_conf_str;

/* Private Variables */
static gpio_irq_conf_str DRAM_FORCE_ATTR gpio_irq_conf[GPIO_NUM_PINS] = {
#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
    {.irqnb = EXTI0_1_IRQn, .callback = NULL},  //GPIO_PIN_0
    {.irqnb = EXTI0_1_IRQn, .callback = NULL},  //GPIO_PIN_1
    {.irqnb = EXTI2_3_IRQn, .callback = NULL},  //GPIO_PIN_2
    {.irqnb = EXTI2_3_IRQn, .callback = NULL},  //GPIO_PIN_3
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_4
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_5
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_6
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_7
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_8
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_9
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_10
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_11
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_12
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_13
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}, //GPIO_PIN_14
    {.irqnb = EXTI4_15_IRQn, .callback = NULL}  //GPIO_PIN_15
#else
#if defined(STM32F3xx)
#define EXTI2_IRQn EXTI2_TSC_IRQn
#endif
    {.irqnb = EXTI0_IRQn, .callback = NULL},     //GPIO_PIN_0
    {.irqnb = EXTI1_IRQn, .callback = NULL},     //GPIO_PIN_1
    {.irqnb = EXTI2_IRQn, .callback = NULL},     //GPIO_PIN_2
    {.irqnb = EXTI3_IRQn, .callback = NULL},     //GPIO_PIN_3
    {.irqnb = EXTI4_IRQn, .callback = NULL},     //GPIO_PIN_4
    {.irqnb = EXTI9_5_IRQn, .callback = NULL},   //GPIO_PIN_5
    {.irqnb = EXTI9_5_IRQn, .callback = NULL},   //GPIO_PIN_6
    {.irqnb = EXTI9_5_IRQn, .callback = NULL},   //GPIO_PIN_7
    {.irqnb = EXTI9_5_IRQn, .callback = NULL},   //GPIO_PIN_8
    {.irqnb = EXTI9_5_IRQn, .callback = NULL},   //GPIO_PIN_9
    {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_10
    {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_11
    {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_12
    {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_13
    {.irqnb = EXTI15_10_IRQn, .callback = NULL}, //GPIO_PIN_14
    {.irqnb = EXTI15_10_IRQn, .callback = NULL}  //GPIO_PIN_15
#endif
};

void gpio_in_isr(struct gpio_in g, isr_cb_t callback, uint8_t it_mode)
{
    GPIO_TypeDef * regs = (GPIO_TypeDef*)g.regs;
    uint32_t const pin = regs_to_pin(regs, g.bit);
#if 0
    if (!regs)
        Error_Handler();
    uint32_t index = GPIO2IDX(pin);

    gpio_clock_enable(regs);

    GPIO_InitTypeDef init;
    init.Pin = (1 << index);
    init.Mode = GPIO_MODE_IT_RISING;
    uint32_t pull = regs->PUPDR;
#ifdef GPIO_PUPDR_PUPD0
    pull &= (GPIO_PUPDR_PUPD0 << (index * 2));
    init.Pull = (GPIO_PUPDR_PUPD0 & (pull >> (index * 2)));
#else
    pull &= (GPIO_PUPDR_PUPDR0 << (index * 2));
    init.Pull = (GPIO_PUPDR_PUPDR0 & (pull >> (index * 2)));
#endif /* GPIO_PUPDR_PUPD0 */
    //init.Pull = GPIO_PULLDOWN; //GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(regs, &init);

#else
    if (!regs)
        Error_Handler();
    uint32_t const port_idx = GPIO2PORT(pin);
    uint32_t const index = GPIO2IDX(pin);
    uint32_t const bit = g.bit; //GPIO2BIT(pin);

    /* Configure the External Interrupt or event for the current IO */
#ifdef AFIO_BASE
    __IO uint32_t * exticr_reg = &AFIO->EXTICR[index >> 2u];
#elif defined(EXTI) && defined(STM32G0xx)
#define EXTI_IDX_OFFSET 0x8
    __IO uint32_t * exticr_reg = &EXTI->EXTICR[index >> 2u];
#elif defined(SYSCFG_BASE)
    __IO uint32_t * exticr_reg = &SYSCFG->EXTICR[index >> 2u];
#endif
#ifndef EXTI_IDX_OFFSET
#define EXTI_IDX_OFFSET 0x4
#endif
    uint32_t EXTICR = *exticr_reg;
    CLEAR_BIT(EXTICR, (0x0FU << (EXTI_IDX_OFFSET * (index & 0x03))));
    SET_BIT(EXTICR, (port_idx << (EXTI_IDX_OFFSET * (index & 0x03))));
    *exticr_reg = EXTICR;

    /* Configure the interrupt mask */
    if ((it_mode & GPIO_MODE_IT) == GPIO_MODE_IT) {
        SET_BIT(EXTI->IMR, bit);
    } else {
        CLEAR_BIT(EXTI->IMR, bit);
    }

    /* Configure the event mask */
    if ((it_mode & GPIO_MODE_EVT) == GPIO_MODE_EVT) {
        SET_BIT(EXTI->EMR, bit);
    } else {
        CLEAR_BIT(EXTI->EMR, bit);
    }

    /* Enable or disable the rising trigger */
    if ((it_mode & RISING) == RISING) {
        SET_BIT(EXTI->RTSR, bit);
    } else {
        CLEAR_BIT(EXTI->RTSR, bit);
    }

    /* Enable or disable the falling trigger */
    if ((it_mode & FALLING) == FALLING) {
        SET_BIT(EXTI->FTSR, bit);
    } else {
        CLEAR_BIT(EXTI->FTSR, bit);
    }
#endif

    write_u32(&gpio_irq_conf[index].callback, (uint32_t)callback);

    // Enable and set EXTI Interrupt
    NVIC_SetPriority(
        gpio_irq_conf[index].irqnb,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_EXTI, 0));
    NVIC_EnableIRQ(gpio_irq_conf[index].irqnb);
    // Clear pending IRQ
    EXTI->PR = (1 << index);
}

void gpio_in_isr_remove(struct gpio_in const g)
{
    if (!gpio_in_valid(g))
        Error_Handler();
    uint32_t index = ffs(g.bit) - 1;
    irqstatus_t irq = irq_save();
    //NVIC_DisableIRQ(gpio_irq_conf[index].irqnb);
    write_u32(&gpio_irq_conf[index].callback, 0);
    irq_restore(irq);
}

void FAST_CODE_1
gpio_in_isr_clear_pending(struct gpio_in const g)
{
    if (gpio_in_valid(g))
        // Clear pending IRQ
        EXTI->PR = g.bit;
}

/*********************/

void FAST_CODE_1
GPIO_EXTI_IRQHandler(uint16_t const pin)
{
    /* EXTI line interrupt detected */
    uint32_t const index = 0x1 << pin;
    if (EXTI->PR & index) {
        if (gpio_irq_conf[pin].callback != NULL) {
            gpio_irq_conf[pin].callback();
        }
        EXTI->PR = index;
    }
}

/*********************/

struct gpio_adc gpio_adc_setup(uint32_t pin)
{
    return {.adc = NULL, .chan = 0};
}

uint32_t gpio_adc_sample(struct gpio_adc g)
{
    return 0;
}

uint16_t gpio_adc_read(struct gpio_adc g)
{
    return 0;
}

void gpio_adc_cancel_sample(struct gpio_adc g)
{

}
