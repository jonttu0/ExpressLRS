// Code to setup clocks and gpio on stm32f1
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// https://github.com/KevinOConnor/klipper
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __INTERNAL_H_
#define __INTERNAL_H_

#include "hal_inc.h"
#include "gpio.h"

#define _NOP() asm ("nop")


extern gpio_type * digital_regs[];

#define GPIO_NUM_PINS   16
#define GPIO(PORT, NUM) (((PORT) - 'A') * GPIO_NUM_PINS + (NUM))
#define GPIO2PORT(PIN)  ((PIN) / GPIO_NUM_PINS)
#define GPIO2BIT(PIN)   (1U << GPIO2IDX(PIN))
#define GPIO2IDX(PIN)   ((PIN) % GPIO_NUM_PINS)

#define GPIO_INPUT        0
#define GPIO_OUTPUT       1
#define GPIO_AF           2
#define GPIO_ANALOG       3
#define GPIO_INPUT_PULLUP 4 /* For arduino ide compatibility */
#define GPIO_OPEN_DRAIN   0x100
#define GPIO_FUNCTION(fn) (GPIO_AF | ((fn) << 4))

#define CONFIG_CLOCK_FREQ     system_core_clock

#define clockCyclesPerMicrosecond()  ( CONFIG_CLOCK_FREQ / 1000000U )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

extern unsigned int system_core_clock;

void shutdown(const char * reason);

uint8_t timer_is_before(uint16_t time1, uint16_t time2);
uint8_t timer_is_before(uint32_t time1, uint32_t time2);

void enable_pclock(uint32_t periph_base);
uint32_t is_enabled_pclock(uint32_t periph_base);
uint32_t get_pclock_frequency(uint32_t periph_base);
void gpio_clock_enable(gpio_type *regs);
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);

enum {
    DMA_USART_RX,
    DMA_USART_TX,
};

void* dma_get(uint32_t periph, uint8_t type, uint8_t index);
void* dma_channel_get(uint32_t periph, uint8_t type, uint8_t index);
uint8_t dma_channel_idx_get(uint32_t periph, uint8_t type, uint8_t index);
uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index);
void dma_request_config(uint32_t periph, uint8_t type, uint8_t index);

uint32_t uart_peripheral_get(uint32_t pin);
uint8_t uart_pin_is_tx(int32_t pin);
uint8_t uart_tx_pin_is_rx(int32_t pin);
void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin);
#endif
