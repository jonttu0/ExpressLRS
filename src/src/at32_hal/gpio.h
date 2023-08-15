// Code to setup clocks and gpio on stm32f1
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// https://github.com/KevinOConnor/klipper
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __STM32_GPIO_H
#define __STM32_GPIO_H

#include <stdint.h> // uint32_t

#define LOW  0
#define HIGH 1

enum {
    GPIO_MODE_IT  = 1 << 0,
    GPIO_MODE_EVT = 1 << 1,

    FALLING = (GPIO_MODE_IT | (1 << 4)),
    RISING  = (GPIO_MODE_IT | (1 << 3)),
    CHANGE  = (FALLING | RISING),
};


struct gpio_out
{
    void *regs;
    uint32_t bit;
};
struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val);
void gpio_out_reset(struct gpio_out g, uint32_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint32_t val);
uint8_t gpio_out_read(struct gpio_out g);
static inline uint8_t gpio_out_valid(struct gpio_out g) {
    return (!!g.regs);
}

struct gpio_in
{
    void *regs;
    uint32_t bit;
};
struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up);
void gpio_in_reset(struct gpio_in g, int32_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);
static inline uint8_t gpio_in_valid(struct gpio_in g) {
    return (!!g.regs);
}
typedef void (*isr_cb_t)(void);
void gpio_in_isr(struct gpio_in g, isr_cb_t func, uint8_t type);
void gpio_in_isr_remove(struct gpio_in g);
void gpio_in_isr_clear_pending(struct gpio_in g);


struct gpio_adc
{
    void *adc;
    uint32_t chan;
};
struct gpio_adc gpio_adc_setup(uint32_t pin);
uint32_t gpio_adc_read(struct gpio_adc g);

struct spi_config
{
    void *spi;
};
struct spi_config spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode);
void spi_prepare(struct spi_config config);
void spi_transfer(struct spi_config config, uint8_t receive_data, uint8_t len, uint8_t *data);
static inline uint8_t spi_is_valid(struct spi_config spi) {
    return !!spi.spi;
}

struct i2c_config
{
    void *i2c;
    uint8_t addr;
};
//struct i2c_config i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr);
struct i2c_config i2c_setup(uint32_t rate, int scl, int sda, uint8_t addr);
void i2c_transfer(struct i2c_config config, uint8_t receive_data, uint8_t len, uint8_t *data);

#endif /* __STM32_GPIO_H */
