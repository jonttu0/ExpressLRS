#ifndef _GPIO_H__
#define _GPIO_H__

#include <stdint.h> // uint32_t

#define GPIO_PIN_IVALID ((uint32_t)-1)

struct gpio_out
{
    uint32_t pin;
};
struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val);
void gpio_out_reset(struct gpio_out g, uint32_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint32_t val);
static inline uint8_t gpio_out_valid(struct gpio_out g) {
    return (g.pin != GPIO_PIN_IVALID);
}

struct gpio_in
{
    uint32_t pin;
};
struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up);
void gpio_in_reset(struct gpio_in g, int32_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);
static inline uint8_t gpio_in_valid(struct gpio_in g) {
    return (g.pin != GPIO_PIN_IVALID);
}
typedef void (*isr_cb_t)(void);
void gpio_in_isr(struct gpio_in g, isr_cb_t func, uint8_t type);
void gpio_in_isr_remove(struct gpio_in g);
void gpio_in_isr_clear_pending(struct gpio_in g);

struct gpio_adc
{
    uint32_t pin;
    uint32_t att;
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

#endif /*_GPIO_H__*/
