#include "internal.h"
#include "helpers.h"
#include "gpio.h"
#include "at32_def.h"
#include "debug_elrs.h"

struct spi_info
{
    spi_type *spi;
    uint8_t miso_pin, mosi_pin, sck_pin, function;
};

static const struct spi_info spi_bus[] = {
#ifdef SPI2
    {SPI2, GPIO('B', 14), GPIO('B', 15), GPIO('B', 13), GPIO_AF},
#endif
#ifdef SPI1
    {SPI1, GPIO('A', 6), GPIO('A', 7), GPIO('A', 5), GPIO_AF},
#endif
#ifdef SPI3
    {SPI3, GPIO('B', 4), GPIO('B', 5), GPIO('B', 3), GPIO_AF},
#if CONFIG_MACH_STM32F4
    {SPI3, GPIO('C', 11), GPIO('C', 12), GPIO('C', 10), GPIO_AF},
#endif
#endif
};


struct spi_config
spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode)
{
    uint8_t bus;
    for (bus = 0; bus < ARRAY_SIZE(spi_bus); bus++) {
        struct spi_info *spi = (struct spi_info *)&spi_bus[bus];
        if (spi->miso_pin == miso && spi->mosi_pin == mosi && spi->sck_pin == sck)
            break;
    }
    if (bus < ARRAY_SIZE(spi_bus)) {
        // Enable SPI
        spi_type *spi = spi_bus[bus].spi;
        if (!is_enabled_pclock((uint32_t)spi))
        {
            enable_pclock((uint32_t)spi);
            gpio_peripheral(spi_bus[bus].miso_pin, spi_bus[bus].function, 1);
            gpio_peripheral(spi_bus[bus].mosi_pin, spi_bus[bus].function, 0);
            gpio_peripheral(spi_bus[bus].sck_pin, spi_bus[bus].function, 0);
        }

        // Calculate CR1 register
        uint32_t const pclk = get_pclock_frequency((uint32_t)spi);
        uint32_t div = 0;
        while ((pclk >> (div + 1)) > speed && div <= 9)
            div++;

        spi_init_type config;
        config.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
        config.master_slave_mode = SPI_MODE_MASTER;
        config.mclk_freq_division = (spi_mclk_freq_div_type)div;
        config.first_bit_transmission = SPI_FIRST_BIT_MSB;
        config.frame_bit_num = SPI_FRAME_8BIT;
        config.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
        switch (mode) {
            default:
            case 0:
                config.clock_polarity = SPI_CLOCK_POLARITY_LOW;
                config.clock_phase = SPI_CLOCK_PHASE_1EDGE;
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }

        spi_init(spi, &config);
        spi_enable(spi, FALSE);

        return (struct spi_config){.spi = spi};
    }
    return {.spi = NULL};
}

void FAST_CODE_1
spi_prepare(struct spi_config config)
{
    spi_type *spi = (spi_type *)config.spi;
    if (!spi) return;
    spi_enable(spi, TRUE);
}

void FAST_CODE_1
spi_transfer(struct spi_config config, uint8_t receive_data, uint8_t len, uint8_t *data)
{
    spi_type *spi = (spi_type *)config.spi;
    if (!spi) return;
    while (len--) {
        //while (!(spi->sts & (SPI_I2S_TDBE_FLAG)));
        spi->dt = *data;
        while (!(spi->sts & SPI_I2S_RDBF_FLAG));
        uint8_t const rdata = spi->dt;
        if (receive_data)
            *data = rdata;
        data++;
    }
}
