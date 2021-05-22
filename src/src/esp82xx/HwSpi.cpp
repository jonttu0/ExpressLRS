#include "platform.h"
#include "gpio.h"
#include <Arduino.h>
#include <SPI.h>

SPIClass SpiBus;

struct spi_config IRAM_ATTR
spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode)
{
    uint8_t datamode[4] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
    if (SpiBus.pins(sck, miso, mosi, -1)) {
        SpiBus.begin();
        SpiBus.setHwCs(false);
        SpiBus.setBitOrder(MSBFIRST);
        SpiBus.setDataMode(datamode[mode]);
        SpiBus.setFrequency(speed);
        return {.spi = &SpiBus};
    }
    return {.spi = NULL};
}

void IRAM_ATTR
spi_prepare(struct spi_config config)
{
    (void)config;
}

void IRAM_ATTR
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    if (config.spi) {
        SPIClass * spi = (SPIClass*)config.spi;
        if (receive_data)
            spi->transfer(data, len);
        else
            spi->writeBytes(data, len);
    }
}
