#include "platform.h"
#include "gpio.h"
#include <Arduino.h>
#include <SPI.h>

#define USE_ARDUINO_SPI 0

#if USE_ARDUINO_SPI
SPIClass SpiBus;
#else
#include <esp8266_peri.h>

typedef union {
        uint32_t regValue;
        struct {
                unsigned regL :6;
                unsigned regH :6;
                unsigned regN :6;
                unsigned regPre :13;
                unsigned regEQU :1;
        };
} spiClk_t;

void setClockDivider(uint32_t clockDiv) {
    if(clockDiv == 0x80000000) {
        GPMUX |= (1 << 9); // Set bit 9 if sysclock required
    } else {
        GPMUX &= ~(1 << 9);
    }
    SPI1CLK = clockDiv;
}

static uint32_t ClkRegToFreq(spiClk_t * reg) {
    return (ESP8266_CLOCK / ((reg->regPre + 1) * (reg->regN + 1)));
}

void setFrequency(uint32_t freq) {
    static uint32_t lastSetFrequency = 0;
    static uint32_t lastSetRegister = 0;

    if (freq >= ESP8266_CLOCK) {
        // magic number to set spi sysclock bit (see below.)
        setClockDivider(0x80000000);
        return;
    }

    if (lastSetFrequency == freq && lastSetRegister == SPI1CLK) {
        // do nothing (speed optimization)
        return;
    }

    const spiClk_t minFreqReg = { 0x7FFFF020 };
    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
    if (freq < minFreq) {
        // use minimum possible clock regardless
        setClockDivider(minFreqReg.regValue);
        lastSetRegister = SPI1CLK;
        lastSetFrequency = freq;
        return;
    }

    uint8_t calN = 1;

    spiClk_t bestReg = { 0 };
    int32_t bestFreq = 0;

    // aka 0x3F, aka 63, max for regN:6
    const uint8_t regNMax = (1 << 6) - 1;

    // aka 0x1fff, aka 8191, max for regPre:13
    const int32_t regPreMax = (1 << 13) - 1;

    // find the best match for the next 63 iterations
    while(calN <= regNMax) {

        spiClk_t reg = { 0 };
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.regN = calN;

        while(calPreVari++ <= 1) { // test different variants for Pre (we calculate in int so we miss the decimals, testing is the easyest and fastest way)
            calPre = (((ESP8266_CLOCK / (reg.regN + 1)) / freq) - 1) + calPreVari;
            if(calPre > regPreMax) {
                reg.regPre = regPreMax;
            } else if(calPre <= 0) {
                reg.regPre = 0;
            } else {
                reg.regPre = calPre;
            }

            reg.regL = ((reg.regN + 1) / 2);
            // reg.regH = (reg.regN - reg.regL);

            // test calculation
            calFreq = ClkRegToFreq(&reg);
            //os_printf("-----[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d = %d\n", reg.regValue, freq, reg.regEQU, reg.regPre, reg.regN, reg.regH, reg.regL, calFreq);

            if(calFreq == static_cast<int32_t>(freq)) {
                // accurate match use it!
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            } else if(calFreq < static_cast<int32_t>(freq)) {
                // never go over the requested frequency
                auto cal = std::abs(static_cast<int32_t>(freq) - calFreq);
                auto best = std::abs(static_cast<int32_t>(freq) - bestFreq);
                if(cal < best) {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if(calFreq == static_cast<int32_t>(freq)) {
            // accurate match use it!
            break;
        }
        calN++;
    }

    // os_printf("[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d\t - Real Frequency: %d\n", bestReg.regValue, freq, bestReg.regEQU, bestReg.regPre, bestReg.regN, bestReg.regH, bestReg.regL, ClkRegToFreq(&bestReg));

    setClockDivider(bestReg.regValue);
    lastSetRegister = SPI1CLK;
    lastSetFrequency = freq;
}

void setDataMode(uint8_t dataMode) {

    /**
     SPI_MODE0 0x00 - CPOL: 0  CPHA: 0
     SPI_MODE1 0x01 - CPOL: 0  CPHA: 1
     SPI_MODE2 0x10 - CPOL: 1  CPHA: 0
     SPI_MODE3 0x11 - CPOL: 1  CPHA: 1
     */

    bool CPOL = (dataMode & 0x10); ///< CPOL (Clock Polarity)
    bool CPHA = (dataMode & 0x01); ///< CPHA (Clock Phase)

    // https://github.com/esp8266/Arduino/issues/2416
    // https://github.com/esp8266/Arduino/pull/2418
    if (CPOL)         // Ensure same behavior as
        CPHA ^= 1;    // SAM, AVR and Intel Boards

    if (CPHA) {
        SPI1U |= (SPIUSME);
    } else {
        SPI1U &= ~(SPIUSME);
    }

    if (CPOL) {
        SPI1P |= 1<<29;
    } else {
        SPI1P &= ~(1<<29);
        //todo test whether it is correct to set CPOL like this.
    }
}

inline void setDataBits(uint16_t bits) {
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    bits--;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

#endif

struct spi_config IRAM_ATTR
spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode)
{
    uint8_t datamode[4] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
#if USE_ARDUINO_SPI
    if (SpiBus.pins(sck, miso, mosi, -1)) {
        SpiBus.begin();
        SpiBus.setHwCs(false);
        SpiBus.setBitOrder(MSBFIRST);
        SpiBus.setDataMode(datamode[mode]);
        SpiBus.setFrequency(speed);
        return {.spi = &SpiBus};
    }
    return {.spi = NULL};
#else
    // pins()
    pinMode(sck, SPECIAL);  ///< GPIO14
    pinMode(miso, SPECIAL); ///< GPIO12
    pinMode(mosi, SPECIAL); ///< GPIO13

    // begin()
    SPI1C = 0;
    setFrequency(speed);
    SPI1U = SPIUMOSI | SPIUDUPLEX | SPIUSSE;
    SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO); // 8bits
    SPI1C1 = 0;

    // disable HW CS
    SPI1U &= ~(SPIUCSSETUP | SPIUCSHOLD);

    // MSB first = SPI1C &= ~(SPICWBO | SPICRBO);
    // LSB first = SPI1C |= (SPICWBO | SPICRBO);

    setDataMode(datamode[mode]);
    //setDataBits(8);
    return {.spi = (void*)&SPI1CMD};
#endif
}

void IRAM_ATTR
spi_prepare(struct spi_config config)
{
#if USE_ARDUINO_SPI
    (void)config;
#else
#endif
}

void IRAM_ATTR
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    if (!config.spi) return;
#if USE_ARDUINO_SPI
    SPIClass * spi = (SPIClass*)config.spi;
    if (receive_data)
        spi->transfer(data, len);
    else
        spi->writeBytes(data, len);
#else
    uint8_t rdata;
    while (len--) {
        SPI1W0 = *data;
        SPI1CMD |= SPIBUSY;
        while (SPI1CMD & SPIBUSY) {}
        rdata = (uint8_t) (SPI1W0 & 0xff);
        if (receive_data)
            *data = rdata;
        data++;
    }
#endif
}
