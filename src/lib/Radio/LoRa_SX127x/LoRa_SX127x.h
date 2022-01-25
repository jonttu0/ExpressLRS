#pragma once

#include "platform.h"
#include "RadioInterface.h"
#include <stdint.h>

typedef enum
{
    BW_7_80_KHZ = 0,
    BW_10_40_KHZ = 1,
    BW_15_60_KHZ = 2,
    BW_20_80_KHZ = 3,
    BW_31_25_KHZ = 4,
    BW_41_70_KHZ = 5,
    BW_62_50_KHZ = 6,
    BW_125_00_KHZ = 7,
    BW_250_00_KHZ = 8,
    BW_500_00_KHZ = 9
} Bandwidth;

typedef enum
{
    SF_6,
    SF_7,
    SF_8,
    SF_9,
    SF_10,
    SF_11,
    SF_12
} SpreadingFactor;

typedef enum
{
    CR_4_5,
    CR_4_6,
    CR_4_7,
    CR_4_8
} CodingRate;

#define SX127X_SYNC_WORD    201  //  201 - default ExpressLRS sync word
#define SX127X_SPI_SPEED    10000000

class SX127xDriver: public RadioInterface
{
public:
    SX127xDriver();

    ///////////Radio Variables////////

    ////////////////Configuration Functions/////////////
    int8_t Begin(int sck, int miso, int mosi);
    void End(void);
    void Config(uint32_t bw, uint32_t sf, uint32_t cr,
                uint32_t freq, uint16_t PreambleLength,
                uint8_t crc = 0, uint8_t flrc = 0);

    void SetOutputPower(int8_t Power, uint8_t init=0);
    void SetFrequency(uint32_t freq, uint8_t mode);
    int32_t GetFrequencyError();
    void setPPMoffsetReg(int32_t error_hz, uint32_t frf = 0);

    //////////// Utility Funcitons //////////////////
    int16_t MeasureNoiseFloor(uint32_t num_meas, uint32_t freq);
    void GetLastRssiSnr();
    int16_t GetCurrRSSI() const;
    uint8_t GetIRQFlags();
    void ClearIRQFlags();

    //////////// TX related Functions ///////////////
    void TXnb(const uint8_t *data, uint8_t length, uint32_t freq = 0);
    void TXnbISR(uint8_t irqs); //ISR for non-blocking TX routine

    //////////// RX related Functions ///////////////
    void StopContRX();
    void RXnb(uint32_t freq = 0);
    void RXnbISR(uint32_t rx_us, uint8_t irqs); //ISR for non-blocking RC routine
    uint8_t RunCAD(uint32_t timeout = 500);

private:
    uint32_t p_bw_hz;
    uint32_t p_freqOffset;
    uint8_t p_ppm_off;
    //uint8_t p_isr_mask;
    uint8_t p_last_payload_len;

    uint8_t CheckChipVersion();
    void SX127xConfig(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t syncWord, uint8_t crc);
    void SetPreambleLength(uint16_t PreambleLen);
    void SetMode(uint8_t mode);

    void RxConfig(uint32_t freq);

    FORCED_INLINE void _change_mode_val(uint8_t mode);
    void reg_op_mode_mode_lora(void);
    void reg_dio1_rx_done(void);
    void reg_dio1_tx_done(void);
    void reg_dio1_isr_mask_write(uint8_t mask);
};
