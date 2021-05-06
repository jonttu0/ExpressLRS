#include "LoRa_SX127x.h"
#include "LoRa_SX127x_Regs.h"
#include "debug_elrs.h"
#include <stdio.h>
#include <string.h>

/////////////////////////////////////////////////////////////////
// Note: ignore default LoRaWAM (0x34/52) sync word!

/* Big thanks to AlessandroAU who tested these to work best!! */
uint8_t SX127x_AllowedSyncwords_sf6[] = {
    5,   6,   7,   11,  12,  13,  15,  18,
    21,  23,  26,  29,  30,  31,  33,  34,
    37,  38,  39,  40,  42,  44,  50,  51,
    54,  55,  57,  58,  59,  61,  63,  65,
    67,  68,  71,  77,  78,  79,  80,  82,
    84,  86,  89,  92,  94,  96,  97,  99,
    101, 102, 105, 106, 109, 111, 113, 115,
    117, 118, 119, 121, 122, 124, 126, 127,
    129, 130, 138, 143, 161, 170, 172, 173,
    175, 180, 181, 182, 187, 190, 191, 192,
    193, 196, 199, 201, 204, 205, 208, 209,
    212, 213, 219, 220, 221, 223, 227, 229,
    235, 239, 240, 242, 243, 246, 247, 255
};

uint8_t SX127x_AllowedSyncwords_sf7[] = {
    18, 21, 23, 26, 29, 30, 31, 33, 34, 37,
    38, 39, 40, 42, 44, 50, 51, 54, 55, 57,
    58, 59, 61, 63, 65, 67, 68, 71, 77, 78,
    79, 82, 84, 86, 89, 92, 94, 97, 99, 101,
    102, 105, 106, 109, 111, 113, 115, 117, 118,
    119, 121, 122, 124, 126, 127, 129, 130, 138,
    143, 161, 170, 172, 173, 175, 180, 181, 182,
    187, 190, 191, 193, 196, 199, 201, 204, 205,
    209, 212, 213, 219, 220, 221, 223, 227, 229,
    235, 239, 242, 243, 246, 247, 255
};
uint8_t SX127x_AllowedSyncwords_sf8[] = {
    18, 21, 23, 26, 29, 30, 31, 33, 34, 37,
    38, 39, 40, 44, 50, 51, 54, 55, 57,
    58, 59, 61, 63, 65, 67, 68, 71, 77, 78,
    79, 82, 84, 86, 89, 92, 94, 97, 99, 101,
    102, 105, 106, 109, 111, 113, 117, 118, 119,
    121, 122, 124, 126, 127, 129, 130, 138, 143,
    161, 170, 172, 173, 175, 180, 181, 182, 187,
    190, 191, 193, 196, 199, 201, 204, 205, 209,
    212, 213, 219, 220, 221, 223, 227, 229, 235,
    243, 246, 247, 255
};

uint8_t SyncWordFindValid(uint8_t const syncWord, uint8_t const sf)
{
    uint8_t *words_ptr;
    uint8_t iter, num_words;
    if (sf == SX127X_SF_6) {
        words_ptr = SX127x_AllowedSyncwords_sf6;
        num_words = sizeof(SX127x_AllowedSyncwords_sf6);
    } else if (sf == SX127X_SF_7) {
        words_ptr = SX127x_AllowedSyncwords_sf7;
        num_words = sizeof(SX127x_AllowedSyncwords_sf7);
    } else if (sf == SX127X_SF_8) {
        words_ptr = SX127x_AllowedSyncwords_sf8;
        num_words = sizeof(SX127x_AllowedSyncwords_sf8);
    } else {
        return SX127X_SYNC_WORD; // return default
    }

    for (iter = 0; iter < num_words; iter++) {
        if (syncWord <= words_ptr[iter])
            return words_ptr[iter];
    }
    return words_ptr[0];
}

/////////////////////////////////////////////////////////////////

static uint8_t DMA_ATTR p_RegOpMode = 0;

/////////////////////////////////////////////////////////////////

static SX127xDriver * DMA_ATTR instance = NULL;

static void FAST_CODE_1 _rxtx_isr_handler_dio0(void)
{
    SX127xDriver * const _radio = instance;
    uint32_t rx_us = micros();
    uint8_t irqs = _radio->GetIRQFlags();
    enum isr_states state = _radio->isr_state_get();
    if (state == RX_DONE) {
        _radio->RXnbISR(rx_us, irqs);
    } else if (state == TX_DONE) {
        _radio->TXnbISR(irqs);
    } else {
        if (irqs & SX127X_CLEAR_IRQ_FLAG_TX_DONE)
            state = TX_DONE;
        else if (irqs & SX127X_CLEAR_IRQ_FLAG_RX_TIMEOUT)
            state = RX_TIMEOUT;
        else if (irqs & SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR)
            state = CRC_ERROR;
        else if (irqs & SX127X_CLEAR_IRQ_FLAG_RX_DONE)
            state = RX_DONE;
        else if (irqs & SX127X_CLEAR_IRQ_FLAG_CAD_DETECTED)
            state = CAD_DETECTED;
        else if (irqs & SX127X_CLEAR_IRQ_FLAG_CAD_DONE)
            state = CAD_DONE;
        _radio->isr_state_set(state);
    }
    _radio->ClearIRQFlags();
}

//////////////////////////////////////////////

SX127xDriver::SX127xDriver(uint8_t payload_len):
    RadioInterface(payload_len, SX127X_SPI_READ, SX127X_SPI_WRITE)
{
    instance = this;
    p_bw_hz = 0;
    _syncWord = SX127X_SYNC_WORD;
    current_freq = 0;
    current_power = 0xF; // outside range to make sure the power is initialized
    module_type = MODULE_SX127x;
}

int8_t SX127xDriver::Begin(int sck, int miso, int mosi)
{
    p_bw_hz = 0;
    current_freq = 0;
    current_power = 0xF; // outside range to make sure the power is initialized

    TxRxDisable();

    Reset();

    // initialize low-level drivers
    RadioHalSpi::Begin(SX127X_SPI_SPEED, sck, miso, mosi);

    if (CheckChipVersion() != ERR_NONE) {
        return -1;
    }

    // Store mode register locally
    p_RegOpMode = readRegister(SX127X_REG_OP_MODE);
    // Write enable CAD, RX and TX ISRs
    writeRegister(SX127X_REG_DIO_MAPPING_1,
                  (SX127X_DIO0_CAD_DONE | SX127X_DIO1_CAD_DETECTED |
                   SX127X_DIO0_TX_DONE | SX127X_DIO0_RX_DONE));
    reg_dio1_isr_mask_write(0);

    /* Attach interrupts to pins */
    if (gpio_in_valid(_DIO1))
        gpio_in_isr(_DIO1, _rxtx_isr_handler_dio0, RISING);
    return 0;
}

void SX127xDriver::End(void)
{
    StopContRX();
    if (gpio_in_valid(_DIO1))
        gpio_in_isr_remove(_DIO1);
    if (gpio_out_valid(CS))
        gpio_out_write(CS, 1);
}

void SX127xDriver::SetOutputPower(int8_t Power, uint8_t init)
{
    Power &= 0xF; // 4bits
    if (current_power == Power && !init)
        return;
    uint8_t reg = Power;
    reg |= SX127X_PA_SELECT_BOOST;
#if defined(TARGET_MODULE_LORA1276F30)
    reg |= (0x1 << 4);
#else
    reg |= SX127X_MAX_OUTPUT_POWER;
#endif
    writeRegister(SX127X_REG_PA_CONFIG, reg);
    writeRegister(SX127X_REG_PA_DAC,
        (0x80 | ((Power == 0xf) ? SX127X_PA_BOOST_ON : SX127X_PA_BOOST_OFF)));
    DEBUG_PRINTF("SetOutputPower: %d\n", Power);
    current_power = Power;
}

void SX127xDriver::SetPreambleLength(uint16_t PreambleLen)
{
    if (PreambleLen < 6)
        PreambleLen = 6;
    SetMode(SX127X_SLEEP);
    uint8_t len[2] = {(uint8_t)(PreambleLen >> 16), (uint8_t)(PreambleLen & 0xff)};
    writeRegisterBurst(SX127X_REG_PREAMBLE_MSB, len, sizeof(len));
    SetMode(SX127X_STANDBY);
}

void FAST_CODE_2 SX127xDriver::SetFrequency(uint32_t freq, uint8_t mode)
{
    if (freq == current_freq)
        return;

    if (mode != 0xff)
        SetMode(SX127X_SLEEP);

#if FREQ_USE_DOUBLE
#define FREQ_STEP 61.03515625

    uint32_t FRQ = ((uint32_t)((double)freq / (double)FREQ_STEP));
    uint8_t buff[3] = {(uint8_t)((FRQ >> 16) & 0xFF), (uint8_t)((FRQ >> 8) & 0xFF), (uint8_t)(FRQ & 0xFF)};
#else
    uint64_t frf = ((uint64_t)(freq + p_freqOffset) << 19) / 32000000;
    uint8_t buff[3] = {
        (uint8_t)(frf >> 16),
        (uint8_t)(frf >> 8),
        (uint8_t)(frf >> 0),
    };
#endif
    writeRegisterBurst(SX127X_REG_FRF_MSB, buff, sizeof(buff));
    current_freq = freq;

    if (mode != 0xff && mode != SX127X_SLEEP)
        SetMode(mode);
}

uint8_t SX127xDriver::CheckChipVersion()
{
    uint8_t i = 0, version;
    while (i++ < 10) {
        version = readRegister(SX127X_REG_VERSION);
        if (version == 0x12) {
            DEBUG_PRINTF("SX127x found! (match by REG_VERSION == 0x12)\n");
            return (ERR_NONE);
        }
        DEBUG_PRINTF("SX127x not found! [%u / 10] REG_VERSION == 0x%X\n", i, version);
        delay(200);
    }
    return (ERR_CHIP_NOT_FOUND);
}

int16_t SX127xDriver::MeasureNoiseFloor(uint32_t num_meas, uint32_t freq)
{
    SetFrequency(freq, SX127X_CAD);
    delay(3);

    int noise = 0;
    for (uint32_t iter = 0; iter < num_meas; iter++)
    {
        delayMicroseconds(100);
        noise += GetCurrRSSI();
    }
    noise /= (int)num_meas; // Compiler bug! must cast to int to make this div working!
    return noise;
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// TX functions ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void FAST_CODE_1 SX127xDriver::TXnbISR(uint8_t irqs)
{
    // Ignore if not a TX DONE ISR
    if (!(irqs & SX127X_CLEAR_IRQ_FLAG_TX_DONE))
        return;

    _change_mode_val(SX127X_STANDBY); // Standby mode is automatically set by IC, set it also locally

    //the larger TX/RX modules require that the TX/RX enable pins are toggled
    TxRxDisable();

    TXdoneCallback1();
}

void FAST_CODE_2 SX127xDriver::TXnb(const uint8_t *data, uint8_t length, uint32_t freq)
{
    SetMode(SX127X_STANDBY);

    TxEnable();

    // Set freq if defined
    if (freq)
        SetFrequency(freq, 0xff);

    if (p_last_payload_len != length)
    {
        writeRegister(SX127X_REG_PAYLOAD_LENGTH, length);
        p_last_payload_len = length;
    }
    uint8_t cfg[2] = {SX127X_FIFO_TX_BASE_ADDR_MAX, SX127X_FIFO_TX_BASE_ADDR_MAX};
    writeRegisterBurst(SX127X_REG_FIFO_ADDR_PTR, cfg, sizeof(cfg));
    writeRegisterBurst(SX127X_REG_FIFO, (uint8_t *)data, length);

    ClearIRQFlags();
    gpio_in_isr_clear_pending(_DIO1);
    //reg_dio1_isr_mask_write(SX127X_MASK_IRQ_FLAG_TX_DONE);

    SetMode(SX127X_TX);
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// RX functions ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void FAST_CODE_2 SX127xDriver::RxConfig(uint32_t freq)
{
    SetMode(SX127X_STANDBY);

    RxEnable();

    // Set freq if defined
    if (freq)
        SetFrequency(freq, 0xff);

    if (p_last_payload_len != RX_buffer_size)
    {
        writeRegister(SX127X_REG_PAYLOAD_LENGTH, RX_buffer_size);
        p_last_payload_len = RX_buffer_size;
    }

    /* ESP requires aligned buffer when -Os is not set! */
#if SX127X_FIFO_RX_BASE_ADDR_MAX == 0 && SX127X_FIFO_TX_BASE_ADDR_MAX == 0
    uint32_t cfg = 0;
#else
    uint32_t cfg = SX127X_FIFO_RX_BASE_ADDR_MAX;
    cfg = (cfg << 8) + SX127X_FIFO_TX_BASE_ADDR_MAX;
    cfg = (cfg << 8) + SX127X_FIFO_RX_BASE_ADDR_MAX;
#endif
    //uint8_t WORD_ALIGNED_ATTR cfg[3] = {SX127X_FIFO_RX_BASE_ADDR_MAX,
    //                                    SX127X_FIFO_TX_BASE_ADDR_MAX,
    //                                    SX127X_FIFO_RX_BASE_ADDR_MAX};
    writeRegisterBurst(SX127X_REG_FIFO_ADDR_PTR, (uint8_t *)&cfg, 3);
    ClearIRQFlags();
    gpio_in_isr_clear_pending(_DIO1);

    //reg_dio1_isr_mask_write(SX127X_MASK_IRQ_FLAG_RX_DONE & SX127X_MASK_IRQ_FLAG_PAYLOAD_CRC_ERROR);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t DMA_ATTR RXdataBuffer[16];

void FAST_CODE_1 SX127xDriver::RXnbISR(uint32_t rx_us, uint8_t irqs)
{
    uint8_t * ptr = NULL;
    // Ignore if CRC is invalid
    if ((!(irqs & SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR)) &&
        (irqs & SX127X_CLEAR_IRQ_FLAG_RX_DONE))
    {
        // fetch data from modem
        readRegisterBurst((uint8_t)SX127X_REG_FIFO, RX_buffer_size, (uint8_t *)RXdataBuffer);
        // fetch RSSI and SNR
        GetLastRssiSnr();
        // Push to application if callback is set
        ptr = RXdataBuffer;
    }
    RXdoneCallback1(ptr, rx_us);
}

void SX127xDriver::StopContRX()
{
    SetMode(SX127X_STANDBY);
    TxRxDisable();
}

void FAST_CODE_2 SX127xDriver::RXnb(uint32_t freq)
{
    RxConfig(freq);
    SetMode(SX127X_RXCONTINUOUS);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t SX127xDriver::RunCAD(uint32_t timeout)
{
    isr_state_set(NONE);

    SetMode(SX127X_STANDBY);

    //reg_dio1_isr_mask_write((SX127X_MASK_IRQ_FLAG_CAD_DONE & SX127X_MASK_IRQ_FLAG_CAD_DETECTED));

    SetMode(SX127X_CAD);

    uint32_t startTime = millis();

    while ((isr_state_get() != CAD_DETECTED) && (isr_state_get() != CAD_DONE))
    {
        if (timeout <= (millis() - startTime))
        {
            return (CHANNEL_FREE);
        }
    }
    return (isr_state_get() == CAD_DETECTED) ? (PREAMBLE_DETECTED) : (CHANNEL_FREE);
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// config functions //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

FORCED_INLINE void SX127xDriver::_change_mode_val(uint8_t mode)
{
    p_RegOpMode &= (~SX127X_CAD);
    p_RegOpMode |= (mode & SX127X_CAD);
}

void FAST_CODE_1 SX127xDriver::SetMode(uint8_t mode)
{ //if radio is not already in the required mode set it to the requested mode
    mode &= SX127X_CAD;
    if (mode != (p_RegOpMode & SX127X_CAD))
    {
        //p_RegOpMode &= (~SX127X_CAD);
        //p_RegOpMode |= (mode & SX127X_CAD);
        _change_mode_val(mode);
        writeRegister(SX127X_REG_OP_MODE, p_RegOpMode);
    }
}

void SX127xDriver::Config(uint32_t bw, uint32_t sf, uint32_t cr,
                          uint32_t freq, uint16_t PreambleLength,
                          uint8_t crc)
{
    uint8_t newBandwidth, newSpreadingFactor, newCodingRate;

    if (freq == 0)
        freq = current_freq;

    if ((freq < 137000000) || (freq > 1020000000))
    {
        DEBUG_PRINTF("Invalid Frequnecy!: %u\n", freq);
        return;
    }

    // check the supplied BW, CR and SF values
    switch ((Bandwidth)bw)
    {
        case BW_7_80_KHZ:
            newBandwidth = SX127X_BW_7_80_KHZ;
            break;
        case BW_10_40_KHZ:
            newBandwidth = SX127X_BW_10_40_KHZ;
            break;
        case BW_15_60_KHZ:
            newBandwidth = SX127X_BW_15_60_KHZ;
            break;
        case BW_20_80_KHZ:
            newBandwidth = SX127X_BW_20_80_KHZ;
            break;
        case BW_31_25_KHZ:
            newBandwidth = SX127X_BW_31_25_KHZ;
            break;
        case BW_41_70_KHZ:
            newBandwidth = SX127X_BW_41_70_KHZ;
            break;
        case BW_62_50_KHZ:
            newBandwidth = SX127X_BW_62_50_KHZ;
            break;
        case BW_125_00_KHZ:
            newBandwidth = SX127X_BW_125_00_KHZ;
            break;
        case BW_250_00_KHZ:
            newBandwidth = SX127X_BW_250_00_KHZ;
            break;
        case BW_500_00_KHZ:
            newBandwidth = SX127X_BW_500_00_KHZ;
            break;
        default:
            return;
    }

    switch ((SpreadingFactor)sf)
    {
        case SF_6:
            newSpreadingFactor = SX127X_SF_6;
            break;
        case SF_7:
            newSpreadingFactor = SX127X_SF_7;
            break;
        case SF_8:
            newSpreadingFactor = SX127X_SF_8;
            break;
        case SF_9:
            newSpreadingFactor = SX127X_SF_9;
            break;
        case SF_10:
            newSpreadingFactor = SX127X_SF_10;
            break;
        case SF_11:
            newSpreadingFactor = SX127X_SF_11;
            break;
        case SF_12:
            newSpreadingFactor = SX127X_SF_12;
            break;
        default:
            return;
    }

    switch ((CodingRate)cr)
    {
        case CR_4_5:
            newCodingRate = SX127X_CR_4_5;
            break;
        case CR_4_6:
            newCodingRate = SX127X_CR_4_6;
            break;
        case CR_4_7:
            newCodingRate = SX127X_CR_4_7;
            break;
        case CR_4_8:
            newCodingRate = SX127X_CR_4_8;
            break;
        default:
            return;
    }

    // configure common registers
    SX127xConfig(newBandwidth, newSpreadingFactor, newCodingRate, freq, _syncWord, crc);
    SetPreambleLength(PreambleLength);

    // save the new settings
    current_freq = freq;

    switch ((Bandwidth)bw) {
        case BW_7_80_KHZ:
            p_bw_hz = 7800;
        case BW_10_40_KHZ:
            p_bw_hz = 10400;
        case BW_15_60_KHZ:
            p_bw_hz = 15600;
        case BW_20_80_KHZ:
            p_bw_hz = 20800;
        case BW_31_25_KHZ:
            p_bw_hz = 31250;
        case BW_41_70_KHZ:
            p_bw_hz = 41667;
        case BW_62_50_KHZ:
            p_bw_hz = 62500;
        case BW_125_00_KHZ:
            p_bw_hz = 125000;
        case BW_250_00_KHZ:
            p_bw_hz = 250000;
        case BW_500_00_KHZ:
            p_bw_hz = 500000;
        default:
            p_bw_hz = 0;
    }
}

void SX127xDriver::SX127xConfig(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t syncWord, uint8_t crc)
{
    uint8_t reg;

    // set mode to SLEEP
    SetMode(SX127X_SLEEP);

    // set LoRa mode
    reg_op_mode_mode_lora();

    SetFrequency(freq, 0xff); // 0xff = skip mode set calls

    // output power configuration
    SetOutputPower(current_power, 1);
    // // 15 = 120mA, 18 = 150mA, 23 = 200ma
    writeRegister(SX127X_REG_OCP, SX127X_OCP_ON | 23);
    // output power configuration
    //writeRegister(SX127X_REG_PA_DAC, (0x80 | SX127X_PA_BOOST_OFF));

    //writeRegister(SX127X_REG_LNA, SX127X_LNA_GAIN_1 | SX127X_LNA_BOOST_ON);

    // turn off frequency hopping
    writeRegister(SX127X_REG_HOP_PERIOD, SX127X_HOP_PERIOD_OFF);

    // Invert IQ according to sync word
    //reg = 0x27 | ((syncWord & 0x1) << 6);
    //writeRegister(SX127X_REG_INVERT_IQ, reg);

    // basic setting (bw, cr, sf, header mode and CRC)
    reg = (sf | SX127X_TX_MODE_SINGLE); // RX timeout MSB = 0b00
    reg |= (crc) ? SX127X_RX_CRC_MODE_ON : SX127X_RX_CRC_MODE_OFF;
    writeRegister(SX127X_REG_MODEM_CONFIG_2, reg);

    if (sf == SX127X_SF_6)
    {
        reg = SX127X_DETECT_OPTIMIZE_SF_6;
        writeRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_6);
    }
    else
    {
        reg = SX127X_DETECT_OPTIMIZE_SF_7_12;
        writeRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_7_12);
    }
    if (bw == SX127X_BW_500_00_KHZ)
        reg |= (1u << 7); // Errata: bit 7 to 1
    writeRegister(SX127X_REG_DETECT_OPTIMIZE, reg);

    //  Errata fix
    switch (bw)
    {
        case SX127X_BW_7_80_KHZ:
            p_freqOffset = 7810;
            reg = 0x48;
            break;
        case SX127X_BW_10_40_KHZ:
            p_freqOffset = 10420;
            reg = 0x44;
            break;
        case SX127X_BW_15_60_KHZ:
            p_freqOffset = 15620;
            reg = 0x44;
            break;
        case SX127X_BW_20_80_KHZ:
            p_freqOffset = 20830;
            reg = 0x44;
            break;
        case SX127X_BW_31_25_KHZ:
            p_freqOffset = 31250;
            reg = 0x44;
            break;
        case SX127X_BW_41_70_KHZ:
            p_freqOffset = 41670;
            reg = 0x44;
            break;
        case SX127X_BW_62_50_KHZ:
        case SX127X_BW_125_00_KHZ:
        case SX127X_BW_250_00_KHZ:
            p_freqOffset = 0;
            reg = 0x40;
            break;
        case SX127X_BW_500_00_KHZ:
        default:
            p_freqOffset = 0;
            reg = 0;
            break;
    }

    if (reg)
        writeRegister(0x2F, reg);

    if (bw != SX127X_BW_500_00_KHZ)
        writeRegister(0x30, 0x00);

    // set the sync word
    syncWord = SyncWordFindValid(syncWord, sf);
    DEBUG_PRINTF("Using sync word %u (input %u)\n",
                 syncWord, _syncWord);
    writeRegister(SX127X_REG_SYNC_WORD, syncWord);

    reg = SX127X_AGC_AUTO_ON;
    if ((bw == SX127X_BW_125_00_KHZ) && ((sf == SX127X_SF_11) || (sf == SX127X_SF_12)))
    {
        reg |= SX127X_LOW_DATA_RATE_OPT_ON;
    }
    else
    {
        reg |= SX127X_LOW_DATA_RATE_OPT_OFF;
    }
    writeRegister(SX127X_REG_MODEM_CONFIG_3, reg);

    reg = bw | cr | SX127X_HEADER_IMPL_MODE;
    writeRegister(SX127X_REG_MODEM_CONFIG_1, reg);

    if (bw == SX127X_BW_500_00_KHZ)
    {
        //datasheet errata reconmendation http://caxapa.ru/thumbs/972894/SX1276_77_8_ErrataNote_1.1_STD.pdf
        writeRegister(0x36, 0x02);
        writeRegister(0x3a, 0x64);
    }
    else
    {
        writeRegister(0x36, 0x03);
    }

    // set mode to STANDBY
    SetMode(SX127X_STANDBY);
}

void FAST_CODE_2 SX127xDriver::setPPMoffsetReg(int32_t error_hz, uint32_t frf)
{
    if (!frf) // use locally stored value if not defined
        frf = current_freq;
    if (!frf)
        return;
    // Calc new PPM
    error_hz *= 95;
    error_hz /= (frf / 10000);

    uint8_t regValue = (uint8_t)error_hz;
    if (regValue == p_ppm_off)
        return;
    p_ppm_off = regValue;
    writeRegister(SX127x_REG_PPMOFFSET, regValue);
}

int32_t FAST_CODE_2 SX127xDriver::GetFrequencyError()
{
    int32_t intFreqError;
    uint8_t fei_reg[3] = {0x0, 0x0, 0x0};
    readRegisterBurst(SX127X_REG_FEI_MSB, sizeof(fei_reg), fei_reg);

    //memcpy(&intFreqError, fei_reg, sizeof(fei_reg));
    //intFreqError = (__builtin_bswap32(intFreqError) >> 8); // 24bit

    intFreqError = fei_reg[0] & 0b0111;
    intFreqError <<= 8;
    intFreqError += fei_reg[1];
    intFreqError <<= 8;
    intFreqError += fei_reg[2];

    if (fei_reg[0] & 0b1000) // Sign bit is on
    {
        // convert to negative
        intFreqError -= 524288;
    }

    // Calculate Hz error where XTAL is 32MHz
    int64_t tmp_f = intFreqError;
    tmp_f <<= 11;
    tmp_f *= p_bw_hz;
    tmp_f /= 1953125000;
    intFreqError = tmp_f;
    //intFreqError = ((tmp_f << 11) * p_bw_hz) / 1953125000;

    return intFreqError;
}

int16_t FAST_CODE_2 SX127xDriver::GetCurrRSSI() const
{
    return (-157 + readRegister(SX127X_REG_RSSI_VALUE));
}

void FAST_CODE_1 SX127xDriver::GetLastRssiSnr()
{
    // Packet SNR + RSSI and current RSSI
    uint8_t resp[] = {0x0, 0x0 /*, 0x0*/};
    readRegisterBurst(SX127X_REG_PKT_SNR_VALUE, sizeof(resp), resp);
    LastPacketSNR = (int8_t)resp[0] / 4;
    LastPacketRSSI = -157 + resp[1];
}

uint8_t FAST_CODE_1 SX127xDriver::GetIRQFlags()
{
    return readRegister(SX127X_REG_IRQ_FLAGS);
}

void FAST_CODE_1 SX127xDriver::ClearIRQFlags()
{
    writeRegister(SX127X_REG_IRQ_FLAGS, 0xff);
}

void SX127xDriver::reg_op_mode_mode_lora(void)
{
    p_RegOpMode |= SX127X_LORA;
    writeRegister(SX127X_REG_OP_MODE, p_RegOpMode);
}

void SX127xDriver::reg_dio1_rx_done(void)
{
    // 0b00 == DIO0 RxDone
    writeRegister(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_RX_DONE);
    writeRegister(SX127X_REG_IRQ_FLAGS_MASK, SX127X_MASK_IRQ_FLAG_RX_DONE);
    //p_isr_mask = SX127X_MASK_IRQ_FLAG_RX_DONE;
}

void SX127xDriver::reg_dio1_tx_done(void)
{
    // 0b00 == DIO0 TxDone
    writeRegister(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_TX_DONE);
    writeRegister(SX127X_REG_IRQ_FLAGS_MASK, SX127X_MASK_IRQ_FLAG_TX_DONE);
    //p_isr_mask = SX127X_MASK_IRQ_FLAG_TX_DONE;
}

void SX127xDriver::reg_dio1_isr_mask_write(uint8_t mask)
{
    // write mask and clear irqs
    uint8_t cfg[2] = {mask, 0xff};
    writeRegisterBurst(SX127X_REG_IRQ_FLAGS_MASK, cfg, sizeof(cfg));

    /*if (p_isr_mask != mask)
    {
        writeRegister(SX127X_REG_IRQ_FLAGS_MASK, mask);
        p_isr_mask = mask;
    }*/
}
