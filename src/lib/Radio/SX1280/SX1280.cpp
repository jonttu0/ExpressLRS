#include "SX1280.h"
#include "debug_elrs.h"

/////////////////////////////////////////////////////////////////

static SX1280Driver * DRAM_ATTR instance = NULL;

static void FAST_CODE_1 _rxtx_isr_handler(void)
{
    SX1280Driver * const _radio = instance;
    uint32_t const rx_us = micros();
    uint16_t const irqs = _radio->GetIRQFlags();
    _radio->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);

    switch (_radio->isr_state_get()) {
        case RX_DONE:
            _radio->RXnbISR(rx_us, irqs);
            break;
        case TX_DONE:
            _radio->TXnbISR(irqs);
            break;
        default:
            break;
    };
}

/////////////////////////////////////////////////////////////////

SX1280Driver::SX1280Driver(uint8_t payload_len):
    RadioInterface(payload_len)
{
    instance = this;
    current_freq = 0; //2400000000;
    current_power = -100;
    currOpmode = SX1280_MODE_UNKNOWN_MAX;
    _syncWord = 0;
    module_type = MODULE_SX128x;
}

int8_t SX1280Driver::Begin(int sck, int miso, int mosi)
{
    current_freq = 0; //2400000000;
    current_power = -100;
    currOpmode = SX1280_MODE_UNKNOWN_MAX;

    TxRxDisable();
    // initialize low-level drivers
    RadioHalSpi::Begin(SX128X_SPI_SPEED, sck, miso, mosi);

    if (!gpio_in_valid(_BUSY)) {
        // Error handler!
        DEBUG_PRINTF("[ERROR] BUSY pin is mandatory for SX128x!\n");
        return -1;
    }

    Reset();
    WaitOnBusy();

    uint8_t buffer[] = {
        SX1280_RADIO_READ_REGISTER,
        (uint8_t)(REG_LR_FIRMWARE_VERSION_MSB >> 8),
        (uint8_t)REG_LR_FIRMWARE_VERSION_MSB,
        0, // NOP
        0, 0};
    TransferBuffer(buffer, sizeof(buffer), 1);

    uint16_t firmwareRev = buffer[4];
    firmwareRev <<= 8;
    firmwareRev += buffer[5];
    DEBUG_PRINTF("SX128x fw rev %u\n", firmwareRev); // 43445, 43447
    // 65023 ???
    if (43440 != (firmwareRev & 0xfff0)) {
        DEBUG_PRINTF("[ERROR] Invalid revision!\n");
        return -1;
    }

    if (gpio_in_valid(_DIO1))
        gpio_in_isr(_DIO1, _rxtx_isr_handler, RISING);
    return 0;
}

void SX1280Driver::End(void)
{
    StopContRX();
    if (gpio_in_valid(_DIO1))
        gpio_in_isr_remove(_DIO1);
    if (gpio_out_valid(CS))
        gpio_out_write(CS, 1);
    if (gpio_out_valid(_RST))
        gpio_out_write(_RST, 0);
}

int16_t SX1280Driver::MeasureNoiseFloor(uint32_t num_meas, uint32_t freq)
{
    return -999;
}

void SX1280Driver::Config(uint32_t bw, uint32_t sf, uint32_t cr,
                          uint32_t freq, uint16_t PreambleLength,
                          uint8_t crc)
{
    uint16_t irqs = (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);
    //SetMode(SX1280_MODE_STDBY_XOSC);
    SetMode(SX1280_MODE_STDBY_RC);
    SetRegulatorMode(SX1280_USE_LDO);
    SetPacketType(SX1280_PACKET_TYPE_LORA);
    SetFrequency(freq);
    ConfigModParams(bw, sf, cr);
    SetPacketParams(SX1280_LORA_PACKET_IMPLICIT,
                    (crc) ? SX1280_LORA_CRC_ON : SX1280_LORA_CRC_OFF,
                    ((_syncWord & 0x1) ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL),
                    PreambleLength, RX_buffer_size);
    SetAutoFs(1);
    SetHighSensitivityMode(1);
    if (crc)
        irqs |= SX1280_IRQ_CRC_ERROR;
    // Config IRQs
    SetDioIrqParams(SX1280_IRQ_RADIO_ALL,
                    irqs,
                    SX1280_IRQ_RADIO_NONE,
                    SX1280_IRQ_RADIO_NONE);
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
}

void SX1280Driver::SetPacketParams(uint8_t HeaderType,
                                   uint8_t crc,
                                   uint8_t InvertIQ,
                                   uint8_t PreambleLength,
                                   uint8_t PayloadLength)
{
    uint8_t buf[8];
    buf[0] = SX1280_RADIO_SET_PACKETPARAMS;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = crc;
    buf[5] = InvertIQ;
    buf[6] = 0x00;
    buf[7] = 0x00;
    TransferBuffer(buf, sizeof(buf), 0);
}

void SX1280Driver::SetPacketType(uint8_t type)
{
    uint8_t cmd[] = {
        SX1280_RADIO_SET_PACKETTYPE,
        type
    };
    TransferBuffer(cmd, sizeof(cmd), 0);
}

void SX1280Driver::SetAutoFs(uint8_t enabled)
{
    uint8_t cmd[] = {
        SX1280_RADIO_SET_AUTOFS,
        (uint8_t)(enabled ? 0x1 : 0x0)
    };
    TransferBuffer(cmd, sizeof(cmd), 0);
}

void SX1280Driver::SetHighSensitivityMode(uint8_t enabled)
{
    /* High sensitivity mode is enabled by setting bits 7:6 at address 0x891 to 0x3
     *      Once enabled the noise figure of the receiver is improved by up to
     *      3 dB for 500 μA of additional current consumption.
     */
    uint8_t buffer[] = {
        SX1280_RADIO_READ_REGISTER,
        (uint8_t)(SX1280_REG_SENSITIVITY >> 8),
        (uint8_t)SX1280_REG_SENSITIVITY,
        0, // NOP
        0};
    TransferBuffer(buffer, sizeof(buffer), 1);

    enabled = (enabled ? (0x3 << 6) : 0x0) | (buffer[4] & ~(0x3 << 6));

    buffer[0] = SX1280_RADIO_WRITE_REGISTER;
    buffer[3] = enabled;
    TransferBuffer(buffer, 4, 0);
}

void SX1280Driver::SetRegulatorMode(uint8_t mode)
{
    uint8_t cmd[] = {SX1280_RADIO_SET_REGULATORMODE, mode};
    TransferBuffer(cmd, sizeof(cmd), 0);
}

void FAST_CODE_2 SX1280Driver::SetMode(SX1280_RadioOperatingModes_t OPmode)
{
    WORD_ALIGNED_ATTR uint8_t buffer[4];
    uint8_t len = 2;

    if (OPmode == currOpmode) {
       return;
    }

    switch (OPmode)
    {

    case SX1280_MODE_SLEEP:
        buffer[0] = SX1280_RADIO_SET_SLEEP;
        buffer[1] = 0x1;
        break;

    case SX1280_MODE_STDBY_RC:
        buffer[0] = SX1280_RADIO_SET_STANDBY;
        buffer[1] = SX1280_STDBY_RC;
        break;
    case SX1280_MODE_STDBY_XOSC:
        buffer[0] = SX1280_RADIO_SET_STANDBY;
        buffer[1] = SX1280_STDBY_XOSC;
        break;

    case SX1280_MODE_FS:
        buffer[0] = SX1280_RADIO_SET_FS;
        buffer[1] = 0x00;
        break;

    case SX1280_MODE_RX:
        buffer[0] = SX1280_RADIO_SET_RX;
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buffer[1] = 0x02; // periodBase: 0x2 = 1ms, page 71 datasheet
        buffer[2] = 0xFF; // periodBaseCount: 0xffff = Rx Continuous mode.
        buffer[3] = 0xFF;
        len = 4;
        break;

    case SX1280_MODE_TX:
        buffer[0] = SX1280_RADIO_SET_TX;
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buffer[1] = 0x02; // periodBase: 0x2 = 1ms, page 71 datasheet
        buffer[2] = 0x00; // periodBaseCount: 0x0 = no timeout, returns when TX is ready
        buffer[3] = 0x00;
        len = 4;
        break;

    case SX1280_MODE_CALIBRATION:
    case SX1280_MODE_CAD:
    default:
        return;
    }
    TransferBuffer(buffer, len, 0);
    currOpmode = OPmode;
}

void SX1280Driver::ConfigModParams(uint8_t bw, uint8_t sf, uint8_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[4] = {SX1280_RADIO_SET_MODULATIONPARAMS, sf, bw, cr};
    TransferBuffer(rfparams, sizeof(rfparams), 0);

    rfparams[0] = SX1280_RADIO_WRITE_REGISTER;
    rfparams[1] = 0x9; // address = 0x925
    rfparams[2] = 0x25;
    switch (sf)
    {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        rfparams[3] = 0x1E;
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        rfparams[3] = 0x37;
        break;
    default:
        rfparams[3] = 0x32;
        break;
    }
    TransferBuffer(rfparams, sizeof(rfparams), 0);

#if EFE_NO_DOUBLE
    /* This cause a bit of Hz error which does not effect to functionality
     *   SX1280 can handle 203kHz (BW 0.8MHz) or even 406kHz (BW 1.6MHz)
     *   frequency error.
     */
    switch (bw) {
        case SX1280_LORA_BW_0200:
            p_efe_scaler = 315;
            break;
        case SX1280_LORA_BW_0400:
            p_efe_scaler = 630;
            break;
        case SX1280_LORA_BW_0800:
            p_efe_scaler = 1259;
            break;
        case SX1280_LORA_BW_1600:
            p_efe_scaler = 2519; // max value fits just to int32_t
            break;
    }
#else // EFE_NO_DOUBLE
    switch (bw) {
        case SX1280_LORA_BW_0200:
            p_efe_scaler = 314.84375; // 203.125 * 1.55
            break;
        case SX1280_LORA_BW_0400:
            p_efe_scaler = 629.6875; // 406.25 * 1.55
            break;
        case SX1280_LORA_BW_0800:
            p_efe_scaler = 1259.375; // 812.5 * 1.55
            break;
        case SX1280_LORA_BW_1600:
            p_efe_scaler = 2518.75; // 1625.0 * 1.55
            break;
    }
#endif // EFE_NO_DOUBLE
}

void SX1280Driver::SetOutputPower(int8_t power, uint8_t init)
{
    power += 18;
    if (power < 0) power = 0;           //  0 = -18dBm
    else if (power > 31) power = 31;    // 31 = +13dBm

    // Skip if already set
    if (power == current_power && !init)
        return;

    uint8_t buf[] = {SX1280_RADIO_SET_TXPARAMS, (uint8_t)power, SX1280_RADIO_RAMP_04_US};
    TransferBuffer(buf, sizeof(buf), 0);
    DEBUG_PRINTF("SetOutputPower: %d\n", (power - 18));
    current_power = power;
}

void FAST_CODE_2 SX1280Driver::SetFrequency(uint32_t Reqfreq)
{
    // Skip if already set
    if (current_freq == Reqfreq) return;
    // equation: freq = Reqfreq / (52000000 / (1 << 18))
#if 0
    uint32_t freq = (uint32_t)((double)Reqfreq / SX1280_FREQ_STEP);
#else
#define ALLOW_FREQ_ERR 1
#if ALLOW_FREQ_ERR
    // This cause a bit freq error but LoRa can handle it easily :)
    //   equation: (f / 203125) * 1024;
    uint32_t freq = Reqfreq / 203125;
    freq <<= 10;
#else
    // equation: (1024 * f) / 203125
    uint32_t freq = ((uint64_t)Reqfreq << 10u) / 203125;
#endif
#endif
    uint8_t buf[4];
    buf[0] = SX1280_RADIO_SET_RFFREQUENCY;
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);

    TransferBuffer(buf, sizeof(buf), 0);
    current_freq = Reqfreq;
}

int32_t FAST_CODE_2 SX1280Driver::GetFrequencyError()
{
#if EFE_NO_DOUBLE
    int32_t efe;
#else
    double efeHz;
    int32_t efe = 0;
#endif
    uint8_t fei_reg[7] = {
        SX1280_RADIO_READ_REGISTER,
        (uint8_t)(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB >> 8),
        (uint8_t)(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB),
        0, // NOP
        0x0, 0x0, 0x0};

    TransferBuffer(fei_reg, sizeof(fei_reg), 1);
    efe = fei_reg[4] & 0b0111;
    efe <<= 8;
    efe += fei_reg[5];
    efe <<= 8;
    efe += fei_reg[6];

     // Check the sign bit
    if (fei_reg[4] & 0b1000) {
        // convert to negative
        efe -= 524288;
    }

#if EFE_NO_DOUBLE
    efe *= p_efe_scaler;
    return (efe / 1600);
#else // EFE_NO_DOUBLE
    efeHz = efe;
    efeHz *= p_efe_scaler;
    efeHz /= 1600;
    return (int32_t)efeHz;
#endif // EFE_NO_DOUBLE
}

void FAST_CODE_2 SX1280Driver::setPPMoffsetReg(int32_t error_hz, uint32_t frf)
{
    // Apply freq error correction
}

void FAST_CODE_1 SX1280Driver::TXnbISR(uint16_t irqs)
{
    // Ignore if not a TX DONE ISR
    if (!(irqs & SX1280_IRQ_TX_DONE))
        return;

    currOpmode = SX1280_MODE_FS;
    TXdoneCallback1();
}

void FAST_CODE_2 SX1280Driver::TXnb(const uint8_t *data, uint8_t length, uint32_t freq)
{
    SetMode(SX1280_MODE_FS);
    TxEnable(); // do first to allow PA stablise
    gpio_in_isr_clear_pending(_DIO1);
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (freq)
        SetFrequency(freq);
    SetFIFOaddr(0x00, 0x00);   // not 100% sure if needed again
    WriteBuffer(0x00, (uint8_t*)data, length); //todo fix offset to equal fifo addr
    SetMode(SX1280_MODE_TX);
}

static uint8_t DMA_ATTR RXdataBuffer[16];

void FAST_CODE_1 SX1280Driver::RXnbISR(uint32_t rx_us, uint16_t irqs)
{
    int32_t FIFOaddr;
    // Ignore if not a RX DONE ISR, CRC fail or timeout
    if (!(irqs & SX1280_IRQ_RX_DONE) ||
            (irqs & (SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT))) {
        RXdoneCallback1(NULL, rx_us); // Error!
        return;
    }
    currOpmode = SX1280_MODE_FS;
    GetLastRssiSnr();
    FIFOaddr = GetRxBufferAddr();
    if (FIFOaddr < 0) // RX len is not correct!
        return;
    ReadBuffer(FIFOaddr, RXdataBuffer, RX_buffer_size);
    RXdoneCallback1(RXdataBuffer, rx_us);
}

void FAST_CODE_2 SX1280Driver::RXnb(uint32_t freq)
{
    SetMode(SX1280_MODE_FS);
    RxEnable();
    gpio_in_isr_clear_pending(_DIO1);
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (freq)
        SetFrequency(freq);
    //SetFIFOaddr(0x00, 0x00);
    SetMode(SX1280_MODE_RX);
}

void SX1280Driver::StopContRX(void)
{
    SetMode(SX1280_MODE_STDBY_RC);
    TxRxDisable();
}

int8_t FAST_CODE_2 SX1280Driver::GetLastPacketRSSI()
{
    // Instantaneous RSSI is updated at every symbol received
    uint8_t buff[] = {SX1280_RADIO_GET_RSSIINST, 0, 0};
    TransferBuffer(buff, sizeof(buff), 1);
    //LastRadioStatus = buff[1];
    return (-((int)buff[2])) / 2;
};


/*************************************************************************************
 * PRIVATE METHODS
 *************************************************************************************/

void FAST_CODE_2 SX1280Driver::SetFIFOaddr(uint8_t const txBaseAddr, uint8_t const rxBaseAddr)
{
    uint8_t buf[] = {SX1280_RADIO_SET_BUFFERBASEADDRESS, txBaseAddr, rxBaseAddr};
    TransferBuffer(buf, sizeof(buf), 0);
}

uint16_t FAST_CODE_2 SX1280Driver::GetIRQFlags()
{
    uint16_t irqs;
    uint8_t buff[] = {SX1280_RADIO_GET_IRQSTATUS, 0, 0, 0};
    TransferBuffer(buff, sizeof(buff), 1);
    //LastRadioStatus = buff[1];
    irqs = buff[2];
    irqs <<= 8;
    irqs += buff[3];
    return irqs;
}

void FAST_CODE_2 SX1280Driver::ClearIrqStatus(uint16_t const irqMask)
{
    uint8_t buf[] = {SX1280_RADIO_CLR_IRQSTATUS, (uint8_t)(irqMask >> 8), (uint8_t)irqMask};
    TransferBuffer(buf, sizeof(buf), 0);
}

void SX1280Driver::SetDioIrqParams(uint16_t const irqMask, uint16_t const dio1Mask,
                                   uint16_t const dio2Mask, uint16_t const dio3Mask)
{
    uint8_t buf[9];
    buf[0] = SX1280_RADIO_SET_DIOIRQPARAMS;
    buf[1] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[2] = (uint8_t)(irqMask & 0x00FF);
    buf[3] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[4] = (uint8_t)(dio1Mask & 0x00FF);
    buf[5] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[6] = (uint8_t)(dio2Mask & 0x00FF);
    buf[7] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[8] = (uint8_t)(dio3Mask & 0x00FF);
    TransferBuffer(buf, sizeof(buf), 0);
}

int32_t FAST_CODE_1 SX1280Driver::GetRxBufferAddr(void)
{
    uint8_t status[] = {SX1280_RADIO_GET_RXBUFFERSTATUS, 0, 0, 0};
    TransferBuffer(status, sizeof(status), 1);
    // [1] status, [2] rxPayloadLength, [3] rxStartBufferPointer
    //LastRadioStatus = status[0];
    //return (status[2] == sizeof(RXdataBuffer)) ? status[2] : -1;
    return status[3];
}

void FAST_CODE_1 SX1280Driver::GetLastRssiSnr(void)
{
    // RSSI value latched upon the detection of the sync address
    // SNR on last packet received (estimation)
    uint8_t buff[] = {SX1280_RADIO_GET_PACKETSTATUS, 0, 0, 0};
    TransferBuffer(buff, sizeof(buff), 1);
    //LastRadioStatus = buff[1];
    LastPacketRSSI = -(int8_t)(buff[2] / 2);
    LastPacketSNR = (int8_t)buff[3] / 4;
}

///////// SPI Interface

void FAST_CODE_2 SX1280Driver::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) const
{
    WaitOnBusy();
    writeRegisterOffset(SX1280_RADIO_WRITE_BUFFER, offset, buffer, size);
}
void FAST_CODE_2 SX1280Driver::ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) const
{
    WaitOnBusy();
    readRegisterOffset(SX1280_RADIO_READ_BUFFER, offset, buffer, size);
}
