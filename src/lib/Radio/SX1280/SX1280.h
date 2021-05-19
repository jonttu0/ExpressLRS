#pragma once

#include "platform.h"
#include "SX1280_Regs.h"
#include "RadioInterface.h"

// speed up to 10 MHz, SX1280 can handle up to 18MHz
#define SX128X_SPI_SPEED (10000000)
//#define SX128X_SPI_SPEED (4000000)

#define EFE_NO_DOUBLE 1

#if EFE_NO_DOUBLE
typedef uint32_t efe_scaler_t;
#else // !EFE_NO_DOUBLE
typedef double efe_scaler_t;
#endif // EFE_NO_DOUBLE


class SX1280Driver: public RadioInterface
{
public:
    /////////////Packet Stats//////////

    ////////////////Configuration Functions/////////////
    SX1280Driver(uint8_t payload_len = RX_BUFFER_LEN);
    int8_t Begin(int sck, int miso, int mosi);
    void End(void);
    int16_t MeasureNoiseFloor(uint32_t num_meas, uint32_t freq);
    void SetMode(SX1280_RadioOperatingModes_t OPmode);
    void Config(uint32_t bw, uint32_t sf, uint32_t cr,
                uint32_t freq, uint16_t PreambleLength,
                uint8_t crc = 0, uint8_t flrc = 0);
    void SetOutputPower(int8_t power, uint8_t init=0);
    void SetFrequency(uint32_t freq);
    int32_t GetFrequencyError();
    void setPPMoffsetReg(int32_t error_hz, uint32_t frf = 0);

    void TXnb(const uint8_t *data, uint8_t length, uint32_t freq = 0);
    void RXnb(uint32_t freq = 0);
    void StopContRX(void);

    int8_t GetLastPacketRSSI();

    // Internal ISR callbacks, don't use these!
    void TXnbISR(uint16_t irqs);
    void RXnbISR(uint32_t rx_us, uint16_t irqs);

    uint16_t GetIRQFlags();
    void ClearIrqStatus(uint16_t irqMask);

private:
    SX1280_RadioOperatingModes_t currOpmode;
    efe_scaler_t p_efe_scaler;
    uint8_t packet_mode;
    uint8_t LastRadioStatus;

    void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr);
    void ConfigModParamsFLRC(uint8_t bw, uint8_t cr, uint8_t bt=SX1280_FLRC_BT_0_5);
    void SetPacketParamsLoRa(uint8_t HeaderType,
                             uint8_t crc,
                             uint8_t InvertIQ,
                             uint8_t PreambleLength,
                             uint8_t PayloadLength);
    void SetPacketParamsFLRC(uint8_t HeaderType,
                             uint8_t crc,
                             uint8_t PreambleLength,
                             uint8_t PayloadLength);
    void SetPacketType(uint8_t type);
    void SetAutoFs(uint8_t enabled);
    void SetHighSensitivityMode(uint8_t enabled);
    void SetRegulatorMode(uint8_t mode);

    void SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    int32_t GetRxBufferAddr(void);
    uint8_t GetLastPacketStatus(void);

    ///////// SPI Interface
    void WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) const;
    void ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) const;
    void TransferBuffer(uint8_t *buffer, uint8_t size, uint8_t read) const {
        WaitOnBusy();
        transfer(buffer, size, read);
    }
};
