#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

#define CRSF_v3_PORT_ID 0x5 // 3bits = 0 ... 7


class CRSF_RX : public CRSF
{
public:
    CRSF_RX(HwSerial &dev) : CRSF(&dev) {
        configured_baudrate = CRSF_RX_BAUDRATE;
    }

    void Begin(void);

    void handleUartIn(void);

    void sendRCFrameToFC(rc_channels_rx_t * channels) const;
    void LinkStatisticsSend(LinkStatsLink_t & stats) const;
    void sendMSPFrameToFC(mspPacket_t & msp) const;

    uint32_t current_baudrate(void) const {
        return configured_baudrate;
    }

private:
    void sendFrameToFC(uint8_t *buff, uint8_t size) const;
    void processPacket(uint8_t const *data);
    void negotiate_baud(void) const;
    void change_baudrate(uint32_t baud = CRSF_RX_BAUDRATE);

    uint32_t last_rx_from_fc;
    uint32_t successful_packets_from_fc;
    uint32_t configured_baudrate;
};

#endif /* CRSF_RX_H_ */
