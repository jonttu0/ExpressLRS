#ifndef __GHST_H
#define __GHST_H

#include "HwSerial.h"
#include "helpers.h"
#include "msp.h"
#include "rc_channels.h"

/* Definitions copied from Betaflight implementation! */

#define GHST_RX_BAUDRATE                420000

//#define GHST_TX_BAUDRATE_FAST           400000
//#define GHST_TX_BAUDRATE_SLOW           115200
//#define GHST_BYTE_TIME_FAST_US          ((1000000/GHST_TX_BAUDRATE_FAST)*10)      // 10 bit words (8 data, 1 start, 1 stop)
//#define GHST_BYTE_TIME_SLOW_US          ((1000000/GHST_TX_BAUDRATE_SLOW)*10)
//#define GHST_UART_WORDLENGTH            UART_WORDLENGTH_8B

typedef enum {
    GHST_ADDR_RADIO             = 0x80,
    GHST_ADDR_TX_MODULE_SYM     = 0x81,     // symmetrical, 400k pulses, 400k telemetry
    GHST_ADDR_TX_MODULE_ASYM    = 0x88,     // asymmetrical, 400k pulses, 115k telemetry
    GHST_ADDR_FC                = 0x82,
    GHST_ADDR_GOGGLES           = 0x83,
    GHST_ADDR_QUANTUM_TEE1      = 0x84,     // phase 2
    GHST_ADDR_QUANTUM_TEE2      = 0x85,
    GHST_ADDR_QUANTUM_GW1       = 0x86,
    GHST_ADDR_5G_CLK            = 0x87,     // phase 3
    GHST_ADDR_RX                = 0x89
} ghstAddr_e;

typedef enum {
    GHST_UL_RC_CHANS_HS4_5TO8   = 0x10,     // High Speed 4 channel, plus CH5-8
    GHST_UL_RC_CHANS_HS4_9TO12  = 0x11,     // High Speed 4 channel, plus CH9-12
    GHST_UL_RC_CHANS_HS4_13TO16 = 0x12,     // High Speed 4 channel, plus CH13-16
    GHST_UL_RC_CHANS_HS4_RSSI   = 0x13,     // primary 4 channel, plus RSSI, LQ, RF Mode, and Tx Power
    GHST_UL_RC_CHANS_HS4_LAST   = 0x1f,     // Last frame type including 4 primary channels
} ghstUl_e;

#define GHST_UL_RC_CHANS_SIZE       12      // 1 (type) + 10 (data) + 1 (crc)

typedef enum {
    GHST_DL_OPENTX_SYNC         = 0x20,
    GHST_DL_LINK_STAT           = 0x21,
    GHST_DL_VTX_STAT            = 0x22,
    GHST_DL_PACK_STAT           = 0x23,     // Battery (Pack) Status
} ghstDl_e;

#define GHST_RC_CTR_VAL_12BIT       0x7C0   // servo center for 12 bit values (0x3e0 << 1)
#define GHST_RC_CTR_VAL_8BIT        0x7C    // servo center for 8 bit values

#define GHST_FRAME_SIZE             14      // including addr, type, len, crc, and payload

#define GHST_PAYLOAD_SIZE_MAX       14

#define GHST_FRAME_SIZE_MAX         24

typedef struct ghstFrameDef_s {
    uint8_t addr;
    uint8_t len;
    uint8_t type;
    uint8_t payload[GHST_PAYLOAD_SIZE_MAX + 1];         // CRC adds 1
} ghstFrameDef_t;

typedef union ghstFrame_u {
    uint8_t bytes[GHST_FRAME_SIZE];
    ghstFrameDef_t frame;
} ghstFrame_t;

/* Pulses payload (channel data), for 4x 12-bit channels */
typedef struct ghstPayloadServo4_s {
    // 48 bits, or 6 bytes
    unsigned int ch1: 12;
    unsigned int ch2: 12;
    unsigned int ch3: 12;
    unsigned int ch4: 12;
} PACKED ghstPayloadServo4_t;

/* Pulses payload (channel data), for 4x 8-bit channels */
typedef struct ghstPayloadChannels_s {
    uint8_t cha;
    uint8_t chb;
    uint8_t chc;
    uint8_t chd;
} PACKED ghstPayloadChannels_t;

/* Pulses payload (channel data), with RSSI/LQ, and other related data */
typedef struct ghstPayloadPulsesRssi_s {
    uint8_t lq;                 // 0-100
    uint8_t rssi;               // 0 - 128 sign inverted, dBm
    uint8_t rfProtocol;
    int8_t  txPwrdBm;           // tx power in dBm, use lookup table to map to published mW values
} PACKED ghstPayloadPulsesRssi_t;


/* Pulses payload (channel data). Includes 4x high speed control channels, plus 4 channels from CH5-CH12 */
typedef struct ghstPayloadPulses_s {
    // 80 bits, or 10 bytes
    ghstPayloadServo4_t ch1to4;
    union {
        ghstPayloadChannels_t aux;
        ghstPayloadPulsesRssi_s stat;
    };
} PACKED ghstPayloadPulses_t;

typedef struct ghstHeader_s {
    uint8_t addr;
    uint8_t len;
} ghstHeader_t;

typedef struct ghstRcFrame_s {
    ghstHeader_t hdr;
    uint8_t type;
    ghstPayloadPulses_t channels;
    uint8_t crc;
} PACKED ghstRcFrame_t;

typedef struct ghstTlmDl_s {
    uint16_t voltage;  // mv * 100
    uint16_t current;  // ma * 100
    uint16_t capacity; // mah * 100
    uint8_t rx_voltage;
    uint8_t tbd1;
    uint8_t tbd2;
    uint8_t tbd3;
} PACKED ghstTlmDl_t;

/* This is just to keep other code compatible */
typedef struct ghstLinkstatistics_s {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t  uplink_SNR;
    uint8_t rf_Mode;
} ghstLinkStatistics_t;

struct crsf_channels_s;

class GHST {
public:
    GHST(HwSerial &dev): _dev(&dev) {}

    void Begin(void);
    void handleUartIn(void);

    void sendRCFrameToFC(rc_channels_rx_t * channels);
    void LinkStatisticsSend(LinkStatsLink_t & stats, uint32_t now_us);
    void sendMSPFrameToFC(mspPacket_t & msp) const;

    MspCallback_t MspCallback;
    BattInfoCallback_t BattInfoCallback;
    GpsCallback_t GpsCallback;

private:
    void sendFrameToFC(uint8_t *buff, uint8_t size) const;
    uint8_t * ParseInByte(uint8_t inChar);
    void processPacket(uint8_t const *data);

    HwSerial * const _dev;

    bool frameActive;
    uint8_t SerialInCrc;
    uint8_t SerialInPacketStart;
    uint8_t SerialInPacketLen;
    uint8_t SerialInPacketPtr;

    volatile uint8_t stats_updated;
    uint8_t uplink_RSSI;
    uint8_t uplink_Link_quality;
};

#endif /* __GHST_H */
