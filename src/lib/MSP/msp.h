#pragma once

#include "platform.h"
#include "helpers.h"
#include "crc.h"

enum {
    ELRS_INT_MSP_PARAMS = 1,
    ELRS_INT_MSP_DEV_INFO,
    ELRS_INT_MSP_ESPNOW_UPDATE,

    ELRS_HANDSET_BASE = 100,
    ELRS_HANDSET_CALIBRATE = (ELRS_HANDSET_BASE + 1),
    ELRS_HANDSET_MIXER,
    ELRS_HANDSET_ADJUST,
    ELRS_HANDSET_ADJUST_MIN,
    ELRS_HANDSET_ADJUST_MID,
    ELRS_HANDSET_ADJUST_MAX,
    ELRS_HANDSET_CONFIGS_LOAD,
    ELRS_HANDSET_CONFIGS_SAVE,
    ELRS_HANDSET_TLM_LINK_STATS,
    ELRS_HANDSET_TLM_BATTERY, // not used, to be removed...
    ELRS_HANDSET_TLM_GPS,
};

#define MSP_PORT_INBUF_SIZE 256

#define MSP_VERSION     (1U << 5)
#define MSP_STARTFLAG   (1U << 4)
#define MSP_ERRORFLAG   (1U << 5) // MSP RESP
#define MSP_ELRS_INT    (3U << 0)

typedef enum
{
    MSP_IDLE,
    MSP_MSP_START,
    MSP_HEADER_M, // MSPv1
    MSP_HEADER_X, // MSPv2

    MSP_FLAGS,

    MSP_PAYLOAD_SIZE,
    MSP_PAYLOAD_FUNC,
    MSP_PAYLOAD,
    MSP_CHECKSUM,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} mspState_e;

typedef enum
{
    MSP_PACKET_UNKNOWN,
    MSP_PACKET_TLM_OTA, // Used to carry info OTA
    MSP_PACKET_V1_ELRS,
    MSP_PACKET_V1_CMD,
    MSP_PACKET_V1_RESP,
    MSP_PACKET_V2_COMMAND,
    MSP_PACKET_V2_RESPONSE
} mspPacketType_e;

enum /* MSPv1 functions */
{
    MSP_VTX_CONFIG = 0x58,     // read
    MSP_VTX_SET_CONFIG = 0x59, // write
    MSP_EEPROM_WRITE = 250,
};

enum /* MSPv2 functions */
{
    MSP_ELRS_FUNC = 0x4578, // ['E','x']

    MSP_ESPNOW_BIND_FUNC = 0x454E, // ['E', 'N']

    MSP_CHORUS32_LAP_TIME = 0x4c54, // ['L', 'T']
};

typedef struct
{
    uint8_t flags;
    uint16_t function;
    uint16_t payloadSize;
} PACKED mspHeaderV2_t;

typedef struct
{
    mspPacketType_e type;
    uint8_t WORD_ALIGNED_ATTR payload[MSP_PORT_INBUF_SIZE];
    uint16_t function;
    uint16_t payloadSize;
    uint16_t payloadIterator;
    uint8_t flags;
    uint8_t sequence_nbr;
    uint8_t crc;
    bool error;

    inline uint8_t iterated()
    {
        return ((type != MSP_PACKET_UNKNOWN) &&
                ((0 < payloadSize && payloadSize <= payloadIterator) || (payloadSize == 0)));
    }

    uint8_t is_free(void) const
    {
        return (type == MSP_PACKET_UNKNOWN);
    }

    void reset(void)
    {
        type = MSP_PACKET_UNKNOWN;
        flags = 0;
        function = 0;
        payloadSize = 0;
        payloadIterator = 0;
        error = false;
        sequence_nbr = 0;
        crc = 0;
    }

    inline void addByte(uint8_t const b)
    {
        if (payloadIterator >= sizeof(payload)) {
            error = true;
            return;
        }
        crc = CalcCRCxor(b, crc);
        payload[payloadIterator++] = b;
    }

    inline void add(uint8_t b)
    {
        addByte(b);
    }

    inline void add(uint8_t const * data, size_t len)
    {
        while(len--) addByte(*data++);
    }

    inline void FAST_CODE_2 setIteratorToSize()
    {
        payloadSize = payloadIterator;
        payloadIterator = 0;
    }

    uint8_t FAST_CODE_2 readByte()
    {
        if (iterated()) {
            // We are trying to read beyond the length of the payload
            error = true;
            return 0;
        }
        return payload[payloadIterator++];
    }
} mspPacket_t;

/////////////////////////////////////////////////

class MSP
{
public:
    MSP() {}
    bool processReceivedByte(uint8_t c);
    bool mspOngoing() {
        return (m_inputState != MSP_IDLE);
    }
    bool mspReceived() {
        return (m_inputState == MSP_COMMAND_RECEIVED);
    }
    mspPacket_t &getPacket() {
        return m_packet;
    }
    mspPacket_t *getPacketPtr() {
        return &m_packet;
    }
    uint8_t error() {
        return m_packet.error;
    }
    void markPacketFree() {
        // Set input state to idle, ready to receive the next packet
        // The current packet data will be discarded internally
        m_inputState = MSP_IDLE;
    }
    static size_t bufferPacket(uint8_t *output_ptr, mspPacketType_e type,
                             uint16_t function, uint8_t flags,
                             uint8_t len, uint8_t const * payload);
    static size_t bufferPacket(uint8_t *output_ptr, mspPacket_t *packet);
    static bool sendPacket(CtrlSerial *port, mspPacketType_e type,
                           uint16_t function, uint8_t flags,
                           uint8_t len, uint8_t const * payload);
    static bool sendPacket(mspPacket_t *packet, CtrlSerial *port);

private:
    mspPacket_t m_packet;
    mspState_e m_inputState = MSP_IDLE;
    uint16_t m_offset = 0;
    uint8_t m_crc = 0, m_crc_v1 = 0;
};
