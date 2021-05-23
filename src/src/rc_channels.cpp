#include "rc_channels.h"
#include "common.h"
#include "helpers.h"
#include "FHSS.h"
#include "debug_elrs.h"

#include <assert.h>

#if N_SWITCHES < 2
#error "At least two (2) AUX channels are needed!"
#endif
#if 12 < N_SWITCHES
#error "Up to 12 AUX channels supported!"
#endif


/*************************************************************************************
 * RC OTA PACKET
 *************************************************************************************/

#if (16 < N_CHANNELS)
#error "CRSF Channels Config is not OK"
#endif

typedef struct
{
    // The analog channels
    unsigned rc1 : 10;
    unsigned rc2 : 10;
    unsigned rc3 : 10;
    unsigned rc4 : 10;
    // Packet type
    unsigned pkt_type : 2;
    // The round-robin switch
#if N_SWITCHES <= 5
    unsigned aux_n_idx : 2; // 4 other channels
    unsigned aux_n : 2;
    unsigned aux0 : 2; // aux0 is sent always
#elif N_SWITCHES <= 8
    unsigned aux_n_idx : 3;
    unsigned aux_n : 3;
#else
    unsigned aux_n_idx : 4;
    unsigned aux_n : 2;
#endif
} PACKED RcDataPacket_s;

//#if (OTA_PACKET_DATA < sizeof(RcDataPacket_s))
//#error "Min OTA size is 6 bytes!"
//#endif

static_assert(sizeof(RcDataPacket_s) <= OTA_PACKET_DATA,
              "OTA pkt size is not correct");


// Holds last state of the switches
uint8_t DRAM_ATTR currentSwitches[N_SWITCHES];

// Packed OTA packet buffer
// NOTE! ESP32 requires aligned buffer
uint8_t DMA_ATTR packed_buffer[OTA_PACKET_SIZE];

// bitmap of changed switches
uint16_t DRAM_ATTR p_auxChannelsChanged;
// which switch should be sent in the next rc packet
uint8_t DRAM_ATTR p_nextSwitchIndex;


/**
 * Determine which switch to send next.
 *
 * If any switch has changed since last sent, it sends the lowest index changed switch.
 * If no switches have changed then this sends p_nextSwitchIndex and increment the value.
 */
FORCED_INLINE uint8_t
getNextSwitchIndex(void)
{
    uint16_t changed = p_auxChannelsChanged;
#if (N_SWITCHES <= 5)
    changed &= ~(0x1);
#endif

    /* Check if channel is changed and send it immediately,
     * otherwise send next sequential switch */
    int8_t index = __builtin_ffs(changed) - 1;
    if ((index < 0) || (N_SWITCHES < index)) {
        uint8_t s_index = p_nextSwitchIndex;
        index = s_index++;
        s_index %= N_SWITCHES;
#if N_SWITCHES <= 5
        if (!s_index) s_index++;
#endif
        p_nextSwitchIndex = s_index;
        changed = 0;
    } else {
        changed &= ~(0x1 << index);
    }
    p_auxChannelsChanged = changed;
    return index; // % N_SWITCHES;
}

/**
 * Sequential switches packet
 *
 * Cycle through N_SWITCHES switches on successive packets. If any switches have
 * changed takes the lowest indexed one and send that, hence lower indexed switches have
 * higher priority in the event that several are changed at once.
 */
FORCED_INLINE void
channels_pack(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4)
{
    // find the next switch to send
    uint8_t ch_idx = getNextSwitchIndex();

    uint32_t irq = _SAVE_IRQ();
    RcDataPacket_s *rcdata = (RcDataPacket_s *)&packed_buffer[0];
    // The analog channels, scale down to 10bits
    rcdata->rc1 = (ch1 >> 1);
    rcdata->rc2 = (ch2 >> 1);
    rcdata->rc3 = (ch3 >> 1);
    rcdata->rc4 = (ch4 >> 1);
    // The round-robin switch
#if N_SWITCHES <= 5
    rcdata->aux0 = currentSwitches[0];
    rcdata->aux_n_idx = ch_idx - 1; // 1...4 => 0...3
#else
    rcdata->aux_n_idx = ch_idx;
#endif
    rcdata->aux_n = currentSwitches[ch_idx];
    // Set type
    rcdata->pkt_type = UL_PACKET_RC_DATA;
    _RESTORE_IRQ(irq);
}

/**
 * Convert received OTA packet data to CRSF packet for FC
 */
void FAST_CODE_1 RcChannels_channels_extract(uint8_t const *const input,
                                             rc_channels_rx_t &PackedRCdataOut)
{
    RcDataPacket_s *rcdata = (RcDataPacket_s *)&input[0];
    uint16_t switchValue;
    uint8_t switchIndex;

#if PROTOCOL_ELRS_TO_FC
#define RC_SCALE 0
#else // PROTOCOL_ELRS_TO_FC
#define RC_SCALE 1
#endif // PROTOCOL_ELRS_TO_FC

#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
    PackedRCdataOut.ch_idx = 0;
#endif

    // The analog channels, scale packet to 11bit
    PackedRCdataOut.ch0 = ((uint16_t)rcdata->rc1 << RC_SCALE);
    PackedRCdataOut.ch1 = ((uint16_t)rcdata->rc2 << RC_SCALE);
    PackedRCdataOut.ch2 = ((uint16_t)rcdata->rc3 << RC_SCALE);
    PackedRCdataOut.ch3 = ((uint16_t)rcdata->rc4 << RC_SCALE);
    // The round-robin switch
    switchIndex = rcdata->aux_n_idx;
#if PROTOCOL_ELRS_TO_FC
#if N_SWITCHES <= 5
    PackedRCdataOut.ch4 = SWITCH2b_to_3b(rcdata->aux0);
    switchValue = SWITCH2b_to_3b(rcdata->aux_n);
    switchIndex++; // values 1...4
#elif N_SWITCHES <= 8
    switchValue = rcdata->aux_n;
#else
    switchValue = SWITCH2b_to_3b(rcdata->aux_n);
#endif
#else // PROTOCOL_ELRS_TO_FC
#if N_SWITCHES <= 5
    PackedRCdataOut.ch4 = SWITCH2b_to_CRSF(rcdata->aux0);
    switchValue = SWITCH2b_to_CRSF(rcdata->aux_n);
    switchIndex++; // values 1...4
#elif N_SWITCHES <= 8
    switchValue = SWITCH3b_to_CRSF(rcdata->aux_n);
#else
    switchValue = SWITCH2b_to_CRSF(rcdata->aux_n);
#endif
#endif // PROTOCOL_ELRS_TO_FC

    switch (switchIndex) {
#if 5 < N_SWITCHES
        case 0:
            PackedRCdataOut.ch4 = switchValue;
            break;
#endif
        case 1:
            PackedRCdataOut.ch5 = switchValue;
            break;
#if 2 < N_SWITCHES
        case 2:
            PackedRCdataOut.ch6 = switchValue;
            break;
#endif
#if 3 < N_SWITCHES
        case 3:
            PackedRCdataOut.ch7 = switchValue;
            break;
#endif
#if 4 < N_SWITCHES
        case 4:
            PackedRCdataOut.ch8 = switchValue;
            break;
#endif
#if 5 < N_SWITCHES
        case 5:
            PackedRCdataOut.ch9 = switchValue;
            break;
#endif
#if 6 < N_SWITCHES
        case 6:
            PackedRCdataOut.ch10 = switchValue;
            break;
#endif
#if 7 < N_SWITCHES
        case 7:
            PackedRCdataOut.ch11 = switchValue;
            break;
#endif
#if 8 < N_SWITCHES
        case 8:
            PackedRCdataOut.ch12 = switchValue;
            break;
#endif
#if 9 < N_SWITCHES
        case 9:
            PackedRCdataOut.ch13 = switchValue;
            break;
#endif
#if 10 < N_SWITCHES
        case 10:
            PackedRCdataOut.ch14 = switchValue;
            break;
#endif
#if 11 < N_SWITCHES
        case 11:
            PackedRCdataOut.ch15 = switchValue;
            break;
#endif
        default:
            break;
    }
}

/**
 * Convert received CRSF serial packet from handset and
 * store it to local buffers for OTA packet
 */
void RcChannels_processChannels(rc_channels_t const *const rcChannels)
{
    // channels input range: 0...2048
    uint16_t ChannelDataIn[N_SWITCHES] = {
        (uint16_t)rcChannels->ch4,
        (uint16_t)rcChannels->ch5,
#if 2 < N_SWITCHES
        (uint16_t)rcChannels->ch6,
#endif
#if 3 < N_SWITCHES
        (uint16_t)rcChannels->ch7,
#endif
#if 4 < N_SWITCHES
        (uint16_t)rcChannels->ch8,
#endif
#if 5 < N_SWITCHES
        (uint16_t)rcChannels->ch9,
#endif
#if 6 < N_SWITCHES
        (uint16_t)rcChannels->ch10,
#endif
#if 7 < N_SWITCHES
        (uint16_t)rcChannels->ch11
#endif
#if 8 < N_SWITCHES
    , (uint16_t)rcChannels->ch12
#endif
#if 9 < N_SWITCHES
    , (uint16_t)rcChannels->ch13
#endif
#if 10 < N_SWITCHES
    , (uint16_t)rcChannels->ch14
#endif
#if 11 < N_SWITCHES
    , (uint16_t)rcChannels->ch15
#endif
    };
    uint8_t switch_state;

    /**
     * Convert the rc data corresponding to switches to 2 bit values.
     *
     * I'm defining channels 4 through 11 inclusive as representing switches
     * Take the input values and convert them to the range 0 - 2.
     * (not 0-3 because most people use 3 way switches and expect the middle
     *  position to be represented by a middle numeric value)
     */
    for (uint8_t idx = 0; idx < N_SWITCHES; idx++) {
#if N_SWITCHES <= 5 || 8 < N_SWITCHES
        switch_state = CRSF_to_SWITCH2b(ChannelDataIn[idx]);
        if (0b11 < switch_state) switch_state = 0b11;
#else
        // input is 0 - 2048, output is 0 - 7
        switch_state = CRSF_to_SWITCH3b(ChannelDataIn[idx]);
        if (0b111 < switch_state) switch_state = 0b111;
#endif
        // Check if state is changed
        if (switch_state != currentSwitches[idx]) {
            p_auxChannelsChanged |= (0x1 << idx);
            currentSwitches[idx] = switch_state;
        }
    }

    channels_pack(rcChannels->ch0, rcChannels->ch1, rcChannels->ch2, rcChannels->ch3);
}

void FAST_CODE_1 RcChannels_get_packed_data(uint8_t *const output)
{
    uint8_t iter = sizeof(RcDataPacket_s);
    uint32_t irq = _SAVE_IRQ();
    while (iter--)
        output[iter] = packed_buffer[iter];
    _RESTORE_IRQ(irq);
}

uint8_t FAST_CODE_1
RcChannels_get_arm_channel_state(void)
{
#if TX_SKIP_SYNC
    return currentSwitches[AUX_CHANNEL_ARM];
#else
    return 0;
#endif
}

/*************************************************************************************
 * TELEMETRY OTA PACKET
 *************************************************************************************/
#define MSP_OTA_FIRST  (0x1 << 3)
#define MSP_OTA_HEADER (0x1 << 4)
#define MSP_OTA_LAST   (0x1 << 5)

typedef struct {
    union {
        struct
        {
            uint16_t func;
            uint16_t payloadSize;
            uint8_t flags;
        } PACKED hdr;
        struct
        {
            uint8_t data[5];
        } PACKED payload;
    };
    uint8_t flags;
} PACKED TlmDataPacket_s;

static_assert(sizeof(TlmDataPacket_s) <= OTA_PACKET_DATA,
              "OTA pkt size is not correct");

uint8_t FAST_CODE_1
RcChannels_tlm_ota_send(uint8_t *const output,
                        mspPacket_t &packet,
                        uint8_t tx)
{
    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)output;
    uint8_t iter = 0;
    tlm_ptr->flags = (packet.sequence_nbr++) & 0x7;

    if (packet.sequence_nbr == 1) {
        /* First send header */
        tlm_ptr->flags |= MSP_OTA_FIRST;
        tlm_ptr->flags |= MSP_OTA_HEADER;
        tlm_ptr->hdr.flags = packet.flags;
        tlm_ptr->hdr.func = packet.function;
        tlm_ptr->hdr.payloadSize = packet.payloadSize;
        iter = 0xff;
    }

    for (; iter < sizeof(tlm_ptr->payload.data); iter++)
        tlm_ptr->payload.data[iter] = packet.readByte();

    uint8_t const done = packet.iterated();
    if (done)
        /* this is last junk */
        tlm_ptr->flags |= MSP_OTA_LAST;

    // add pkt_type
    tlm_ptr->flags <<= 2;
    tlm_ptr->flags += tx ? (uint8_t)UL_PACKET_MSP : (uint8_t)DL_PACKET_TLM_MSP;
    return done;
}

uint8_t FAST_CODE_1
RcChannels_tlm_ota_receive(uint8_t const *const input,
                           mspPacket_t &packet)
{
    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)input;
    uint8_t iter;
    tlm_ptr->flags >>= 2; // remove pkt_type

    if (tlm_ptr->flags & MSP_OTA_FIRST) {
        /* first junk, reset packet and start reception */
        packet.reset();
        packet.type = MSP_PACKET_TLM_OTA;
        packet.sequence_nbr = tlm_ptr->flags & 0x7;
        if (tlm_ptr->flags & MSP_OTA_HEADER) {
            packet.flags = tlm_ptr->hdr.flags;
            packet.function = tlm_ptr->hdr.func;
            packet.payloadSize = tlm_ptr->hdr.payloadSize;
            //DEBUG_PRINTF("flags: %u, func %u, size %u\n", packet.flags, packet.function, packet.payloadSize);
            goto exit_tlm_rcv;
        }
    }

    //DEBUG_PRINTF("DONE seq %u == %u data: ", (tlm_ptr->flags & 0x7), (packet.sequence_nbr & 0x7));
    if ((tlm_ptr->flags & 0x7) == (packet.sequence_nbr & 0x7)) {
        for (iter = 0; iter < sizeof(tlm_ptr->payload.data) && !packet.iterated(); iter++) {
            packet.addByte(tlm_ptr->payload.data[iter]);
            //DEBUG_PRINTF("0x%X,", tlm_ptr->payload.data[iter]);
        }
    }
    //DEBUG_PRINTF("  \n");
    //DEBUG_PRINTF("DONE size %u, iter %u\n", packet.payloadSize, packet.payloadIterator);

    /* is this the last junk */
    if (tlm_ptr->flags & MSP_OTA_LAST) {
        packet.payloadIterator = 0;
        packet.sequence_nbr = 0;
        return 1;
    }

exit_tlm_rcv:
    packet.sequence_nbr++;
    return 0;
}


/*************************************************************************************
 * LINK STATISTICS OTA PACKET
 *************************************************************************************/
#define FRAMETYPE_LINK_STATISTICS 0x14
void FAST_CODE_1
RcChannels_link_stas_pack(uint8_t *const output,
                          LinkStats_t &input, uint_fast8_t ul_lq)
{
    // NOTE: output is only 5 bytes + 6bits (MSB)!!

    // OpenTX hard codes "rssi" warnings to the LQ sensor for crossfire, so the
    // rssi we send is for display only.
    // OpenTX treats the rssi values as signed.
    uint8_t openTxRSSI = input.link.uplink_RSSI_1;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI > 127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255 - openTxRSSI;
    output[0] = openTxRSSI;
    output[1] = (input.batt.voltage & 0xFF00) >> 8;
    output[2] = input.link.uplink_SNR;
    output[3] = ul_lq;
    output[4] = (input.batt.voltage & 0x00FF);
    output[5] = FRAMETYPE_LINK_STATISTICS << 2;
}

void FAST_CODE_1
RcChannels_link_stas_extract(uint8_t const *const input,
                             LinkStats_t &output,
                             int8_t snr, int16_t rssi)
{
    // NOTE: input is only 5 bytes + 6bits (MSB)!!
    if (rssi < INT8_MIN) rssi = INT8_MIN;
    if (12 < snr) snr = 12;
    else if (snr < -12) snr = 12;

    output.link.downlink_SNR = snr * 10;
    output.link.downlink_RSSI = 120 + rssi;

    if ((input[5] >> 2) == FRAMETYPE_LINK_STATISTICS) {
        output.link.uplink_RSSI_1 = input[0];
        output.link.uplink_RSSI_2 = 0;
        output.link.uplink_SNR = input[2];
        output.link.uplink_Link_quality = input[3];

        output.batt.voltage = ((uint16_t)input[1] << 8) + input[4];
    }
}

/*************************************************************************************
 * GPS OTA PACKET
 *************************************************************************************/
#define FRAMETYPE_GPS 0x02

void FAST_CODE_1
RcChannels_gps_extract(uint8_t const *const input, GpsOta_t & output)
{
    uint8_t type = input[5] >> 2;
    if ((type & 0xf) != FRAMETYPE_GPS)
        return;
    switch (type >> 4) {
        case 3:
            output.latitude = input[0];
            output.latitude <<= 8;
            output.latitude += input[1];
            output.latitude <<= 8;
            output.latitude += input[2];
            output.latitude <<= 8;
            output.latitude += input[3];
            output.speed = input[4];
            output.speed <<= 8;
            break;
        case 2:
            output.longitude = input[0];
            output.longitude <<= 8;
            output.longitude += input[1];
            output.longitude <<= 8;
            output.longitude += input[2];
            output.longitude <<= 8;
            output.longitude += input[3];
            output.speed += input[4];
            break;
        case 1:
            output.heading = input[0];
            output.heading <<= 8;
            output.heading += input[1];
            output.altitude = input[2];
            output.altitude <<= 8;
            output.altitude += input[3];
            output.satellites = input[4];
            output.pkt_cnt = 1;
            break;
    }
}

uint8_t FAST_CODE_1
RcChannels_gps_pack(uint8_t *const output, GpsOta_t & input)
{
    uint8_t type = (input.pkt_cnt << 4) + FRAMETYPE_GPS;
    if (!input.pkt_cnt)
        return 0;
    // GPS block is split into pieces
    switch (input.pkt_cnt--) {
        case 3:
            output[0] = (uint8_t)(input.latitude >> 24);
            output[1] = (uint8_t)(input.latitude >> 16);
            output[2] = (uint8_t)(input.latitude >> 8);
            output[3] = (uint8_t)input.latitude;
            output[4] = (uint8_t)(input.speed >> 8);
            break;
        case 2:
            output[0] = (uint8_t)(input.longitude >> 24);
            output[1] = (uint8_t)(input.longitude >> 16);
            output[2] = (uint8_t)(input.longitude >> 8);
            output[3] = (uint8_t)input.longitude;
            output[4] = (uint8_t)(input.speed);
            break;
        case 1:
            output[0] = (uint8_t)(input.heading >> 8);
            output[1] = (uint8_t)(input.heading);
            output[2] = (uint8_t)(input.altitude >> 8);
            output[3] = (uint8_t)(input.altitude);
            output[4] = input.satellites;
            break;
    }
    output[5] = type << 2;
    return 1;
}
