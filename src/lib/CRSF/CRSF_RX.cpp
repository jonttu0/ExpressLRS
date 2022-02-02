#include "CRSF_RX.h"
#include "utils.h"
#include "debug_elrs.h"
#include <string.h>

crsf_channels_msg_t DMA_ATTR p_crsf_channels;
crsf_msp_packet_fc_t DMA_ATTR msp_packet;

#if PROTOCOL_ELRS_TO_FC
crsfLinkStatisticsMsg_elrs_t DMA_ATTR link_stat_packet;
#else
#if PROTOCOL_CRSF_V3_TO_FC
static uint32_t DMA_ATTR link_stat_full_sent_us;
crsfLinkStatisticsTxMsg_t DMA_ATTR link_stat_packet_tx;
#endif // PROTOCOL_CRSF_V3_TO_FC
crsfLinkStatisticsMsg_t DMA_ATTR link_stat_packet;
#endif // PROTOCOL_ELRS_TO_FC
static uint32_t DMA_ATTR link_stat_sent_us;

#define LINK_STATS_INTERVAL_MS       100U   // 100ms
#define LINK_STATS_FULL_INTERVAL_MS 2000U   // 2sec
#define MS_TO_US(_ms) ((_ms) * 1000U)


void CRSF_RX::Begin(void)
{
    successful_packets_from_fc = 0;

    link_stat_packet.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    link_stat_packet.header.frame_size = sizeof(link_stat_packet) - CRSF_FRAME_START_BYTES;
#if PROTOCOL_ELRS_TO_FC
    link_stat_packet.header.type = CRSF_FRAMETYPE_LINK_STATISTICS_ELRS;
#else // !PROTOCOL_ELRS_TO_FC
    link_stat_packet.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;

#if PROTOCOL_CRSF_V3_TO_FC
    link_stat_packet_tx.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    link_stat_packet_tx.header.frame_size = sizeof(link_stat_packet_tx) - CRSF_FRAME_START_BYTES;
    link_stat_packet_tx.header.type = CRSF_FRAMETYPE_LINK_STATISTICS_TX;
    link_stat_packet_tx.stats.downlink_power = 0;
    link_stat_packet_tx.stats.uplink_FPS = 0;
#endif //PROTOCOL_CRSF_V3_TO_FC
#endif // PROTOCOL_ELRS_TO_FC

    msp_packet.header.device_addr = CRSF_ADDRESS_BROADCAST;
    msp_packet.header.frame_size = sizeof(msp_packet) - CRSF_FRAME_START_BYTES;
    msp_packet.header.type = CRSF_FRAMETYPE_MSP_WRITE;
    msp_packet.header.dest_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    msp_packet.header.orig_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;

    p_crsf_channels.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    p_crsf_channels.header.frame_size = sizeof(p_crsf_channels) - CRSF_FRAME_START_BYTES;
#if PROTOCOL_ELRS_TO_FC
    p_crsf_channels.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED_ELRS;
#elif PROTOCOL_CRSF_V3_TO_FC
    p_crsf_channels.header.type = CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED;
#else // !PROTOCOL_ELRS_TO_FC && !PROTOCOL_CRSF_V3_TO_FC
    p_crsf_channels.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
#endif // PROTOCOL_ELRS_TO_FC

    CRSF::Begin();

    negotiate_baud();
}

void FAST_CODE_1 CRSF_RX::sendFrameToFC(uint8_t *buff, uint8_t const size) const
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
#if !NO_DATA_TO_FC
    uint32_t irq = _SAVE_IRQ();
    _dev->write(buff, size);
    _RESTORE_IRQ(irq);
#endif
}

void CRSF_RX::LinkStatisticsSend(LinkStatsLink_t & stats, uint32_t now_us) const
{
    if (MS_TO_US(LINK_STATS_INTERVAL_MS) <= (uint32_t)(now_us - link_stat_sent_us)) {
#if PROTOCOL_ELRS_TO_FC
        link_stat_packet.stats.uplink_RSSI =
            stats.active_antenna ? stats.uplink_RSSI_2 : stats.uplink_RSSI_1;
        link_stat_packet.stats.uplink_Link_quality = stats.uplink_Link_quality;
        link_stat_packet.stats.uplink_SNR = stats.uplink_SNR;
        link_stat_packet.stats.rf_Mode = stats.rf_Mode;
        sendFrameToFC((uint8_t*)&link_stat_packet, sizeof(link_stat_packet));
#else // !PROTOCOL_ELRS_TO_FC
#if PROTOCOL_CRSF_V3_TO_FC
        if (MS_TO_US(LINK_STATS_FULL_INTERVAL_MS) <= (uint32_t)(now_us - link_stat_full_sent_us)) {
            link_stat_full_sent_us = now;
#endif // PROTOCOL_CRSF_V3_TO_FC
            link_stat_packet.stats.uplink_RSSI_1 = stats.uplink_RSSI_1;
            link_stat_packet.stats.uplink_RSSI_2 = stats.uplink_RSSI_2;
            link_stat_packet.stats.uplink_Link_quality = stats.uplink_Link_quality;
            link_stat_packet.stats.uplink_SNR = stats.uplink_SNR;
            link_stat_packet.stats.active_antenna = stats.active_antenna;
            link_stat_packet.stats.rf_Mode = stats.rf_Mode;
            link_stat_packet.stats.uplink_TX_Power = stats.uplink_TX_Power;
            link_stat_packet.stats.downlink_RSSI = stats.downlink_RSSI;
            link_stat_packet.stats.downlink_Link_quality = stats.downlink_Link_quality;
            link_stat_packet.stats.downlink_SNR = stats.downlink_SNR;
            sendFrameToFC((uint8_t*)&link_stat_packet, sizeof(link_stat_packet));
#if PROTOCOL_CRSF_V3_TO_FC
        } else {
            link_stat_packet_tx.stats.uplink_RSSI =
                stats.active_antenna ? stats.uplink_RSSI_2 : stats.uplink_RSSI_1;
            link_stat_packet_tx.stats.uplink_RSSI_percentage = 100;
            link_stat_packet_tx.stats.uplink_Link_quality = stats.uplink_Link_quality;
            link_stat_packet_tx.stats.uplink_SNR = stats.uplink_SNR;
            sendFrameToFC((uint8_t*)&link_stat_packet_tx, sizeof(link_stat_packet_tx));
        }
#endif // PROTOCOL_CRSF_V3_TO_FC
#endif // PROTOCOL_ELRS_TO_FC
        link_stat_sent_us = now_us;
    }
}

void FAST_CODE_1 CRSF_RX::sendRCFrameToFC(rc_channels_rx_t * channels) const
{
    memcpy(&p_crsf_channels.data, (void*)channels, sizeof(p_crsf_channels.data));
    sendFrameToFC((uint8_t*)&p_crsf_channels, sizeof(p_crsf_channels));
}

void FAST_CODE_1 CRSF_RX::sendMSPFrameToFC(mspPacket_t & msp) const
{
    uint8_t i;
    msp_packet.msp.flags = MSP_VERSION + msp.sequence_nbr;
    if (msp.payloadIterator == 0 && msp.sequence_nbr == 0) {
        msp_packet.msp.flags |= MSP_STARTFLAG;
    }
    msp.sequence_nbr++;
    for (i = 0; i < sizeof(msp_packet.msp.payload); i++) {
        msp_packet.msp.payload[i] = msp.readByte();
    }
    sendFrameToFC((uint8_t*)&msp_packet, sizeof(msp_packet));
}

void CRSF_RX::negotiate_baud(void) const
{
#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
    /* Skip if already negotiated */
    if (configured_baudrate == CRSF_RX_BAUDRATE_V3)
        return;

    crsf_speed_req req;
    req.header.device_addr = CRSF_ADDRESS_BROADCAST;
    req.header.frame_size = sizeof(req) - CRSF_FRAME_START_BYTES;
    req.header.type = CRSF_FRAMETYPE_COMMAND;
    req.header.dest_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    req.header.orig_addr = CRSF_ADDRESS_CRSF_RECEIVER;
    req.proposal.command = CRSF_COMMAND_SUBCMD_GENERAL;
    req.proposal.sub_command = CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL;
    req.proposal.portID = CRSF_v3_PORT_ID;
    req.proposal.baudrate = BYTE_SWAP_U32(CRSF_RX_BAUDRATE_V3);
    req.crc_cmd = CalcCRC(&req.header.type, (sizeof(req) - 4));
    sendFrameToFC((uint8_t*)&req, sizeof(req));
    delay(20); // Wait DMA to finish its job
#endif // PROTOCOL_CRSF_V3_TO_FC
}

void CRSF_RX::change_baudrate(uint32_t const baud)
{
    /* Skip if already set */
    if (configured_baudrate == baud)
        return;

#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
    _dev->end();
    _dev->Begin(baud);
    Begin();
    configured_baudrate = baud;
#endif // PROTOCOL_CRSF_V3_TO_FC
}

void CRSF_RX::processPacket(uint8_t const *data)
{
    crsf_buffer_t const * const msg = (crsf_buffer_t*)data;

    last_rx_from_fc = millis();
    successful_packets_from_fc++;

    switch (msg->type) {
        case CRSF_FRAMETYPE_COMMAND: {
            if ((msg->command.dest_addr == CRSF_ADDRESS_CRSF_RECEIVER) &&
                (msg->command.orig_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                (msg->command.command == CRSF_COMMAND_SUBCMD_GENERAL)) {

                if (msg->command.sub_command == CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE) {
                    crsf_v3_speed_control_resp_t const * const resp =
                        (crsf_v3_speed_control_resp_t*)msg->command.payload;
                    if (resp->portID == CRSF_v3_PORT_ID) {
#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
                        // Baudrate accepted, configure new baud
                        change_baudrate((resp->status) ? CRSF_RX_BAUDRATE_V3 : CRSF_RX_BAUDRATE);
#endif // PROTOCOL_CRSF_V3_TO_FC
                    }
                }

            } else if ((msg->command.dest_addr == ELRS_BOOT_CMD_DEST) &&
                       (msg->command.orig_addr == ELRS_BOOT_CMD_ORIG)) {
                platform_reboot_into_bootloader(&msg->command.command);
            }
            break;
        }

        case CRSF_FRAMETYPE_DISPLAYPORT_CMD:
        case CRSF_FRAMETYPE_DEVICE_INFO: {
            /* These are sent after startup */
            negotiate_baud();
            break;
        }

        case CRSF_FRAMETYPE_FLIGHT_MODE: {
            break;
        }

        case CRSF_FRAMETYPE_BATTERY_SENSOR: {
            if (BattInfoCallback) {
                LinkStatsBatt_t batt;
                memcpy(&batt, &data[1], sizeof(batt));
                /*
                batt.voltage = data[1];
                batt.voltage <<= 8;
                batt.voltage += data[2];
                // current is in units of 0.1A
                batt.current = data[3];
                batt.current <<= 8;
                batt.current += data[4];
                // capacity is in units of mAh
                batt.capacity = data[5];
                batt.capacity <<= 8;
                batt.capacity += data[6];
                batt.capacity <<= 8;
                batt.capacity += data[7];
                // battery remaining percentage
                batt.remaining = data[8];
                */
                BattInfoCallback(&batt);
            }
            break;
        }

        case CRSF_FRAMETYPE_GPS: {
            if (GpsCallback) {
                GpsOta_t gps;
                memcpy(&gps, &data[1], sizeof(gps) - 1);
                /*
                gps.latitude = data[1];
                gps.latitude <<= 8;
                gps.latitude += data[2];
                gps.latitude <<= 8;
                gps.latitude += data[3];
                gps.latitude <<= 8;
                gps.latitude += data[4];

                gps.longitude = data[5];
                gps.longitude <<= 8;
                gps.longitude += data[6];
                gps.longitude <<= 8;
                gps.longitude += data[7];
                gps.longitude <<= 8;
                gps.longitude += data[8];

                gps.speed = data[9];
                gps.speed <<= 8;
                gps.speed += data[10];

                gps.heading = data[11];
                gps.heading <<= 8;
                gps.heading += data[12];

                gps.altitude = data[13];
                gps.altitude <<= 8;
                gps.altitude += data[14];

                gps.satellites = data[15];
                */
                GpsCallback(&gps);
            }
            break;
        }

        case CRSF_FRAMETYPE_MSP_RESP: {
            if (MspCallback &&
                (msg->extended.dest_addr == CRSF_ADDRESS_RADIO_TRANSMITTER) &&
                (msg->extended.orig_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
                MspCallback(msg->extended.payload);
            }
            break;
        }

        default:
            break;
    }

    if (10 < successful_packets_from_fc) {
        /* Try to increase speed */
        negotiate_baud();
        successful_packets_from_fc = 0;
    }
}

void CRSF_RX::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;

#if !PROTOCOL_ELRS_TO_FC && PROTOCOL_CRSF_V3_TO_FC
    if (1000 <= (millis() - last_rx_from_fc)) {
        change_baudrate(CRSF_RX_BAUDRATE); // reset back to default
    }
#endif // PROTOCOL_CRSF_V3_TO_FC

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
        ptr = ParseInByte(_dev->read());
        if (ptr)
            processPacket(ptr);
    }
}
