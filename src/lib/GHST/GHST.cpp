#include "GHST.h"
#include "CRSF.h" // CRSF_FRAMETYPE_LINK_STATISTICS
#include "crc.h"
#include "platform.h"
#include "utils.h"
#include "debug_elrs.h"


static ghstRcFrame_t rc_data;


void GHST::Begin(void)
{
    rc_data.hdr.addr = GHST_ADDR_FC;
    rc_data.hdr.len = sizeof(rc_data) - offsetof(ghstRcFrame_t, type);
    rc_data.type = GHST_UL_RC_CHANS_HS4_5TO8;

    stats_updated = 0;

    SerialInPacketStart = 0;
    SerialInPacketLen = 0;
    SerialInPacketPtr = 0;
    frameActive = false;

    MspCallback = NULL;
    BattInfoCallback = NULL;
    GpsCallback = NULL;

    _dev->flush_read();
}

void GHST::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
        ptr = ParseInByte(_dev->read());
        if (ptr)
            processPacket(ptr);
    }
}

void GHST::sendRCFrameToFC(rc_channels_rx_t * channels)
{
    rc_data.channels.ch1to4.ch1 = channels->ch0 << 1;
    rc_data.channels.ch1to4.ch2 = channels->ch1 << 1;
    rc_data.channels.ch1to4.ch3 = channels->ch2 << 1;
    rc_data.channels.ch1to4.ch4 = channels->ch3 << 1;

    if (stats_updated) {
        rc_data.type = GHST_UL_RC_CHANS_HS4_RSSI;
        rc_data.channels.stat.lq = uplink_Link_quality;
        rc_data.channels.stat.rssi = -1 * (int8_t)uplink_RSSI;
        stats_updated = 0;
    } else {
        rc_data.type = GHST_UL_RC_CHANS_HS4_5TO8;
        rc_data.channels.aux.cha = channels->ch4 >> 3;
        rc_data.channels.aux.chb = channels->ch5 >> 3;
        rc_data.channels.aux.chc = channels->ch6 >> 3;
        rc_data.channels.aux.chd = channels->ch7 >> 3;
    }
    sendFrameToFC((uint8_t*)&rc_data, sizeof(rc_data));
}

void GHST::LinkStatisticsSend(LinkStatsLink_t & stats)
{
    stats_updated = 1;
    uplink_Link_quality = stats.uplink_Link_quality;
    uplink_RSSI = stats.uplink_RSSI_1;
}

void GHST::sendMSPFrameToFC(mspPacket_t & msp) const
{
    // TODO: not support atm
    (void)msp;
}

void GHST::sendFrameToFC(uint8_t *buff, uint8_t size) const
{
    buff[size - 1] = CalcCRC8len(&buff[sizeof(ghstHeader_t)], (size - sizeof(ghstHeader_t) - 1));
#if !NO_DATA_TO_FC
    uint32_t irq = _SAVE_IRQ();
    _dev->write(buff, size);
    _RESTORE_IRQ(irq);
#endif
}

uint8_t *GHST::ParseInByte(uint8_t inChar)
{
    static uint8_t SerialInBuffer[GHST_FRAME_SIZE_MAX*2];
    uint8_t *packet_ptr = NULL;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer))
    {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        frameActive = false;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // GHST Frame:
    // | address | payload_len | payload* | crc |

    if (frameActive == false) {
        if (inChar == GHST_ADDR_RX) {
            frameActive = true;
            SerialInPacketLen = 0;
        } else {
            SerialInPacketPtr = 0;
        }
    } else {
        if (SerialInPacketLen == 0) {
            // we read the packet length and save it
            SerialInCrc = 0;
            SerialInPacketLen = inChar;
            SerialInPacketStart = SerialInPacketPtr;
            if ((SerialInPacketLen < 2) || (GHST_FRAME_SIZE_MAX < SerialInPacketLen)) {
                // failure -> discard
                frameActive = false;
                SerialInPacketPtr = 0;
            }
        } else {
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen)) {
                /* Check packet CRC */
                if (SerialInCrc == inChar) {
                    packet_ptr = &SerialInBuffer[SerialInPacketStart];
                } else {
#if 0
                    // https://crccalc.com/
                    // CRC algorithm: CRC-8/DVB-S2
                    DEBUG_PRINTF("UART CRC %X != %X\n", SerialInCrc, inChar);
                    for (int i = (SerialInPacketStart-2); i < (SerialInPacketLen + 2); i++)
                    {
                        DEBUG_PRINTF(" 0x%X", SerialInBuffer[i]);
                    }
                    DEBUG_PRINTF("\n");
#elif defined(DEBUG_SERIAL)
                    DEBUG_PRINTF("!C");
#endif
                }

                // packet handled, start next
                frameActive = false;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            } else {
                // Calc crc on the fly
                SerialInCrc = CalcCRC8(inChar, SerialInCrc);
            }
        }
    }

    return packet_ptr;
}

void GHST::processPacket(uint8_t const *data)
{
    switch (data[0]) {
        case CRSF_FRAMETYPE_COMMAND: {
            if (data[1] == ELRS_BOOT_CMD_DEST && data[2] == ELRS_BOOT_CMD_ORIG) {
                platform_reboot_into_bootloader(&data[3]);
            }
            break;
        }
        case GHST_DL_PACK_STAT: {
            ghstTlmDl_t * tlm = (ghstTlmDl_t*)&data[1];
            if (BattInfoCallback) {
                LinkStatsBatt_t batt = {
                    .voltage = tlm->voltage, .current = tlm->current,
                    .capacity = tlm->capacity, .remaining = 0
                };
                BattInfoCallback(&batt);
            }
            break;
        }
    }
}
