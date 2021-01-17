#include "CRSF.h"
#include "debug_elrs.h"

void rcNullCb(uint8_t const *const) {}
void (*CRSF::RCdataCallback1)(uint8_t const *const) = &rcNullCb; // function is called whenever there is new RC data.

void nullCallback(void){};
void (*CRSF::disconnected)() = &nullCallback; // called when CRSF stream is lost
void (*CRSF::connected)() = &nullCallback;    // called when CRSF stream is regained

uint8_t DMA_ATTR SerialInBuffer[/*CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX)*/256];

//#define DBF_PIN_CRSF_PACKET 2
#ifdef DBF_PIN_CRSF_PACKET
struct gpio_out crsf_packet;
#endif

void CRSF::Begin()
{
#ifdef DBF_PIN_CRSF_PACKET
    crsf_packet = gpio_out_setup(DBF_PIN_CRSF_PACKET, 0);
#endif

    GoodPktsCount = 0;
    BadPktsCount = 0;
    SerialInPacketStart = 0;
    SerialInPacketLen = 0;
    SerialInPacketPtr = 0;
    CRSFframeActive = false;

    tlm_gps_valid = 0;

    MspCallback = NULL;
    BattInfoCallback = NULL;
    GpsCallback = NULL;

    _dev->flush_read();
}


void ICACHE_RAM_ATTR CRSF::GpsStatsExtract(uint8_t const *const input)
{
    uint8_t type = input[5] >> 2;
    if ((type & 0xf) != CRSF_FRAMETYPE_GPS)
        return;
    switch (type >> 4) {
        case 3:
            TLMGPSsensor.latitude = input[0];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += input[1];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += input[2];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += input[3];
            TLMGPSsensor.speed = input[4];
            TLMGPSsensor.speed <<= 8;
            break;
        case 2:
            TLMGPSsensor.longitude = input[0];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += input[1];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += input[2];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += input[3];
            TLMGPSsensor.speed += input[4];
            break;
        case 1:
            TLMGPSsensor.heading = input[0];
            TLMGPSsensor.heading <<= 8;
            TLMGPSsensor.heading += input[1];
            TLMGPSsensor.altitude = input[2];
            TLMGPSsensor.altitude <<= 8;
            TLMGPSsensor.altitude += input[3];
            TLMGPSsensor.satellites = input[4];
            tlm_gps_valid = 1;
            break;
    }
}

uint8_t ICACHE_RAM_ATTR CRSF::GpsStatsPack(uint8_t *const output)
{
    uint8_t type = (tlm_gps_valid << 4) + CRSF_FRAMETYPE_GPS;
    if (!tlm_gps_valid)
        return 0;
    // GPS block is split into pieces
    switch (tlm_gps_valid--) {
        case 3:
            output[0] = (uint8_t)(TLMGPSsensor.latitude >> 24);
            output[1] = (uint8_t)(TLMGPSsensor.latitude >> 16);
            output[2] = (uint8_t)(TLMGPSsensor.latitude >> 8);
            output[3] = (uint8_t)TLMGPSsensor.latitude;
            output[4] = (uint8_t)(TLMGPSsensor.speed >> 8);
            break;
        case 2:
            output[0] = (uint8_t)(TLMGPSsensor.longitude >> 24);
            output[1] = (uint8_t)(TLMGPSsensor.longitude >> 16);
            output[2] = (uint8_t)(TLMGPSsensor.longitude >> 8);
            output[3] = (uint8_t)TLMGPSsensor.longitude;
            output[4] = (uint8_t)(TLMGPSsensor.speed);
            break;
        case 1:
            output[0] = (uint8_t)(TLMGPSsensor.heading >> 8);
            output[1] = (uint8_t)(TLMGPSsensor.heading);
            output[2] = (uint8_t)(TLMGPSsensor.altitude >> 8);
            output[3] = (uint8_t)(TLMGPSsensor.altitude);
            output[4] = TLMGPSsensor.satellites;
            break;
    }
    output[5] = type << 2;
    return 1;
}

uint8_t *CRSF::ParseInByte(uint8_t inChar)
{
    uint8_t *packet_ptr = NULL;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer))
    {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        CRSFframeActive = false;
        BadPktsCount++;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // CRSF Frame:
    // | address | payload_len | payload* | crc |

    if (CRSFframeActive == false)
    {
        if (inChar == CRSF_ADDRESS_CRSF_RECEIVER ||
            inChar == CRSF_ADDRESS_CRSF_TRANSMITTER ||
            inChar == CRSF_SYNC_BYTE)
        {
            CRSFframeActive = true;
            SerialInPacketLen = 0;
#ifdef DBF_PIN_CRSF_PACKET
            gpio_out_write(crsf_packet, 1);
#endif
        }
        else
        {
            SerialInPacketPtr = 0;
        }
    }
    else
    {
        if (SerialInPacketLen == 0) // we read the packet length and save it
        {
            SerialInCrc = 0;
            SerialInPacketLen = inChar;
            SerialInPacketStart = SerialInPacketPtr;
            if ((SerialInPacketLen < 2) || (CRSF_FRAME_SIZE_MAX < SerialInPacketLen))
            {
                // failure -> discard
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                BadPktsCount++;
            }
        }
        else
        {
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen))
            {
                /* Check packet CRC */
                if (SerialInCrc == inChar)
                {
                    packet_ptr = &SerialInBuffer[SerialInPacketStart];
                    GoodPktsCount++;
                }
                else
                {
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
                    BadPktsCount++;
                }

#ifdef DBF_PIN_CRSF_PACKET
                gpio_out_write(crsf_packet, 0);
#endif

                // packet handled, start next
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            }
            else
            {
                // Calc crc on the fly
                SerialInCrc = CalcCRC(inChar, SerialInCrc);
            }
        }
    }

    return packet_ptr;
}
