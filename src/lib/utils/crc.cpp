#include "crc.h"

#define CRC16_POLY_NEW  16
//#define CRC16_POLY_NEW  15

#if CRC16_POLY_TESTING
uint8_t CRC16_POLY_PKT[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
#endif


// --------------------------------------------------------
// CRC-XOR implementations

uint8_t FAST_CODE_1 CalcCRCxor(uint8_t const *data, uint16_t length, uint8_t crc)
{
    while (length--) {
        crc = crc ^ *data++;
    }
    return crc;
}

uint8_t FAST_CODE_1 CalcCRCxor(uint8_t const data, uint8_t const crc)
{
    return crc ^ data;
}


// --------------------------------------------------------
// CRC-8 implementations

uint8_t FAST_CODE_1
CalcCRC8(uint8_t const data, uint8_t crc, uint8_t const poly)
{
    int_fast8_t bit = 8;
    crc ^= data;
    while (bit--) // Do eight times.
        crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
    return crc;
}

uint8_t FAST_CODE_1
CalcCRC8len(uint8_t const *data, uint16_t length,
            uint8_t crc, uint8_t const poly)
{
    while (length--)
        crc = CalcCRC8(*data++, crc, poly);
    return crc;
}


// --------------------------------------------------------
// CRC implementations for OTA package

#if (CRC16_POLY_NEW == 16) || (CRC16_POLY_NEW == 15)
// https://users.ece.cmu.edu/~koopman/crc/c16/0x9eb2.txt
// Poly: 0x3D65 (implicit+1 = 0x9EB2), reversed: FALSE, (HD=6 up to 135b)
#define CRC16_POLY 0x3D65
#elif (CRC16_POLY_NEW == 14)
// http://users.ece.cmu.edu/~koopman/crc/c14/0x372b.txt
// Poly: 0x2E57 (implicit+1 = 0x372B), reversed: FALSE, (HD=6 up to 57b)
#define CRC16_POLY 0x2E57
#else
#error "INVALID CONFIG!"
#endif

GenericLutCRC<uint16_t, CRC16_POLY, CRC16_POLY_NEW> DMA_ATTR crc_ota;

uint16_t FAST_CODE_1 CalcCRC16(uint8_t const *data, uint16_t length, uint16_t crc)
{
    return crc_ota.calc(data, length, crc);
}


// --------------------------------------------------------
// CRC-32 implementations

uint32_t CalcCRC32(uint8_t const *data, uint16_t len) {
    uint32_t mask;
    uint32_t crc = 0xFFFFFFFF;
    int_fast8_t bit;
    while (len--) {
        crc = crc ^ *data++; // Get next byte.
        bit = 8;
        while (bit--) { // Do eight times.
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }
    return ~crc;
}

