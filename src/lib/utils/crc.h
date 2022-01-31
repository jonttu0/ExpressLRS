#ifndef __CRC_H_
#define __CRC_H_

#include "platform.h"
#include "helpers.h"
#include <stdint.h>

#define CRC_LUT_PRINT   0

// --------------------------------------------------------

#if CRC16_POLY_TESTING
extern uint8_t CRC16_POLY_PKT[5];
#endif

// --------------------------------------------------------

uint8_t CalcCRCxor(uint8_t const *data, uint16_t length, uint8_t crc = 0);
uint8_t CalcCRCxor(uint8_t data, uint8_t crc = 0);

uint8_t CalcCRC8(uint8_t data, uint8_t crc, uint8_t poly=0xD5);
uint8_t CalcCRC8len(uint8_t const *data, uint16_t length, uint8_t crc = 0, uint8_t poly=0xD5);

uint16_t CalcCRC16(uint8_t const *data, uint16_t length, uint16_t crc = 0);

uint32_t CalcCRC32(uint8_t const *data, uint16_t len);

// --------------------------------------------------------

template <typename T, T POLY, uint8_t BITS>
class GenericLutCRC
{
public:
    GenericLutCRC() {
        uint32_t const highbit = 0x1 << (BITS - 1);
        uint32_t const shift = (BITS - 8);
        uint32_t const mask = ((0x1 << BITS) - 1);
        T crc;
        int_fast8_t bit;
        for (size_t index = 0; index < 256; index++) {
            // Create LUT (reversed = false)
            crc = index << shift;
            bit = 8;
            while (bit--)
                crc = (crc << 1) ^ ((crc & highbit) ? POLY : 0);
            crctab[index] = crc & mask;
        }
    }
    T FAST_CODE_1 calc(uint8_t data, T crc = 0) {
        crc = (crc << 8) ^ crctab[((crc >> (BITS - 8)) ^ data) & 0xFF];
        return crc & ((1 << BITS) - 1);
    }
    T FAST_CODE_1 calc(uint8_t const *data, size_t len, T crc = 0) {
        uint_fast8_t _byte;
        while (len--) {
            _byte = *data++;
            crc = (crc << 8) ^ crctab[((crc >> (BITS - 8)) ^ _byte) & 0xFF];
        }
        return crc & ((1 << BITS) - 1);
    }
    /*T calc(volatile uint8_t *data, size_t len, T crc = 0) {
        return calc((uint8_t*)data, len, crc);
    }*/
private:
    T crctab[256];
};

// --------------------------------------------------------

#endif /* __CRC_H_ */
