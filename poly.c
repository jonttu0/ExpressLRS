#include "stdint.h"
#include "stdio.h"


#define HIGH_BIT 14

#define REVERSED 0

#if HIGH_BIT == 16
//#define POLY 0x1021 // CCITT
#define POLY_IMP 0x9eb2
#elif HIGH_BIT == 14
#define POLY_IMP 0x372B
#elif HIGH_BIT == 13
#define POLY_IMP 0x1E97
#endif

#if HIGH_BIT < 8
#error "min is 8 bits"
#endif

uint16_t crc_ccitt_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t CalcCRC16(uint8_t const *data, uint16_t length, uint16_t crc)
{
    printf("== OLD ==\n");
    uint8_t _byte;
    while (length--) {
        _byte = *data++;
        //printf("  %2X -> [%3u] = 0x%4X\n", _byte, ((crc ^ _byte) & 0xff), crc_ccitt_table[(crc ^ _byte) & 0xff]);
        crc = (crc >> 8) ^ crc_ccitt_table[(crc ^ _byte) & 0xff];
    }
    return crc;
}


uint16_t CRC_TABLE[256];
uint16_t CalcCRC16_new(uint8_t const *data, uint16_t length, uint16_t crc)
{
    printf("== NEW ==\n");
    uint8_t _byte;
    while (length--) {
        _byte = *data++;
        //printf("  %2X -> [%3u] = 0x%4X\n", _byte, ((crc ^ _byte) & 0xff), CRC_TABLE[(crc ^ _byte) & 0xff]);
#if !REVERSED
        crc = (crc << 8) ^ CRC_TABLE[((crc >> 8) ^ _byte)];
#else
        crc = (crc >> 8) ^ CRC_TABLE[(crc ^ _byte) & 0xff];
#endif
    }
    return crc;
}

uint32_t reverse(uint32_t x)
{
    x = ((x >> 1) & 0x55555555u) | ((x & 0x55555555u) << 1);
    x = ((x >> 2) & 0x33333333u) | ((x & 0x33333333u) << 2);
    x = ((x >> 4) & 0x0f0f0f0fu) | ((x & 0x0f0f0f0fu) << 4);
    x = ((x >> 8) & 0x00ff00ffu) | ((x & 0x00ff00ffu) << 8);
    x = ((x >> 16) & 0xffffu) | ((x & 0xffffu) << 16);
    return x;
}

uint8_t CalcParity(uint8_t const *data, uint16_t length)
{
    uint8_t parity = 0u;
    while (length--) {
        uint8_t _d = *data++;
        printf("  parity 0x%02X => %u\n", _d, __builtin_parity(_d));
        parity ^= __builtin_parity(_d);
    }
    return parity;
}



uint8_t
CalcCRC8(uint8_t const data, uint8_t crc, uint8_t const poly)
{
    crc ^= data;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc = crc << 1;
    }
    return crc;
}

uint8_t
CalcCRC8len(uint8_t const *data, uint16_t length,
            uint8_t crc, uint8_t const poly)
{
    while (length--)
        crc = CalcCRC8(*data++, crc, poly);
    return crc;
}

uint8_t smartadioCalcCrc(const uint8_t *data, uint8_t len)
{
#define POLYGEN 0xd5
    uint8_t crc = 0;
    uint_fast8_t ii;

    while (len--) {
        crc ^= *data++;
        ii = 8;
        while (ii--) {
            if ((crc & 0x80) != 0)
                crc = (crc << 1) ^ POLYGEN;
            else
                crc <<= 1;
        }
    }
    return crc;
}


int main(void)
{
    uint32_t const highbit = 0x1 << (HIGH_BIT - 1);
    uint32_t const shift = (HIGH_BIT - 8);
    uint32_t const mask = ((0x1 << HIGH_BIT) - 1);
    uint32_t poly;
    uint16_t index;
    int j, xor_flag;
    uint16_t result;

#ifdef POLY_IMP
    printf("implicit+1 poly: 0x%X\n", POLY_IMP);
    poly = ((POLY_IMP << 1) | 0x1) & mask;
#elif defined(POLY)
    poly = POLY
#else
#error "POLY is not defined!"
#endif
#if REVERSED
    poly = reverse(POLY) >> (32 - HIGH_BIT);
#endif


    printf("#include <stdint.h>\n\n");

    printf("// Poly: 0x%04X, reversed: %s\n", poly, (REVERSED ? "TRUE" : "FALSE"));
    printf("uint16_t DRAM_FORCE_ATTR crc16_table[256] = {\n    ");
    for (index = 0; index < 256; index++) {
#if !REVERSED
        result = index << shift;
        for (j = 0; j < 8; j++) {
            result = (result << 1) ^ ((result & highbit) ? poly : 0);
        }
#else // REVERSED
        result = index;
        for (j = 0; j < 8; j++) {
            if (result & 1)
                result = (result >> 1) ^ poly;
            else
                result >>= 1;
        }
#endif // !REVERSED

        // Mask to fit bit amount
        result &= mask;

        // Store into table
        CRC_TABLE[index] = result;

        /*********************************************/

        printf("0x%04hX,", result);
        /* Pretty formatting */
        if ((index + 1) % 8)
            printf(" ");
        else if (index != 255)
            printf("\n    ");
    }
    printf("\n};\n");


    uint8_t buff[] = {0x13,0x22,0x33, 0x44};

#if 0
    printf("1. %u\n", CalcParity(buff, sizeof(buff)));
    printf("2. %u\n", __builtin_parity(*((uint32_t*)buff)));
    printf("3. %u\n", __builtin_parity(0x44332213));
#endif

    printf("CRC16 OLD: 0x%04X\n", CalcCRC16(buff, sizeof(buff), 0));
    printf("CRC16 NEW: 0x%04X\n", CalcCRC16_new(buff, sizeof(buff), 0));



    uint8_t sa[] = {0xAA, 0x55, (uint8_t)(0x78 << 1), 0x3, 'R', 'S', 'T'};
    //uint8_t sa[] = {0xAA, 0x55, 0x09, 0x02, 0x16, 0xBC};
    uint8_t sa_crc = CalcCRC8len(sa, sizeof(sa), 0, 0xD5);

    for (index = 0; index < sizeof(sa); index++) {
        printf("[%u] = 0x%02X\n", index, sa[index]);
    }

    printf("SA CRC = 0x%02X\n", sa_crc);
    printf("SA CRC = 0x%02X\n", smartadioCalcCrc(sa, sizeof(sa)));


    uint32_t micros = 0xffffff00;
    uint32_t now = 0x100;
    while ((now - micros) < 1000) {
        printf("distance: %u\n", (now - micros));
        now += 100;
    }
    return 0;
}
