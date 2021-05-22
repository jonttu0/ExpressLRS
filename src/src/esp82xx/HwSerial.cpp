#include "HwSerial.h"
#include "targets.h"

#ifndef CRSF_SERIAL_NBR
#define CRSF_SERIAL_NBR 0
#elif (CRSF_SERIAL_NBR > 2)
#error "Not supported serial!"
#endif

HwSerial CrsfSerial(CRSF_SERIAL_NBR);

HwSerial::HwSerial(int uart_nr, int32_t pin, uint8_t inv) :
    HardwareSerial(uart_nr)
{
    (void)pin;
    (void)inv;
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
    HardwareSerial::begin(baud, (enum SerialConfig)config);
    enable_receiver();
}

void IRAM_ATTR HwSerial::enable_receiver(void)
{
}

void IRAM_ATTR HwSerial::enable_transmitter(void)
{
}
