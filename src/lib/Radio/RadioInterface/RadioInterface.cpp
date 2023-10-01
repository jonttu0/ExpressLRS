#include "RadioInterface.h"
#include "platform.h"
#include <Arduino.h>

/////////////////////////////////////////////////////////////////

void RadioInterface::SetPins(int rst, int dio1, int dio2, int dio3,
                             int busy, int txpin, int rxpin, int cs, int papin)
{
    CS = gpio_out_setup(cs, 1);
    _RST = gpio_out_setup(rst, 0);
    _DIO1 = gpio_in_setup(dio1, 0);
    _DIO2 = gpio_in_setup(dio2, 0),
    _DIO3 = gpio_in_setup(dio3, 0),
    _BUSY = gpio_in_setup(busy, -1); // pull down for error handling
    _TXen = gpio_out_setup(txpin, 0);
    _RXen = gpio_out_setup(rxpin, 0);
    _PAen = gpio_out_setup(papin, 0);
}

void RadioInterface::Reset(void)
{
    if (gpio_out_valid(_RST)) {
        delay(100);
        gpio_out_write(_RST, 0);
        delay(100);
        gpio_out_write(_RST, 1);
    }
}

void FAST_CODE_1 RadioInterface::WaitOnBusy() const
{
    while (unlikely(gpio_in_read(_BUSY)));
}

void FAST_CODE_1 RadioInterface::TxEnable()
{
    isr_state_set(TX_DONE);
    if (gpio_out_valid(_PAen))
        gpio_out_write(_PAen, 1);
    if (gpio_out_valid(_RXen))
        gpio_out_write(_RXen, 0);
    if (gpio_out_valid(_TXen))
        gpio_out_write(_TXen, 1);
}

void FAST_CODE_1 RadioInterface::RxEnable()
{
    isr_state_set(RX_DONE);
    if (gpio_out_valid(_PAen))
        gpio_out_write(_PAen, 1);
    if (gpio_out_valid(_TXen))
        gpio_out_write(_TXen, 0);
    if (gpio_out_valid(_RXen))
        gpio_out_write(_RXen, 1);
}

void FAST_CODE_1 RadioInterface::TxRxDisable()
{
    isr_state_set(NONE);
    if (gpio_out_valid(_RXen))
        gpio_out_write(_RXen, 0);
    if (gpio_out_valid(_TXen))
        gpio_out_write(_TXen, 0);
    if (gpio_out_valid(_PAen))
        gpio_out_write(_PAen, 0);
}
