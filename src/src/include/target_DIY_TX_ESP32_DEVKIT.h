#pragma once

#define GPIO_PIN_NSS       5  // V_SPI_CS0
#define GPIO_PIN_DIO0      35 //26
#define GPIO_PIN_DIO1      34 //25
#define GPIO_PIN_MOSI      23 // V_SPI
#define GPIO_PIN_MISO      19 // V_SPI
#define GPIO_PIN_SCK       18 // V_SPI
#define GPIO_PIN_RST       -1 //14  // not connected ATM
// so we don't have to solder the extra resistor,
// we switch rx/tx using gpio mux
#define GPIO_PIN_RCSIGNAL_RX 2
#define GPIO_PIN_RCSIGNAL_TX 2
