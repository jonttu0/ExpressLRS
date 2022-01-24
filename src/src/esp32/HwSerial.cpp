#include "HwSerial.h"
#include "targets.h"
#include "freertos/semphr.h"
#include "rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "driver/uart.h"

#ifndef CRSF_SERIAL_NBR
#define CRSF_SERIAL_NBR 1
#elif (CRSF_SERIAL_NBR > 2)
#error "Not supported serial!"
#endif

#if CRSF_SERIAL_NBR == 0
#define UART_RXD_IDX (U0RXD_IN_IDX)
#define UART_TXD_IDX (U0TXD_OUT_IDX)
#elif CRSF_SERIAL_NBR == 1
#define UART_RXD_IDX (U1RXD_IN_IDX)
#define UART_TXD_IDX (U1TXD_OUT_IDX)
#elif CRSF_SERIAL_NBR == 2
#define UART_RXD_IDX (U2RXD_IN_IDX)
#define UART_TXD_IDX (U2TXD_OUT_IDX)
#endif

/********************************************************************************
 *                                   PUBLIC
 ********************************************************************************/
HwSerial CrsfSerial(CRSF_SERIAL_NBR);

HwSerial::HwSerial(int uart_nr, int32_t pin, uint8_t inv) :
    HardwareSerial(uart_nr)
{
    (void)pin;
    (void)inv;
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
    HardwareSerial::begin(
        baud, config, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX,
        (GPIO_PIN_RCSIGNAL_RX == GPIO_PIN_RCSIGNAL_TX));
    enable_receiver();
}

void HwSerial::Pause(void)
{
    /* Do nothing */
}

void HwSerial::Continue(void)
{
    /* Do nothing */
}

void IRAM_ATTR HwSerial::enable_receiver(void)
{
    /* Only in case of half-duplex link */
#if (GPIO_PIN_RCSIGNAL_RX == GPIO_PIN_RCSIGNAL_TX)
    // flush cleans RX buffer as well!
    HardwareSerial::flush(); // wait until write ends
    /* Detach TX pin */
    gpio_matrix_out((gpio_num_t)-1, UART_TXD_IDX, true, false);
    /* Attach RX pin */
    gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, UART_RXD_IDX, true);
#endif
    yield();
}

void IRAM_ATTR HwSerial::enable_transmitter(void)
{
    /* Only in case of half-duplex link */
#if (GPIO_PIN_RCSIGNAL_RX == GPIO_PIN_RCSIGNAL_TX)
    delayMicroseconds(20);
    /* Detach RX pin */
    gpio_matrix_in((gpio_num_t)-1, UART_RXD_IDX, false);
    /* Attach TX pin */
    gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 0);
    gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT);
    gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, UART_TXD_IDX, true, false);
#endif
}
