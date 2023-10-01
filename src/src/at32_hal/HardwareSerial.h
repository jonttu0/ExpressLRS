#pragma once

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "irq.h"
#include "platform.h"
#include "hal_inc.h"

#define SERIAL_8N1 0x06

#define UART_BUFF_SIZE 256

#if defined(STM32F7xx)
#ifndef PRINTF_NUM_BLOCKS
#define UART_DMA_TX_BUFF    64
#else
#define UART_DMA_TX_BUFF    PRINTF_NUM_BLOCKS
#endif
#else
#define UART_DMA_TX_BUFF    16
#endif

// Used by the TX DMA
// Note: make sure the TX is not called ofter for same buffer!
struct tx_pool_s {
    struct tx_pool_s * next;
    uint8_t * data_ptr;
    uint32_t len;
};

/* Interface kept arduino compatible */
class HardwareSerial
{
public:
    HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma=0);
    void setTx(uint32_t tx_pin);
    void setRx(uint32_t rx_pin);
    void begin(unsigned long baud, uint8_t mode = SERIAL_8N1);
    void end(void);
    void Pause(void);
    void Continue(void);
    int available(void);
    int read(void);
    void flush(void);
    uint32_t write(const uint8_t *buff, uint32_t len);
    void setTimeout(unsigned long) {}

    void hw_enable_receiver(void);
    void hw_enable_transmitter(void);

    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_buffer[UART_BUFF_SIZE];

    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_buffer[UART_BUFF_SIZE];

    // for DMA TX
    uint8_t* tx_pool_get(uint32_t * len)
    {
        uint8_t * ret = NULL;
        irqstatus_t flag = irq_save();
        struct tx_pool_s * tail = tx_pool_tail_get();
        if (tx_pool_head_get() != tail) {
            ret = tail->data_ptr;
            *len = tail->len;
            tail->data_ptr = NULL;
            tx_pool_tail_set(tail->next);
        }
        irq_restore(flag);
        return ret;
    }

    dma_type* dma_unit_tx;
    dma_type* dma_unit_rx;
    usart_type* p_usart_tx;
    usart_type* p_usart_rx;
    dma_channel_type* dma_ch_tx;
    dma_channel_type* dma_ch_rx;
    uint8_t dma_ch_idx_tx;
    uint8_t dma_ch_idx_rx;
    uint8_t usart_tx_idx;
    uint8_t usart_rx_idx;

protected:
    uint32_t rx_pin;
    uint32_t tx_pin;
    struct gpio_out p_duplex_pin;
    uint8_t p_duplex_pin_inv;
    uint8_t inverted;

private:
    uint8_t usart_irq_rx;
    uint8_t usart_irq_tx;
    uint8_t p_use_dma;
    uint8_t dma_irq_tx;
    uint8_t dma_irq_rx;
    uint8_t half_duplex;

    // for DMA TX
    struct tx_pool_s tx_pool[UART_DMA_TX_BUFF];
    struct tx_pool_s * tx_pool_head;
    struct tx_pool_s * tx_pool_tail;

    struct tx_pool_s * tx_pool_head_get(void) {
        //return (struct tx_pool_s *)read_u32(&tx_pool_head);
        return tx_pool_head;
    }
    void tx_pool_head_set(struct tx_pool_s * head) {
        //write_u32(&tx_pool_head, (uint32_t)head);
        tx_pool_head = head;
    }
    struct tx_pool_s * tx_pool_tail_get(void) {
        //return (struct tx_pool_s *)read_u32(&tx_pool_tail);
        return tx_pool_tail;
    }
    void tx_pool_tail_set(struct tx_pool_s * tail) {
        //write_u32(&tx_pool_tail, (uint32_t)tail);
        tx_pool_tail = tail;
    }

    void tx_pool_add(const uint8_t * data, uint32_t len)
    {
        irqstatus_t flag = irq_save();
        struct tx_pool_s * next = tx_pool_head_get();
        if (!next->data_ptr) {
            next->data_ptr = (uint8_t*)data;
            next->len = len;
            tx_pool_head_set(next->next);
        }
        irq_restore(flag);
    }
};

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
