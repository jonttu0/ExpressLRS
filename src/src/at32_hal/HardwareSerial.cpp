//
// STM32 serial
//

#include "HardwareSerial.h"
#include "internal.h"
#include "irq.h"
#include "debug_elrs.h"
#include "targets.h"
#include "priorities.h"
#include <string.h>

#ifndef StatReg
#define StatReg         ISR
#endif
#ifndef RxDataReg
#define RxDataReg       RDR
#endif
#ifndef TxDataReg
#define TxDataReg       TDR
#endif
#ifndef USART_SR_IDLE
#define USART_SR_IDLE   USART_ISR_IDLE
#endif
#ifndef USART_SR_RXNE
#define USART_SR_RXNE   USART_ISR_RXNE
#endif
#ifndef USART_SR_ORE
#define USART_SR_ORE    USART_ISR_ORE
#endif
#ifndef USART_SR_TXE
#define USART_SR_TXE    USART_ISR_TXE
#endif
#ifndef USART_SR_TC
#define USART_SR_TC     USART_ISR_TC
#endif

#define UART_ENABLE_DMA_RX 0 // Don't enable yet
#define UART_ENABLE_DMA_TX 1


#ifdef DEBUG_SERIAL
// dummy putchar
#ifndef PRINTF_NUM_BLOCKS
#define PRINTF_NUM_BLOCKS   8
#endif
#ifndef PRINTF_BUFF_SIZE
#define PRINTF_BUFF_SIZE    64
#endif
char   printf_out[PRINTF_NUM_BLOCKS][PRINTF_BUFF_SIZE];
size_t printf_idx, printf_buff;

void FAST_CODE_2 Printf::_putchar(char const character)
{
    printf_out[printf_buff][printf_idx++] = character;

    /* Send buff out if line end or buffer is full */
    if ((character == '\n') || (PRINTF_BUFF_SIZE <= printf_idx)) {
        DEBUG_SERIAL.write((uint8_t*)printf_out[printf_buff], printf_idx);
        printf_buff = (printf_buff+1) % PRINTF_NUM_BLOCKS;
        printf_idx = 0;
    }
}
#endif //DEBUG_SERIAL

constexpr uint8_t serial_cnt = 0
#ifdef USART1
    + 1
#endif
#ifdef USART2
    + 1
#endif
#ifdef USART3
    + 1
#endif
#ifdef UART4
    + 1
#endif
#ifdef UART5
    + 1
#endif
#ifdef USART6
    + 1
#endif
#ifdef UART7
//    + 1
#endif
#ifdef UART8
//    + 1
#endif
    ;
HardwareSerial * _started_serials[serial_cnt];

enum {
    USE_DMA_NONE    = 0,
    USE_DMA_RX      = UART_ENABLE_DMA_RX << 1,
    USE_DMA_TX      = UART_ENABLE_DMA_TX << 2,
};

static FORCED_INLINE uint32_t
dma_ifcr_mask_get(uint32_t mask, uint8_t const dma_ch)
{
    mask <<= (4 * dma_ch);
    return mask;
}

bool FAST_CODE_2 DMA_transmit(HardwareSerial * serial, dma_channel_type * dma_ch, uint8_t const dma_ch_idx)
{
    dma_type * dma = serial->dma_unit_tx;

    /* Check if something to send */
    uint32_t len = 0, data = (uint32_t)serial->tx_pool_get(&len);
    if (data && len) {
        /* Clear all irq flags */
        dma->clr = dma_ifcr_mask_get(0xF, dma_ch_idx);
        /* Set source address */
        dma_ch->dtcnt = len;
        dma_ch->maddr = data;
        /* enable tx */
        serial->hw_enable_transmitter();
        /* Start transfer */
        dma_channel_enable(dma_ch, TRUE);
        dma_interrupt_enable(dma_ch, DMA_FDT_INT, TRUE);
        return true;
    }
    return false;
}

#if UART_USE_TX_POOL_ONLY
int8_t FAST_CODE_2 UART_transmit(HardwareSerial * serial)
{
    //irqstatus_t irqs = irq_save();
    uint32_t len = 0;
    uint8_t* data = serial->tx_pool_get(&len);
    if (data && len) {
        serial->tx_buffer_ptr = data;
        serial->tx_buffer_len = len;
        return 0;
    }
    //irq_restore(irqs);
    return -1;
}
#endif

void FAST_CODE_2 uart_irq_handler(uint32_t const index)
{
    if (serial_cnt <= index)
        return;
    HardwareSerial *serial = _started_serials[index];
    if (!serial)
        return;
    usart_type * uart;

    if (serial->usart_rx_idx == index) {
        uart = serial->p_usart_rx;
        uint32_t const status = uart->sts;

        /* Check for IDLE line interrupt */
        if (serial->dma_ch_rx) {
            if (status & USART_IDLEF_FLAG) {
                (void)uart->dt;
                uint8_t head_pos = sizeof(serial->rx_buffer) -
                    dma_data_number_get((dma_channel_type *)serial->dma_ch_rx);
                /*
                uint8_t head = serial->rx_head;
                uint8_t tail = head + (uint8_t)(head_pos - head);
                if (tail >= serial->rx_tail)
                    serial->rx_tail = tail;
                */
                serial->rx_head = head_pos;
            }
        }
        /* Check for RX data */
        else if (status & (USART_RDBF_FLAG | USART_ROERR_FLAG)) {
            uint8_t next = serial->rx_head;
            uint8_t const data = uart->dt;
            if ((next + 1) != serial->rx_tail) {
                serial->rx_buffer[next] = data;
                serial->rx_head = next + 1;
            }
        }
    }

    if (serial->usart_tx_idx == index) {
        uart = serial->p_usart_tx;
        // Check the TX DMA is not used and TX Empty IRQ was triggered
        if ((uart->sts & USART_TDBE_FLAG) && !serial->dma_ch_tx) {
            //  Check if data available
#if UART_USE_TX_POOL_ONLY
            if (serial->tx_buffer_len) {
                uart->TxDataReg = *serial->tx_buffer_ptr++;
                serial->tx_buffer_len--;
            } else if (UART_transmit(serial) < 0) {
                serial->hw_enable_receiver();
            }
#else
            if (serial->tx_head <= serial->tx_tail) {
                usart_interrupt_enable(uart, USART_TDBE_INT, FALSE);
                serial->hw_enable_receiver();
            } else {
                uart->dt = serial->tx_buffer[serial->tx_tail++];
            }
#endif
        }
    }
}

void FAST_CODE_2 uart_dma_irq_handler(uint32_t const index)
{
    if (serial_cnt <= index)
        return;
    HardwareSerial *serial = _started_serials[index];
    if (!serial || !serial->dma_ch_tx)
        return;
    dma_channel_type * channel = serial->dma_ch_tx;
    dma_type * dma = serial->dma_unit_tx;
    uint32_t const mask = dma_ifcr_mask_get(DMA1_FDT1_FLAG, serial->dma_ch_idx_tx);
    if (dma->sts & mask) {
        dma_channel_enable(channel, FALSE);
        dma_interrupt_enable(channel, DMA_FDT_INT, FALSE);
        if (!DMA_transmit(serial, channel, serial->dma_ch_idx_tx)) {
            serial->hw_enable_receiver();
        }
        dma->clr = mask;
    }
}

HardwareSerial::HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma)
{
    rx_pin = rx;
    tx_pin = tx;
    p_duplex_pin = (struct gpio_out){.regs = NULL, .bit = 0};
    p_usart_tx = p_usart_rx = NULL;
    usart_irq_rx = usart_irq_tx = 0xff;
    p_use_dma = dma ? (USE_DMA_RX | USE_DMA_TX) : USE_DMA_NONE;
    usart_tx_idx = usart_rx_idx = 0xff;
    inverted = 0;
}

void HardwareSerial::setTx(uint32_t pin)
{
    tx_pin = pin;
}

void HardwareSerial::setRx(uint32_t pin)
{
    rx_pin = pin;
}

static void configure_uart_peripheral(usart_type * uart, uint32_t baud, uint8_t half_duplex)
{
    enable_pclock((uint32_t)uart);
    usart_reset(uart);
    //uint32_t const pclk = get_pclock_frequency((uint32_t)uart);

    usart_init(uart, baud, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(uart, USART_PARITY_NONE);
    if (half_duplex)
        usart_single_line_halfduplex_select(uart, TRUE);
    usart_transmitter_enable(uart, TRUE);
    usart_receiver_enable(uart, TRUE);
    usart_enable(uart, TRUE);
}

void HardwareSerial::begin(unsigned long baud, uint8_t mode)
{
    (void)mode;
    usart_type * uart = NULL;
    dma_type * dmaptr;

    half_duplex = ((rx_pin == tx_pin) || (rx_pin == (uint32_t)UNDEF_PIN));

    p_usart_rx = (usart_type*)uart_peripheral_get(rx_pin);
    p_usart_tx = (usart_type*)uart_peripheral_get(tx_pin);

    if (half_duplex)
        p_usart_rx = p_usart_tx;

    if (p_usart_tx == USART1) {
        usart_irq_tx = USART1_IRQn;
        write_u8(&usart_tx_idx, 0);
        write_u32(&_started_serials[0], (uint32_t)this);
#ifdef USART2
    } else if (p_usart_tx == USART2) {
        usart_irq_tx = USART2_IRQn;
        write_u8(&usart_tx_idx, 1);
        write_u32(&_started_serials[1], (uint32_t)this);
#endif // USART2
#ifdef USART3
    } else if (p_usart_tx == USART3) {
        usart_irq_tx = USART3_IRQn;
        write_u8(&usart_tx_idx, 2);
        write_u32(&_started_serials[2], (uint32_t)this);
#endif // USART3
    } else {
        // Invalid HW UART config!
        return;
    }

    if (p_usart_rx != p_usart_tx) {
        if (p_usart_rx == USART1) {
            usart_irq_rx = USART1_IRQn;
            write_u8(&usart_rx_idx, 0);
            write_u32(&_started_serials[0], (uint32_t)this);
#ifdef USART2
        } else if (p_usart_rx == USART2) {
            usart_irq_rx = USART2_IRQn;
            write_u8(&usart_rx_idx, 1);
            write_u32(&_started_serials[1], (uint32_t)this);
#endif // USART2
#ifdef USART3
        } else if (p_usart_rx == USART3) {
            usart_irq_rx = USART3_IRQn;
            write_u8(&usart_rx_idx, 2);
            write_u32(&_started_serials[2], (uint32_t)this);
#endif // USART3
        } else {
            // Invalid HW UART config!
            return;
        }
    } else {
        write_u8(&usart_rx_idx, usart_tx_idx);
    }

    dma_unit_rx = dma_unit_tx = NULL;
    dma_ch_rx = dma_ch_tx = NULL;

    /* Init RX buffer */
    rx_head = rx_tail = 0;
#if UART_USE_TX_POOL_ONLY
    tx_buffer_ptr = NULL;
    tx_buffer_len = 0;
#else
    tx_head = tx_tail = 0;
#endif

    /* Init TX list */
    tx_pool_tail = tx_pool_head = tx_pool;
    uint8_t iter;
    for (iter = 0; iter < (sizeof(tx_pool)/sizeof(tx_pool[0]) - 1); iter++) {
        tx_pool[iter].next = &tx_pool[iter+1];
    }
    tx_pool[iter].next = &tx_pool[0];

    if ((p_use_dma & USE_DMA_RX) && p_usart_rx) {
        uart = p_usart_rx;
        dmaptr = (dma_type *)dma_get((uint32_t)uart, DMA_USART_RX, 0);
        if (dmaptr) {
            /*********** USART RX DMA Init ***********/
            dma_channel_type* dma_ch = (dma_channel_type*)dma_channel_get((uint32_t)uart, DMA_USART_RX, 0);
            dma_irq_rx = dma_irq_get((uint32_t)uart, DMA_USART_RX, 0);
            /* Validate DMA params */
            if (dma_ch && dma_irq_rx != 0xFF) {
                dma_ch_rx = dma_ch;
                dma_ch_idx_rx = dma_channel_idx_get((uint32_t)uart, DMA_USART_RX, 0) - 1;
                dma_unit_rx = dmaptr;

                enable_pclock((uint32_t)dmaptr);
                dma_reset(dma_ch);

                /* RX DMA stream config */
                dma_init_type dma_cfg;
                dma_cfg.peripheral_base_addr = (uint32_t)&uart->dt;
                dma_cfg.memory_base_addr = (uint32_t)rx_buffer;
                dma_cfg.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
                dma_cfg.buffer_size = sizeof(rx_buffer);
                dma_cfg.peripheral_inc_enable = FALSE;
                dma_cfg.memory_inc_enable = TRUE;
                dma_cfg.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
                dma_cfg.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
                dma_cfg.loop_mode_enable = TRUE;
                dma_cfg.priority = DMA_PRIORITY_LOW;
                dma_init(dma_ch, &dma_cfg);

                /* Set interrupt configuration */
                NVIC_SetPriority((IRQn_Type)dma_irq_rx,
                    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART_DMA, 0));
                NVIC_EnableIRQ((IRQn_Type)dma_irq_rx);
                /* Enable DMA */
                dma_channel_enable(dma_ch, TRUE);

                dma_request_config((uint32_t)uart, DMA_USART_RX, 0);
            }
        }
    }

    if ((p_use_dma & USE_DMA_TX) && p_usart_tx) {
        uart = p_usart_tx;
        dmaptr = (dma_type *)dma_get((uint32_t)uart, DMA_USART_TX, 0);
        if (dmaptr) {
            /*********** USART TX DMA Init ***********/
            dma_channel_type* dma_ch = (dma_channel_type*)dma_channel_get((uint32_t)uart, DMA_USART_TX, 0);
            dma_irq_tx = dma_irq_get((uint32_t)uart, DMA_USART_TX, 0);
            /* Validate DMA params */
            if (dma_ch && dma_irq_tx != 0xFF) {
                dma_ch_tx = dma_ch;
                dma_ch_idx_tx = dma_channel_idx_get((uint32_t)uart, DMA_USART_TX, 0) - 1;
                dma_unit_tx = dmaptr;

                enable_pclock((uint32_t)dmaptr);
                dma_reset(dma_ch);

                /* TX DMA stream config */
                dma_init_type dma_cfg;
                dma_cfg.peripheral_base_addr = (uint32_t)&uart->dt;
                dma_cfg.memory_base_addr = (uint32_t)rx_buffer;
                dma_cfg.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
                dma_cfg.buffer_size = sizeof(rx_buffer);
                dma_cfg.peripheral_inc_enable = FALSE;
                dma_cfg.memory_inc_enable = TRUE;
                dma_cfg.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
                dma_cfg.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
                dma_cfg.loop_mode_enable = TRUE;
                dma_cfg.priority = DMA_PRIORITY_LOW;
                dma_init(dma_ch, &dma_cfg);

                /* Set interrupts */
                dma_interrupt_enable(dma_ch, DMA_HDT_INT, FALSE);
                //dma_interrupt_enable(dma_ch, DMA_FDT_INT, TRUE);

                /* Set interrupt configuration */
                NVIC_SetPriority((IRQn_Type)dma_irq_tx,
                    NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART_DMA, 0));
                NVIC_EnableIRQ((IRQn_Type)dma_irq_tx);

                dma_request_config((uint32_t)uart, DMA_USART_TX, 0);
            }
        }
    }

    /* Configure UART pins to alternative mode */
    uart_config_afio((uint32_t)p_usart_tx, rx_pin, tx_pin);

    /*********** USART Init ***********/
    configure_uart_peripheral(p_usart_tx, baud, half_duplex);
    if (p_usart_rx != p_usart_tx) {
        usart_receiver_enable(p_usart_tx, FALSE);

        configure_uart_peripheral(p_usart_rx, baud, uart_pin_is_tx(rx_pin));
        usart_transmitter_enable(p_usart_rx, FALSE);
    }

    /* Enable IDLE ISR when DMA is used */
    usart_interrupt_enable(p_usart_rx, USART_IDLE_INT, (dma_unit_rx) ? TRUE : FALSE);
    usart_interrupt_enable(p_usart_rx, USART_RDBF_INT, (dma_unit_rx) ? FALSE : TRUE);
    usart_interrupt_enable(p_usart_tx, USART_TDBE_INT, FALSE);

    usart_dma_receiver_enable(p_usart_rx, (dma_unit_rx) ? TRUE : FALSE);
    usart_dma_transmitter_enable(p_usart_tx, (dma_unit_tx) ? TRUE : FALSE);

    /* Configure to receiver mode by default */
    hw_enable_receiver();
    /* Enable UART IRQ */
    if (usart_irq_tx < 0xff) {
        NVIC_SetPriority((IRQn_Type)usart_irq_tx,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART, 0));
        NVIC_EnableIRQ((IRQn_Type)usart_irq_tx);
    }
    if (usart_irq_rx < 0xff) {
        NVIC_SetPriority((IRQn_Type)usart_irq_rx,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART, 0));
        NVIC_EnableIRQ((IRQn_Type)usart_irq_rx);
    }
}

void HardwareSerial::Pause(void)
{
    /* Stop RX UART */
    usart_enable(p_usart_rx, FALSE);
}

void HardwareSerial::Continue(void)
{
    /* Enable RX UART */
    usart_enable(p_usart_rx, FALSE);
}

void HardwareSerial::end(void)
{
    if (dma_ch_rx) {
        dma_channel_enable(dma_ch_rx, FALSE);
        NVIC_DisableIRQ((IRQn_Type)dma_irq_rx);
    }
    if (dma_ch_tx) {
        dma_channel_enable(dma_ch_tx, FALSE);
        NVIC_DisableIRQ((IRQn_Type)dma_irq_tx);
    }
    if (p_usart_rx != p_usart_tx) {
        usart_enable(p_usart_rx, FALSE);
        if (usart_irq_rx < 0xff)
            NVIC_DisableIRQ((IRQn_Type)usart_irq_rx);
    }
    if (p_usart_tx) {
        usart_enable(p_usart_tx, FALSE);
        if (usart_irq_tx < 0xff)
            NVIC_DisableIRQ((IRQn_Type)usart_irq_tx);
    }
}

int HardwareSerial::available(void)
{
    //irqstatus_t flag = irq_save();
    uint8_t head = read_u8(&rx_head), tail = read_u8(&rx_tail);
    //irq_restore(flag);
    return (uint8_t)(head - tail);
}

int HardwareSerial::read(void)
{
    if (!available())
        return -1;
    //irqstatus_t flag = irq_save();
    uint8_t tail = read_u8(&rx_tail);
    write_u8(&rx_tail, tail+1);
    //irq_restore(flag);
    return rx_buffer[tail++];
}

void HardwareSerial::flush(void)
{
    // Wait until data is sent
    //while(read_u8(&tx_head) != read_u8(&tx_tail))
    //    ;
}

uint32_t HardwareSerial::write(const uint8_t *buff, uint32_t len)
{
#if UART_USE_TX_POOL_ONLY
    tx_pool_add(buff, len);
#endif
    if (dma_ch_tx) {
#if !UART_USE_TX_POOL_ONLY
        tx_pool_add(buff, len);
#endif
        if (!((dma_ch_tx)->ctrl_bit.chen))
            DMA_transmit(this, dma_ch_tx, dma_ch_idx_tx);
    } else {
#if UART_USE_TX_POOL_ONLY
        if (!read_u32(&tx_buffer_len)) {
            UART_transmit(this);
            hw_enable_transmitter();
        }
#else
        //irqstatus_t flag = irq_save();
        // push data into tx_buffer...
        uint8_t tmax = read_u8(&tx_head), tpos = read_u8(&tx_tail);
        if (tpos >= tmax) {
            tpos = tmax = 0;
            write_u8(&tx_head, 0);
            write_u8(&tx_tail, 0);
        }
        if ((tmax + len) > sizeof(tx_buffer)) {
            if ((tmax + len - tpos) > sizeof(tx_buffer))
                // Not enough space for message
                return 0;
            // Disable TX irq and move buffer
            write_u8(&tx_head, 0); // this stops TX irqs

            tpos = read_u8(&tx_tail);
            tmax -= tpos;
            memmove(&tx_buffer[0], &tx_buffer[tpos], tmax);
            write_u8(&tx_tail, 0);
            write_u8(&tx_head, tmax);
        }

        memcpy(&tx_buffer[tmax], buff, len);
        write_u8(&tx_head, (tmax + len));
        //irq_restore(flag);
        hw_enable_transmitter();
        usart_interrupt_enable(p_usart_tx, USART_TDBE_INT, TRUE);
#endif
    }
    return len;
}

void FAST_CODE_2 HardwareSerial::hw_enable_receiver(void)
{
    uint8_t const duplex = gpio_out_valid(p_duplex_pin);
    if (!half_duplex && !duplex)
        // Full duplex --> ignore config
        return;
    usart_type * const uart_tx = p_usart_tx;
    // Wait until transfer is completed
    while (usart_flag_get(uart_tx, USART_TDC_FLAG) == RESET);
    usart_transmitter_enable(uart_tx, FALSE);
    if (duplex) {
        gpio_out_write(p_duplex_pin, 0 ^ p_duplex_pin_inv);
    }
    usart_receiver_enable(p_usart_rx, TRUE);
}

void FAST_CODE_2 HardwareSerial::hw_enable_transmitter(void)
{
    uint8_t const duplex = gpio_out_valid(p_duplex_pin);
    if (!half_duplex && !duplex)
        // Full duplex --> ignore config
        return;
    usart_receiver_enable(p_usart_rx, FALSE);
    usart_transmitter_enable(p_usart_tx, TRUE);
    if (duplex) {
        gpio_out_write(p_duplex_pin, 1 ^ p_duplex_pin_inv);
    }
}

#if defined(DEFINE_SERIAL1)
#ifndef SERIAL1_USE_DMA
#define SERIAL1_USE_DMA 0
#endif
HardwareSerial Serial1(GPIO('A', 10), GPIO('A', 9), SERIAL1_USE_DMA);
#endif // DEFINE_SERIAL1

#if defined(DEFINE_SERIAL2)
#ifndef SERIAL2_USE_DMA
#define SERIAL2_USE_DMA 0
#endif
HardwareSerial Serial2(GPIO('A', 3), GPIO('A', 2), SERIAL2_USE_DMA);
#endif // DEFINE_SERIAL2

#if defined(DEFINE_SERIAL3)
#ifndef SERIAL3_USE_DMA
#define SERIAL3_USE_DMA 0
#endif
HardwareSerial Serial3(GPIO('B', 11), GPIO('B', 10), SERIAL3_USE_DMA);
#endif // DEFINE_SERIAL2
