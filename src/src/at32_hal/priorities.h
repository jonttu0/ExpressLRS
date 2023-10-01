#ifndef _PRIORITIES_H__
#define _PRIORITIES_H__

#define ISR_PRIO_TICKS      1
#define ISR_PRIO_UART_DMA   2 // 0
#define ISR_PRIO_UART       2
#if defined(TX_MODULE)
/* TX module shall ignore reception if some weird delay has happened */
#define ISR_PRIO_TIM        4
#define ISR_PRIO_EXTI       5
#define ISR_PRIO_ADC        6
#elif defined(RX_MODULE)
/* RX shall run reception prio to timer */
#define ISR_PRIO_TIM        5
#define ISR_PRIO_EXTI       4
#endif /* RX_MODULE */

#endif /* _PRIORITIES_H__ */
