#ifndef _STM32_DEF_H_
#define _STM32_DEF_H_

#include "hal_inc.h"

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
  #define WEAK __attribute__ ((weak))
#endif

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
void _Error_Handler(const char *, int);

void hw_init(void);
void system_clock_config(void);
void ms_timer_init(void);

void USART_IDLE_IRQ_handler(uint32_t index);
void USARTx_DMA_handler(uint32_t index);
void GPIO_EXTI_IRQHandler(uint16_t pin);

#endif //_STM32_DEF_H_
