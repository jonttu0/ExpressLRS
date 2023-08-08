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
void SystemClock_Config(void);
void ms_timer_init(void);

void uart_irq_handler(uint32_t index);
void uart_dma_irq_handler(uint32_t index);
void GPIO_EXTI_IRQHandler(uint16_t pin);

#endif //_STM32_DEF_H_
