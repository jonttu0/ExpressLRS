// Definitions for irq enable/disable on ARM Cortex-M processors
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// https://github.com/KevinOConnor/klipper
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __GENERIC_IRQ_H
#define __GENERIC_IRQ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef unsigned long irqstatus_t;

static inline void __attribute__((always_inline)) irq_disable(void)
{
    asm volatile("cpsid i" ::
                     : "memory");
}
static inline void __attribute__((always_inline)) irq_enable(void)
{
    asm volatile("cpsie i" ::
                     : "memory");
}
static inline irqstatus_t __attribute__((always_inline)) irq_save(void)
{
    irqstatus_t flag;
    asm volatile("mrs %0, primask"
                 : "=r"(flag)::"memory");
    irq_disable();
    return flag;
}
static inline void __attribute__((always_inline)) irq_restore(irqstatus_t flag)
{
    asm volatile("msr primask, %0" ::"r"(flag)
                 : "memory");
}
static inline void __attribute__((always_inline)) irq_wait(void)
{
    asm volatile("cpsie i\n    wfi\n    cpsid i\n" ::
                     : "memory");
}

static inline void __attribute__((always_inline)) irq_poll(void)
{
}

#ifdef __cplusplus
}
#endif

#endif // irq.h
