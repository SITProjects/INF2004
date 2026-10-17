/**
 * @file terrain_platform.c
 * @brief IMU clock, lock and text output on micro T-Kernel.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_platform.h"
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>

/* Private types, constants, and macros. */
#define BITS_PER_WORD 32u

/* Static data. */
#if TK_SUPPORT_SMP
static T_SPLOCK terrain_lock;
#endif

/* Private function declarations. */

/* Public function definitions. */

void
terrain_platform_init (void)
{
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&terrain_lock);
#endif
}

uint64_t
terrain_platform_time_us (void)
{
    /* Re-read if the high word changed while the low word was read. */
    uint32_t high;
    uint32_t low;
    do
    {
        high = (uint32_t)in_w(TIMER_TIMERAWH);
        low = (uint32_t)in_w(TIMER_TIMERAWL);
    } while (high != (uint32_t)in_w(TIMER_TIMERAWH));

    return ((uint64_t)high << BITS_PER_WORD) | low;
}

uint32_t
terrain_platform_lock (void)
{
    UINT intsts;
#if TK_SUPPORT_SMP
    (void)ISpinLock(&terrain_lock, &intsts);
#else
    DI(intsts);
#endif
    return (uint32_t)intsts;
}

void
terrain_platform_unlock (uint32_t saved)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&terrain_lock, (UINT)saved);
#else
    EI((UINT)saved);
#endif
}

void
terrain_platform_print (char const * p_text)
{
    if (p_text != NULL)
    {
        tm_printf((UB *)"%s", p_text);
    }
}

/* Private function definitions. */
