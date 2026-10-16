/**
 * @file terrain_platform.c
 * @brief IMU clock, lock and text output through the Pico SDK.
 *
 * Implements terrain_platform.h for the Pico SDK (CMake) build.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_platform.h"
#include <stdio.h>
#include "hardware/sync.h"
#include "pico/time.h"

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */

/* Public function definitions. */

uint64_t
terrain_platform_time_us (void)
{
    return time_us_64();
}

uint32_t
terrain_platform_lock (void)
{
    return save_and_disable_interrupts();
}

void
terrain_platform_unlock (uint32_t saved)
{
    restore_interrupts(saved);
}

void
terrain_platform_print (char const * p_text)
{
    if (p_text != NULL)
    {
        (void)fputs(p_text, stdout);
    }
}

/* Private function definitions. */
