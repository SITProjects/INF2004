/**
 * @file terrain_platform.h
 * @brief Clock, lock and text output used by the terrain (IMU) module.
 *
 * Internal to the terrain module; other modules should not call it.
 * Implemented by terrain_platform.c on micro T-Kernel.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_PLATFORM_H
#define TERRAIN_PLATFORM_H

/* Public includes. */
#include <stdint.h>

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Prepare the lock. Call once, before any task uses the terrain
 *        module (terrain_init() in terrain.c does this).
 */
void terrain_platform_init (void);

/**
 * @brief Time since boot.
 * @return Microseconds since boot, from the RP2040's 1 MHz TIMER.
 */
uint64_t terrain_platform_time_us (void);

/**
 * @brief Start a short section that nothing else may interrupt.
 * @return State to pass to terrain_platform_unlock().
 * Keep the section to a few microseconds, for example copying a struct.
 * Masks interrupts on the calling core and, on the dual-core build, also
 * holds a spinlock against the other core.
 */
uint32_t terrain_platform_lock (void);

/**
 * @brief End a section started by terrain_platform_lock().
 * @param saved Value returned by the matching terrain_platform_lock().
 */
void terrain_platform_unlock (uint32_t saved);

/**
 * @brief Write text to the console exactly as given; no newline is added.
 * @param p_text Null-terminated text.
 */
void terrain_platform_print (char const * p_text);

#endif /* TERRAIN_PLATFORM_H */
