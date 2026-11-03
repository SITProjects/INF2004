/**
 * @file terrain_fault.h
 * @brief IMU fault bit: the only place the terrain module raises or clears
 *        it.
 *
 * Change Requirement 1 puts each fault in one shared fault mask. The team
 * mask does not exist yet, so the bit is kept here in a local word shaped
 * like it; when the mask exists, only terrain_fault.c changes, to call the
 * team's protected set and clear function.
 *
 * Internal to the terrain module; other modules will read the team mask.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_FAULT_H
#define TERRAIN_FAULT_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/* IMU fault: the accelerometer is missing or has stopped delivering data. */
#define TERRAIN_FAULT_IMU 0x01u

/* Public function declarations. */

/**
 * @brief Raise the IMU fault.
 * Call only from the task that calls terrain_monitor_update(). Takes the
 * platform lock; terrain_platform_init() must have run.
 */
void terrain_fault_set (void);

/**
 * @brief Clear the IMU fault.
 * Call only from the task that calls terrain_monitor_update(). Takes the
 * platform lock; terrain_platform_init() must have run.
 */
void terrain_fault_clear (void);

/**
 * @brief Whether the IMU fault is raised.
 * @return True if raised.
 * Safe to call from any task; takes the platform lock.
 */
bool terrain_fault_is_active (void);

#endif /* TERRAIN_FAULT_H */
