/**
 * @file terrain_fault.c
 * @brief IMU fault bit, kept locally until the team fault mask exists.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_fault.h"
#include "terrain_platform.h"

/* Private types, constants, and macros. */

/* Static data. */

/*
 * Stand-in for the team's shared fault mask; only TERRAIN_FAULT_IMU is
 * used. Access only under terrain_platform_lock(). Do not turn this into a
 * team-wide mask here: that needs the team (shared file and owner).
 */
static uint32_t fault_mask = 0u;

/* Private function declarations. */

/* Public function definitions. */

void
terrain_fault_set (void)
{
    uint32_t saved = terrain_platform_lock();
    fault_mask |= TERRAIN_FAULT_IMU;
    terrain_platform_unlock(saved);
}

void
terrain_fault_clear (void)
{
    uint32_t saved = terrain_platform_lock();
    fault_mask &= ~TERRAIN_FAULT_IMU;
    terrain_platform_unlock(saved);
}

bool
terrain_fault_is_active (void)
{
    uint32_t saved = terrain_platform_lock();
    bool const b_active = ((fault_mask & TERRAIN_FAULT_IMU) != 0u);
    terrain_platform_unlock(saved);
    return b_active;
}

/* Private function definitions. */
