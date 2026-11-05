/**
 * @file terrain_hint.c
 * @brief Movement hints for the terrain module, from the motion
 *        subsystem's odometry (TEAM.md IF-05).
 *
 * Gives only the distance driven, for the hump height. Speed and turn
 * stay unknown, so tilt is never held; they come from the same odometry
 * later. The distance is unknown until tMotion has published, when a read
 * fails, and once the odometry is stale (TERRAIN_HINT_STALE_MS).
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_hint.h"

/* Kernel headers (NULL, size_t) through motion.h: not <stddef.h>, whose
 * size_t clashes with the kernel's. */
#include "../motion/motion.h"
#include "terrain_config.h"
#include "terrain_platform.h"
#include "terrain_process.h"

/* Private types, constants, and macros. */

/*
 * Packed hint state, one word (resting value 0):
 *   last update count, low byte 8 | time it last changed 16 (10 ms) |
 *   stale 1 | spare 7.
 * The low byte is enough: it is compared every 10 ms and tMotion adds 1
 * every 20 ms, so it cannot come round to the same value unseen.
 */
#define LAST_COUNT_SHIFT 0u
#define LAST_COUNT_BITS 8u
#define CHANGED_SHIFT 8u
#define CHANGED_BITS 16u
#define STALE_SHIFT 24u
#define STALE_BITS 1u

#define COUNT_LOW_MASK 0xFFu
#define TIME_MASK 0xFFFFu
#define US_PER_10MS 10000u
#define MS_PER_10MS 10u
#define STALE_10MS (TERRAIN_HINT_STALE_MS / MS_PER_10MS)

/* Static data. */

/* Written only by terrain_hint_read(), from the terrain task. */
static uint32_t hint_state;

/* Private function declarations. */
static bool odometry_is_fresh (uint32_t updates);

/* Public function definitions. */

void
terrain_hint_read (terrain_hint_t * p_out)
{
    if (p_out == NULL)
    {
        return;
    }

    terrain_hint_t hint = {false, false, false, false, false, false, 0};

    motion_shared_odometry_t odometry;
    if (motion_get_odometry(&odometry) && (odometry.updates != 0u) &&
        odometry_is_fresh(odometry.updates))
    {
        hint.b_distance_known = true;
        hint.distance_mm = odometry.distance_mm;
    }

    *p_out = hint;
}

/* Private function definitions. */

/*
 * Note a change in tMotion's update count; false once it has stood
 * still for TERRAIN_HINT_STALE_MS. The stale bit stays set until the
 * count changes, so the 16-bit time coming round (655 s) cannot make a
 * stopped motion task look fresh again.
 */
static bool
odometry_is_fresh (uint32_t updates)
{
    uint32_t const now_10ms =
        (uint32_t)(terrain_platform_time_us() / US_PER_10MS) & TIME_MASK;
    uint32_t const count = updates & COUNT_LOW_MASK;

    if (count != terrain_process_field_get(hint_state, LAST_COUNT_SHIFT,
                                           LAST_COUNT_BITS))
    {
        hint_state = terrain_process_field_put(hint_state, LAST_COUNT_SHIFT,
                                               LAST_COUNT_BITS, count);
        hint_state = terrain_process_field_put(hint_state, CHANGED_SHIFT,
                                               CHANGED_BITS, now_10ms);
        hint_state =
            terrain_process_field_put(hint_state, STALE_SHIFT, STALE_BITS, 0u);
        return true;
    }

    if (terrain_process_field_get(hint_state, STALE_SHIFT, STALE_BITS) != 0u)
    {
        return false;
    }

    uint32_t const changed_10ms =
        terrain_process_field_get(hint_state, CHANGED_SHIFT, CHANGED_BITS);
    if (((now_10ms - changed_10ms) & TIME_MASK) >= STALE_10MS)
    {
        hint_state =
            terrain_process_field_put(hint_state, STALE_SHIFT, STALE_BITS, 1u);
        return false;
    }
    return true;
}
