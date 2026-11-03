/**
 * @file terrain_hint.h
 * @brief Movement hints from other subsystems, used by the terrain module.
 *
 * The one place where outside information about the car's movement comes
 * in: whether its speed is changing, whether it is turning, and how far it
 * has driven. The accelerometer alone cannot tell speeding up from tilting
 * nose up, or turning from leaning sideways, so these hints decide when
 * tilt is held.
 *
 * No subsystem provides them yet: terrain_hint_read() reports everything
 * as unknown, and the terrain module never holds tilt. Once the motion
 * subsystem publishes its speed (an interface in TEAM.md section 5), only
 * terrain_hint.c changes.
 *
 * Internal to the terrain module; other modules should not call it.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_HINT_H
#define TERRAIN_HINT_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/**
 * @brief Latest movement hints. A field is meaningful only while its
 *        _known flag is true; unknown fields are false or 0.
 */
typedef struct
{
    bool b_speed_known;    /**< b_speed_changing and b_moving are valid. */
    bool b_speed_changing; /**< Speeding up or braking; holds pitch. */
    bool b_moving;         /**< The car is moving (speed state). */
    bool b_turn_known;     /**< b_turning is valid. */
    bool b_turning;        /**< The car is turning; holds roll. */
    bool b_distance_known; /**< distance_mm is valid. */
    uint32_t distance_mm;  /**< Distance driven since start, mm (humps). */
} terrain_hint_t;

/* Public function declarations. */

/**
 * @brief Copy the latest movement hints.
 * @param p_out Receives the hints; ignored if NULL.
 * Never waits, so it is safe to call from terrain_monitor_update().
 */
void terrain_hint_read (terrain_hint_t * p_out);

#endif /* TERRAIN_HINT_H */
