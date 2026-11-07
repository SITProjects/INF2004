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
 * They come from the motion subsystem's odometry (motion_get_odometry(),
 * TEAM.md IF-05). For now only the distance, the ground speed and the two
 * wheel speeds are given. The distance is not used: the hump height comes
 * from the lean alone (one wheel on a ramp). The ground speed confirms
 * collisions; the wheel speeds give the turn rate. Whether the speed is
 * changing, and whether to hold roll in turns, are unknown, so the
 * terrain module never holds tilt. Adding them changes only
 * terrain_hint.c.
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
    int32_t distance_mm;   /**< Distance driven since start, mm; + is
                                forward. May wrap round: use only
                                differences. Not used yet. */
    int16_t speed_mm_s;    /**< Ground speed, mean of both wheels, mm/s;
                                + is forward. Valid while
                                b_distance_known (same reading). Averaged
                                by motion over about 100 ms. */
    int16_t left_mm_s;     /**< Left wheel speed, mm/s; + is forward.
                                Valid while b_distance_known (same
                                reading). For the turn rate. */
    int16_t right_mm_s;    /**< Right wheel speed, as left_mm_s. */
    bool b_following;      /**< Following the line, and moving: the only
                                time to look for humps. */
} terrain_hint_t;

/* Public function declarations. */

/**
 * @brief Copy the latest movement hints.
 * @param p_out Receives the hints; ignored if NULL.
 * Never waits, so it is safe to call from terrain_monitor_update().
 * Call from the terrain task only: it keeps the stale-check state.
 */
void terrain_hint_read (terrain_hint_t * p_out);

#endif /* TERRAIN_HINT_H */
