/**
 * @file terrain_process.h
 * @brief IMU processing: axis mapping, tilt, motion state, impacts, bumps.
 *
 * Pure C with no Pico SDK or kernel calls, so the same file can run on
 * the car or be tested on a PC. Phase 1 adds axis mapping;
 * detection algorithms follow from Phase 3.
 *
 * Internal to the terrain module; other modules should use terrain.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_PROCESS_H
#define TERRAIN_PROCESS_H

/* Public includes. */
#include <stdint.h>

/* Public types, constants, and macros. */

/* Number of axes in a 3-D vector. */
#define TERRAIN_AXES 3u

/** @brief A 3-D vector in the car's axes: X right, Y forward, Z up. */
typedef struct
{
    float x;
    float y;
    float z;
} terrain_vec3_t;

/**
 * @brief How the sensor board is mounted on the car.
 * For car axis i (0 = X, 1 = Y, 2 = Z), from[i] is the board axis it is
 * read from (0, 1 or 2) and sign[i] is 1 or -1.
 */
typedef struct
{
    uint8_t from[TERRAIN_AXES];
    int8_t sign[TERRAIN_AXES];
} terrain_axis_map_t;

/* Public function declarations. */

/**
 * @brief Convert one sensor sample from board axes and counts to car
 *        axes and physical units.
 * @param p_map Mounting; each from[] entry must be 0, 1 or 2.
 * @param board Sample in board axes: board[0] = X, [1] = Y, [2] = Z.
 * @param scale Units per count, e.g. m/s^2 per count.
 * @return The sample in car axes, multiplied by @p scale. All zero if
 *         @p p_map or @p board is NULL or a from[] entry is out of range.
 */
terrain_vec3_t terrain_process_map_axes (terrain_axis_map_t const * p_map,
                                         int16_t const board[TERRAIN_AXES],
                                         float scale);

#endif /* TERRAIN_PROCESS_H */
