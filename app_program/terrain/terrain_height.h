/**
 * @file terrain_height.h
 * @brief Hump height from the lean (size of roll) and the track width.
 *
 * Pure C with no kernel calls, so it can be tested on a PC.
 *
 * The course's humps are ramps under one drive wheel. Seen from behind,
 * the axle is the long side of a right-angled triangle: the wheel on the
 * floor is the hinge, the lifted wheel is h mm up, and the wheels are the
 * track width apart. So h = track width x sin(lean), kept at its highest
 * from the start of the climb. Either wheel reads the same.
 *
 * It uses its own, lighter smoothing of the roll (limits alpha): the tilt
 * smoothing of terrain_process.h lasts long enough to cut the short lean
 * of a short ramp and read it low. The hump state machine keeps that
 * steadier roll; only the height uses the lighter one.
 *
 * Height word, masks and shifts, bit 0 first; all zero is the resting
 * state (no hump, smoothing empty):
 *   height so far 8 | smoothing primed 1 | spare 23
 *
 * Internal to the terrain module; other modules should use
 * terrain_monitor.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_HEIGHT_H
#define TERRAIN_HEIGHT_H

/* Public includes. */
#include <stdint.h>
#include "terrain_monitor.h"

/* Public types, constants, and macros. */

/* Largest height the 8-bit field holds, mm; higher reads as this. */
#define TERRAIN_HEIGHT_MAX_MM 255u

/** @brief Limits of the height estimate. */
typedef struct
{
    float track_mm;  /**< Drive wheel to drive wheel, centre to centre,
                          mm. */
    float level_deg; /**< While LEVEL, a lean below this is flat ground:
                          the estimate starts again. */
    float alpha;     /**< Roll smoothing weight, 0 to 1 (1 = none); see
                          TERRAIN_PROCESS_LOWPASS_ALPHA. */
} terrain_height_limits_t;

/**
 * @brief Height estimate of the hump in progress. Only the task that
 *        runs terrain_monitor_update() uses it.
 */
typedef struct
{
    float roll_deg; /**< Lightly smoothed roll, degrees. A filter, so it
                         stays a float. */
    uint32_t word;  /**< Packed height word (layout above). */
} terrain_height_t;

/* Public function declarations. */

/**
 * @brief Forget everything: no hump, and an empty smoothing (the next
 *        roll seeds it). Use at init and after a recalibration.
 * @param p_height Estimate; ignored if NULL.
 */
void terrain_height_reset (terrain_height_t * p_height);

/**
 * @brief Start a new hump: height so far back to 0. The smoothing is
 *        kept.
 * @param p_height Estimate; ignored if NULL.
 */
void terrain_height_clear_hump (terrain_height_t * p_height);

/**
 * @brief Add one sample to the estimate.
 * @param p_height Estimate.
 * @param state Hump state before this sample's hump step.
 * @param roll_deg This sample's unsmoothed roll, degrees; either sign.
 *        Smoothed here with the limits' alpha.
 * @param p_limits Limits.
 * Nothing happens if a pointer is NULL. While @p state is LEVEL and the
 * smoothed lean is flat, the hump is cleared instead
 * (terrain_height_clear_hump()). Skip samples whose roll is held: the
 * smoothing then waits too.
 */
void terrain_height_step (terrain_height_t * p_height,
                          terrain_hump_state_t state, float roll_deg,
                          terrain_height_limits_t const * p_limits);

/**
 * @brief Highest estimate of the hump in progress.
 * @param p_height Estimate.
 * @return mm, 0 to TERRAIN_HEIGHT_MAX_MM (stops there); 0 if NULL.
 */
uint8_t terrain_height_mm (terrain_height_t const * p_height);

#endif /* TERRAIN_HEIGHT_H */
