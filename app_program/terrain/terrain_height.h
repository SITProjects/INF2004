/**
 * @file terrain_height.h
 * @brief Hump height from pitch, and from pitch and driven distance.
 *
 * Pure C with no kernel calls, so it can be tested on a PC.
 *
 * Two estimates, both in mm; the larger one is kept for the hump:
 * - Tilt: wheelbase x sin(pitch). Right for humps narrower than the car
 *   (front on top, rear still on the floor); low for long gentle ramps.
 * - Distance: the running sum of sin(pitch) x distance driven, which is
 *   the average height of the ground under the car. Right for humps
 *   wider than the car; low for narrow ones. Needs the driven distance.
 * Where one is right the other reads low, so the larger is kept. The
 * tilt estimate can never read above the hump; the distance one knows
 * only the car's tilt, not the slope under each wheel, so on steep humps
 * it is within about 5 % either way.
 *
 * Both use their own, lighter smoothing of the pitch (limits alpha): the
 * tilt smoothing of terrain_process.h lasts long enough to cut the short
 * tilt peak of a hump and read it low. The hump state machine keeps that
 * steadier pitch; only the height uses the lighter one.
 *
 * Height word, masks and shifts, bit 0 first; all zero is the resting
 * state (no hump, no distance yet, smoothing empty):
 *   height so far 8 | last distance 16 | distance valid 1 |
 *   smoothing primed 1 | spare 6
 * The last distance keeps only the low 16 bits of the driven distance:
 * only differences between batches are used, and they are far below the
 * 32 m that 16 bits can tell apart.
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
#include <stdbool.h>
#include <stdint.h>
#include "terrain_monitor.h"

/* Public types, constants, and macros. */

/* Largest height the 8-bit field holds, mm; higher reads as this. */
#define TERRAIN_HEIGHT_MAX_MM 255u

/** @brief Limits of the height estimate. */
typedef struct
{
    float wheelbase_mm; /**< Drive axle to caster contact, mm. */
    float level_deg;    /**< While LEVEL, pitch within +/- this is flat
                             ground: the estimate starts again. */
    float max_step_mm;  /**< A distance step bigger than this between
                             batches is a glitch and is not used. */
    float alpha;        /**< Pitch smoothing weight, 0 to 1 (1 = none);
                             see TERRAIN_PROCESS_LOWPASS_ALPHA. */
} terrain_height_limits_t;

/**
 * @brief Height estimate of the hump in progress. Only the task that
 *        runs terrain_monitor_update() uses it.
 */
typedef struct
{
    float sum_mm;    /**< Distance estimate so far, mm. Stays a float:
                          each sample adds about 0.2 mm, which whole mm
                          would round away. */
    float pitch_deg; /**< Lightly smoothed pitch, degrees. A filter, so
                          it stays a float. */
    uint32_t word;   /**< Packed height word (layout above). */
} terrain_height_t;

/* Public function declarations. */

/**
 * @brief Forget everything, including the last distance.
 * @param p_height Estimate; ignored if NULL.
 */
void terrain_height_reset (terrain_height_t * p_height);

/**
 * @brief Start a new hump: sum and height so far back to 0. The last
 *        distance and the smoothing are kept.
 * @param p_height Estimate; ignored if NULL.
 */
void terrain_height_clear_hump (terrain_height_t * p_height);

/**
 * @brief Start again after a recalibration: a new hump and an empty
 *        smoothing (the next pitch seeds it). The last distance is kept,
 *        so the next distance step is still known.
 * @param p_height Estimate; ignored if NULL.
 */
void terrain_height_restart (terrain_height_t * p_height);

/**
 * @brief Distance driven since the last call, from one movement hint.
 * @param p_height Estimate; keeps the last distance.
 * @param b_known Whether @p distance_mm is valid.
 * @param distance_mm Distance driven since start, mm; + is forward. Only
 *        differences are used, so it may wrap round.
 * @param max_step_mm Steps bigger than this (either way) are a glitch.
 * @param p_step_mm Receives the step, mm; 0 when not known.
 * @return True if the step is known: this and the last call both had a
 *         distance, and the step is not a glitch. False if a pointer is
 *         NULL. Call once per batch, also while not calibrated, so the
 *         first calibrated batch already has a step.
 */
bool terrain_height_distance_step (terrain_height_t * p_height, bool b_known,
                                   int32_t distance_mm, float max_step_mm,
                                   float * p_step_mm);

/**
 * @brief Add one sample to the estimate.
 * @param p_height Estimate.
 * @param state Hump state before this sample's hump step.
 * @param pitch_deg This sample's unsmoothed pitch, degrees; nose up is
 *        positive. Smoothed here with the limits' alpha.
 * @param step_mm Distance driven during this sample, mm; + is forward.
 * @param b_step_known Whether @p step_mm is valid. If not, only the tilt
 *        estimate is used.
 * @param p_limits Limits.
 * Nothing happens if a pointer is NULL. While @p state is LEVEL and the
 * smoothed pitch is flat, the hump is cleared instead
 * (terrain_height_clear_hump()). Skip samples whose pitch is held: the
 * smoothing then waits too.
 */
void terrain_height_step (terrain_height_t * p_height,
                          terrain_hump_state_t state, float pitch_deg,
                          float step_mm, bool b_step_known,
                          terrain_height_limits_t const * p_limits);

/**
 * @brief Highest estimate of the hump in progress.
 * @param p_height Estimate.
 * @return mm, 0 to TERRAIN_HEIGHT_MAX_MM (stops there); 0 if NULL.
 */
uint8_t terrain_height_mm (terrain_height_t const * p_height);

#endif /* TERRAIN_HEIGHT_H */
