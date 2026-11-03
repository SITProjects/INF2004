/**
 * @file terrain_detect.h
 * @brief Terrain detectors: the hump state machine.
 *
 * Pure C with no kernel calls, so it can be tested on a PC. Each step
 * takes the detector word and one sample's results and returns the event
 * it found, if any. Waits are counted in samples, so no clock is needed.
 *
 * Detector word, masks and shifts, bit 0 first; all zero is the resting
 * state (LEVEL, counters 0):
 *   hump state 2 | climb counter 4 | other counter 4 | spare 22
 * The spare bits are kept as they are, for later detectors.
 *
 * Internal to the terrain module; other modules should use
 * terrain_monitor.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_DETECT_H
#define TERRAIN_DETECT_H

/* Public includes. */
#include <stdint.h>
#include "terrain_monitor.h"

/* Public types, constants, and macros. */

/* Largest sample count the 4-bit hump counters hold. */
#define TERRAIN_DETECT_COUNT_MAX 15u

/** @brief Limits of the hump state machine. */
typedef struct
{
    float up_deg;          /**< Pitch above +up_deg is climbing. */
    float top_deg;         /**< While ascending, pitch below +top_deg is the
                                top. */
    float down_deg;        /**< Pitch below -down_deg is coming down. */
    float level_deg;       /**< Pitch within +/-level_deg is flat. */
    uint8_t enter_samples; /**< Samples in a row to start a climb or
                                descent, 1 to TERRAIN_DETECT_COUNT_MAX. */
    uint8_t level_samples; /**< Flat samples in a row to end a descent, 1 to
                                TERRAIN_DETECT_COUNT_MAX. */
} terrain_hump_limits_t;

/* Public function declarations. */

/**
 * @brief Run the hump state machine on one sample.
 * @param p_word Detector word; start from 0.
 * @param pitch_deg Pitch of this sample, degrees; nose up is positive.
 * @param p_limits Limits; sample counts of 0 count as 1, and counts over
 *        TERRAIN_DETECT_COUNT_MAX can never be reached.
 * @return The event the sample caused: ASCENDING, DESCENDING, LEVEL or
 *         NONE. NONE, with nothing changed, if a pointer is NULL.
 *
 * LEVEL -> ASCENDING (event) after enter_samples above +up_deg.
 * LEVEL -> DESCENDING (event) after enter_samples below -down_deg (a dip).
 * ASCENDING -> TOP (no event) as soon as pitch is below +top_deg.
 * TOP -> DESCENDING (event) after enter_samples below -down_deg.
 * TOP -> ASCENDING (no event: same hump) after enter_samples above
 * +up_deg.
 * DESCENDING -> LEVEL (event) after level_samples within +/-level_deg.
 * DESCENDING -> ASCENDING (event: next hump) after enter_samples above
 * +up_deg.
 * A counter restarts when its condition breaks; both restart on a change
 * of state.
 */
terrain_event_type_t
terrain_detect_hump_step (uint32_t * p_word, float pitch_deg,
                          terrain_hump_limits_t const * p_limits);

/**
 * @brief Hump state in a detector word.
 * @param word Detector word.
 * @return The state.
 */
terrain_hump_state_t terrain_detect_hump_state (uint32_t word);

#endif /* TERRAIN_DETECT_H */
