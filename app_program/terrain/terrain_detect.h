/**
 * @file terrain_detect.h
 * @brief Terrain detectors: the hump state machine.
 *
 * Pure C with no kernel calls, so it can be tested on a PC. Each step
 * takes the detector word and one sample's roll and returns the event it
 * found, if any. Waits are counted in samples, so no clock is needed.
 *
 * The course's humps are ramps under one drive wheel: the car leans
 * sideways while that wheel is up, and its nose rises only half as much.
 * So the machine follows the lean, the size of the roll: either wheel
 * counts the same, and the lean never goes negative, so coming down is a
 * drop from the highest lean of the hump (its peak), not a dip.
 *
 * Detector word, masks and shifts, bit 0 first; all zero is the resting
 * state (LEVEL, counters 0, no peak):
 *   hump state 2 | climb counter 4 | other counter 4 | peak lean 8 |
 *   spare 14
 * The peak lean is in tenths of a degree, 0 to 25.5 degrees (stops
 * there): about a 46 mm lift of one wheel on the car's track, far above
 * any ramp it can climb. The spare bits are kept as they are.
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

/** @brief Limits of the hump state machine, on the lean (size of roll). */
typedef struct
{
    float up_deg;          /**< Lean above this is climbing. */
    float drop_deg;        /**< Lean this far below the hump's peak is
                                coming down. */
    float level_deg;       /**< Lean below this is flat. */
    uint8_t enter_samples; /**< Samples in a row to start a climb or
                                descent, 1 to TERRAIN_DETECT_COUNT_MAX. */
    uint8_t top_samples;   /**< Samples in a row with no new peak to reach
                                the top, 1 to TERRAIN_DETECT_COUNT_MAX. */
    uint8_t level_samples; /**< Flat samples in a row to end a descent, 1 to
                                TERRAIN_DETECT_COUNT_MAX. */
} terrain_hump_limits_t;

/* Public function declarations. */

/**
 * @brief Run the hump state machine on one sample.
 * @param p_word Detector word; start from 0.
 * @param roll_deg Roll of this sample, degrees; either sign. Only its
 *        size, the lean, is used.
 * @param p_limits Limits; sample counts of 0 count as 1, and counts over
 *        TERRAIN_DETECT_COUNT_MAX can never be reached.
 * @return The event the sample caused: ASCENDING, DESCENDING, LEVEL or
 *         NONE. NONE, with nothing changed, if a pointer is NULL.
 *
 * LEVEL -> ASCENDING (event) after enter_samples above up_deg; the peak
 * starts there.
 * ASCENDING -> TOP (no event) after top_samples with no new peak.
 * TOP -> DESCENDING (event) after enter_samples at least drop_deg below
 * the peak.
 * DESCENDING -> LEVEL (event) after level_samples below level_deg; the
 * peak goes back to 0.
 * While ASCENDING or TOP, a higher lean raises the peak (in TOP it also
 * restarts the descent count). A counter restarts when its condition
 * breaks; both restart on a change of state.
 */
terrain_event_type_t
terrain_detect_hump_step (uint32_t * p_word, float roll_deg,
                          terrain_hump_limits_t const * p_limits);

/**
 * @brief Hump state in a detector word.
 * @param word Detector word.
 * @return The state.
 */
terrain_hump_state_t terrain_detect_hump_state (uint32_t word);

#endif /* TERRAIN_DETECT_H */
