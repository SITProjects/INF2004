/**
 * @file terrain_detect.c
 * @brief Terrain detectors: the hump state machine.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_detect.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Detector word: hump state 2 | climb counter 4 | other counter 4 |
 * peak lean 8. */
#define HUMP_SHIFT 0u
#define HUMP_BITS 2u
#define CLIMB_SHIFT 2u
#define OTHER_SHIFT 6u
#define COUNT_BITS 4u
#define PEAK_SHIFT 10u
#define PEAK_BITS 8u

/* Lean and peak in tenths of a degree. */
#define DDEG_PER_DEG 10.0f

_Static_assert(TERRAIN_HUMP_DESCENDING < (1u << HUMP_BITS),
               "hump states must fit their field");
_Static_assert(TERRAIN_DETECT_COUNT_MAX == ((1u << COUNT_BITS) - 1u),
               "count limit must match the counter width");

/* Static data. */

/* Private function declarations. */
static uint32_t count_if (bool b_condition, uint32_t count);
static uint32_t samples_needed (uint8_t samples);
static uint32_t to_ddeg (float deg);

/* Public function definitions. */

terrain_event_type_t
terrain_detect_hump_step (uint32_t * p_word, float roll_deg,
                          terrain_hump_limits_t const * p_limits)
{
    if ((p_word == NULL) || (p_limits == NULL))
    {
        return TERRAIN_EVENT_NONE;
    }

    uint32_t word = *p_word;
    terrain_hump_state_t const state = terrain_detect_hump_state(word);
    uint32_t const enter = samples_needed(p_limits->enter_samples);

    /* Compared in tenths of a degree, the unit of the peak. */
    uint32_t const lean = to_ddeg(fabsf(roll_deg));
    uint32_t peak = terrain_process_field_get(word, PEAK_SHIFT, PEAK_BITS);
    uint32_t climb = terrain_process_field_get(word, CLIMB_SHIFT, COUNT_BITS);
    uint32_t other = terrain_process_field_get(word, OTHER_SHIFT, COUNT_BITS);
    bool const b_new_peak = lean > peak;
    bool const b_dropped = (lean + to_ddeg(p_limits->drop_deg)) <= peak;

    terrain_hump_state_t next = state;
    terrain_event_type_t event = TERRAIN_EVENT_NONE;

    switch (state)
    {
        case TERRAIN_HUMP_LEVEL:
            climb = count_if(lean > to_ddeg(p_limits->up_deg), climb);
            if (climb >= enter)
            {
                next = TERRAIN_HUMP_ASCENDING;
                event = TERRAIN_EVENT_ASCENDING;
                peak = lean;
            }
            break;
        case TERRAIN_HUMP_ASCENDING:
            other = count_if(!b_new_peak, other);
            if (b_new_peak)
            {
                peak = lean;
            }
            else if (other >= samples_needed(p_limits->top_samples))
            {
                next = TERRAIN_HUMP_TOP;
            }
            else
            {
                /* Still climbing, or only just stopped. */
            }
            break;
        case TERRAIN_HUMP_TOP:
            other = count_if(b_dropped, other);
            if (b_new_peak)
            {
                /* Climbing in steps: the same hump, a higher peak. */
                peak = lean;
            }
            else if (other >= enter)
            {
                next = TERRAIN_HUMP_DESCENDING;
                event = TERRAIN_EVENT_DESCENDING;
            }
            else
            {
                /* On the top. */
            }
            break;
        default:
            /* DESCENDING. */
            other = count_if(lean < to_ddeg(p_limits->level_deg), other);
            if (other >= samples_needed(p_limits->level_samples))
            {
                next = TERRAIN_HUMP_LEVEL;
                event = TERRAIN_EVENT_LEVEL;
                peak = 0u;
            }
            break;
    }

    if (next != state)
    {
        climb = 0u;
        other = 0u;
    }

    word =
        terrain_process_field_put(word, HUMP_SHIFT, HUMP_BITS, (uint32_t)next);
    word = terrain_process_field_put(word, CLIMB_SHIFT, COUNT_BITS, climb);
    word = terrain_process_field_put(word, OTHER_SHIFT, COUNT_BITS, other);
    word = terrain_process_field_put(word, PEAK_SHIFT, PEAK_BITS, peak);
    *p_word = word;
    return event;
}

terrain_hump_state_t
terrain_detect_hump_state (uint32_t word)
{
    return (terrain_hump_state_t)terrain_process_field_get(word, HUMP_SHIFT,
                                                           HUMP_BITS);
}

/* Private function definitions. */

/* One more sample in a row while the condition holds, else start again. */
static uint32_t
count_if (bool b_condition, uint32_t count)
{
    if (!b_condition)
    {
        return 0u;
    }
    return terrain_process_count_add_sat(count, 1u, COUNT_BITS);
}

/* Degrees (0 or more) to tenths, rounded, stopping at the peak's 25.5. */
static uint32_t
to_ddeg (float deg)
{
    return terrain_process_encode_unsigned(deg, DDEG_PER_DEG, PEAK_BITS, false);
}

/* A wait of 0 samples would fire with no sample at all: use 1. */
static uint32_t
samples_needed (uint8_t samples)
{
    return (samples == 0u) ? 1u : (uint32_t)samples;
}
