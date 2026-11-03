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

/* Detector word: hump state 2 | climb counter 4 | other counter 4. */
#define HUMP_SHIFT 0u
#define HUMP_BITS 2u
#define CLIMB_SHIFT 2u
#define OTHER_SHIFT 6u
#define COUNT_BITS 4u

_Static_assert(TERRAIN_HUMP_DESCENDING < (1u << HUMP_BITS),
               "hump states must fit their field");
_Static_assert(TERRAIN_DETECT_COUNT_MAX == ((1u << COUNT_BITS) - 1u),
               "count limit must match the counter width");

/* Static data. */

/* Private function declarations. */
static uint32_t count_if (bool b_condition, uint32_t count);
static uint32_t samples_needed (uint8_t samples);

/* Public function definitions. */

terrain_event_type_t
terrain_detect_hump_step (uint32_t * p_word, float pitch_deg,
                          terrain_hump_limits_t const * p_limits)
{
    if ((p_word == NULL) || (p_limits == NULL))
    {
        return TERRAIN_EVENT_NONE;
    }

    uint32_t word = *p_word;
    terrain_hump_state_t const state = terrain_detect_hump_state(word);
    uint32_t const enter = samples_needed(p_limits->enter_samples);
    uint32_t const flat = samples_needed(p_limits->level_samples);

    /* The climb counter runs in every state; "other" depends on it. */
    uint32_t const climb =
        count_if(pitch_deg > p_limits->up_deg,
                 terrain_process_field_get(word, CLIMB_SHIFT, COUNT_BITS));
    uint32_t other = terrain_process_field_get(word, OTHER_SHIFT, COUNT_BITS);
    bool const b_down = pitch_deg < -p_limits->down_deg;
    bool const b_climbed = climb >= enter;

    terrain_hump_state_t next = state;
    terrain_event_type_t event = TERRAIN_EVENT_NONE;

    switch (state)
    {
        case TERRAIN_HUMP_LEVEL:
            other = count_if(b_down, other);
            if (b_climbed)
            {
                next = TERRAIN_HUMP_ASCENDING;
                event = TERRAIN_EVENT_ASCENDING;
            }
            else if (other >= enter)
            {
                next = TERRAIN_HUMP_DESCENDING;
                event = TERRAIN_EVENT_DESCENDING;
            }
            break;
        case TERRAIN_HUMP_ASCENDING:
            other = 0u;
            if (pitch_deg < p_limits->top_deg)
            {
                next = TERRAIN_HUMP_TOP;
            }
            break;
        case TERRAIN_HUMP_TOP:
            other = count_if(b_down, other);
            if (other >= enter)
            {
                next = TERRAIN_HUMP_DESCENDING;
                event = TERRAIN_EVENT_DESCENDING;
            }
            else if (b_climbed)
            {
                /* Still the same hump, climbing in steps: no event. */
                next = TERRAIN_HUMP_ASCENDING;
            }
            break;
        default:
            /* DESCENDING. */
            other = count_if(fabsf(pitch_deg) < p_limits->level_deg, other);
            if (other >= flat)
            {
                next = TERRAIN_HUMP_LEVEL;
                event = TERRAIN_EVENT_LEVEL;
            }
            else if (b_climbed)
            {
                next = TERRAIN_HUMP_ASCENDING;
                event = TERRAIN_EVENT_ASCENDING;
            }
            break;
    }

    uint32_t climb_kept = climb;
    if (next != state)
    {
        climb_kept = 0u;
        other = 0u;
    }

    word =
        terrain_process_field_put(word, HUMP_SHIFT, HUMP_BITS, (uint32_t)next);
    word = terrain_process_field_put(word, CLIMB_SHIFT, COUNT_BITS, climb_kept);
    word = terrain_process_field_put(word, OTHER_SHIFT, COUNT_BITS, other);
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

/* A wait of 0 samples would fire with no sample at all: use 1. */
static uint32_t
samples_needed (uint8_t samples)
{
    return (samples == 0u) ? 1u : (uint32_t)samples;
}
