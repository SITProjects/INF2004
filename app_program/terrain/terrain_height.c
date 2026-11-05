/**
 * @file terrain_height.c
 * @brief Hump height from the lean (size of roll) and the track width.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_height.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Height word: height so far 8 | smoothing primed 1. */
#define HEIGHT_SHIFT 0u
#define HEIGHT_BITS 8u
#define PRIMED_SHIFT 8u
#define FLAG_BITS 1u

#define MM_PER_STEP 1.0f
#define RAD_PER_DEG 0.017453292f

_Static_assert(TERRAIN_HEIGHT_MAX_MM == ((1u << HEIGHT_BITS) - 1u),
               "height limit must match the field width");

/* Static data. */

/* Private function declarations. */

/* Public function definitions. */

void
terrain_height_reset (terrain_height_t * p_height)
{
    if (p_height == NULL)
    {
        return;
    }
    p_height->roll_deg = 0.0f;
    p_height->word = 0u;
}

void
terrain_height_clear_hump (terrain_height_t * p_height)
{
    if (p_height == NULL)
    {
        return;
    }
    p_height->word = terrain_process_field_put(p_height->word, HEIGHT_SHIFT,
                                               HEIGHT_BITS, 0u);
}

void
terrain_height_step (terrain_height_t * p_height, terrain_hump_state_t state,
                     float roll_deg, terrain_height_limits_t const * p_limits)
{
    if ((p_height == NULL) || (p_limits == NULL))
    {
        return;
    }

    /* Light smoothing; the first sample after a reset seeds it. */
    if (terrain_process_field_get(p_height->word, PRIMED_SHIFT, FLAG_BITS) ==
        0u)
    {
        p_height->roll_deg = roll_deg;
        p_height->word = terrain_process_field_put(p_height->word, PRIMED_SHIFT,
                                                   FLAG_BITS, 1u);
    }
    else
    {
        p_height->roll_deg += p_limits->alpha * (roll_deg - p_height->roll_deg);
    }
    float const lean_deg = fabsf(p_height->roll_deg);

    /* Flat ground between humps: start again, so wobbles never add up. */
    if ((state == TERRAIN_HUMP_LEVEL) && (lean_deg < p_limits->level_deg))
    {
        terrain_height_clear_hump(p_height);
        return;
    }

    float const lift_mm = p_limits->track_mm * sinf(lean_deg * RAD_PER_DEG);
    uint32_t const code = terrain_process_encode_unsigned(lift_mm, MM_PER_STEP,
                                                          HEIGHT_BITS, false);
    uint32_t const kept =
        terrain_process_field_get(p_height->word, HEIGHT_SHIFT, HEIGHT_BITS);
    if (code > kept)
    {
        p_height->word = terrain_process_field_put(p_height->word, HEIGHT_SHIFT,
                                                   HEIGHT_BITS, code);
    }
}

uint8_t
terrain_height_mm (terrain_height_t const * p_height)
{
    if (p_height == NULL)
    {
        return 0u;
    }
    return (uint8_t)terrain_process_field_get(p_height->word, HEIGHT_SHIFT,
                                              HEIGHT_BITS);
}

/* Private function definitions. */
