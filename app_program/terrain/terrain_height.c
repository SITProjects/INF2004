/**
 * @file terrain_height.c
 * @brief Hump height from pitch, and from pitch and driven distance.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_height.h"
#include <math.h>
#include <stddef.h>
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Height word: height so far 8 | last distance 16 | distance valid 1 |
 * smoothing primed 1. */
#define HEIGHT_SHIFT 0u
#define HEIGHT_BITS 8u
#define DIST_SHIFT 8u
#define DIST_BITS 16u
#define VALID_SHIFT 24u
#define PRIMED_SHIFT 25u
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
    p_height->sum_mm = 0.0f;
    p_height->pitch_deg = 0.0f;
    p_height->word = 0u;
}

void
terrain_height_clear_hump (terrain_height_t * p_height)
{
    if (p_height == NULL)
    {
        return;
    }
    p_height->sum_mm = 0.0f;
    p_height->word = terrain_process_field_put(p_height->word, HEIGHT_SHIFT,
                                               HEIGHT_BITS, 0u);
}

void
terrain_height_restart (terrain_height_t * p_height)
{
    if (p_height == NULL)
    {
        return;
    }
    terrain_height_clear_hump(p_height);
    p_height->pitch_deg = 0.0f;
    p_height->word =
        terrain_process_field_put(p_height->word, PRIMED_SHIFT, FLAG_BITS, 0u);
}

bool
terrain_height_distance_step (terrain_height_t * p_height, bool b_known,
                              int32_t distance_mm, float max_step_mm,
                              float * p_step_mm)
{
    if ((p_height == NULL) || (p_step_mm == NULL))
    {
        return false;
    }
    *p_step_mm = 0.0f;

    uint32_t word = p_height->word;
    if (!b_known)
    {
        /* The next known distance has nothing to compare with. */
        p_height->word =
            terrain_process_field_put(word, VALID_SHIFT, FLAG_BITS, 0u);
        return false;
    }

    /* Low 16 bits, worked out unsigned: conversion to uint32_t is modulo
     * 2^32, so negative distances keep their low bits. */
    uint32_t const now =
        terrain_process_field_get((uint32_t)distance_mm, 0u, DIST_BITS);
    bool const b_had =
        (terrain_process_field_get(word, VALID_SHIFT, FLAG_BITS) != 0u);
    uint32_t const before =
        terrain_process_field_get(word, DIST_SHIFT, DIST_BITS);

    word = terrain_process_field_put(word, DIST_SHIFT, DIST_BITS, now);
    word = terrain_process_field_put(word, VALID_SHIFT, FLAG_BITS, 1u);
    p_height->word = word;

    if (!b_had)
    {
        return false;
    }

    /* Difference modulo 2^16, read back as signed: correct across a wrap
     * while the car moved less than 32 m between the two calls. */
    uint32_t const diff = terrain_process_count_add_wrap(
        now, (1u << DIST_BITS) - before, DIST_BITS);
    float const step_mm = (float)terrain_process_decode_signed(diff, DIST_BITS);
    if (fabsf(step_mm) > max_step_mm)
    {
        return false;
    }

    *p_step_mm = step_mm;
    return true;
}

void
terrain_height_step (terrain_height_t * p_height, terrain_hump_state_t state,
                     float pitch_deg, float step_mm, bool b_step_known,
                     terrain_height_limits_t const * p_limits)
{
    if ((p_height == NULL) || (p_limits == NULL))
    {
        return;
    }

    /* Light smoothing; the first sample after a (re)start seeds it. */
    if (terrain_process_field_get(p_height->word, PRIMED_SHIFT, FLAG_BITS) ==
        0u)
    {
        p_height->pitch_deg = pitch_deg;
        p_height->word = terrain_process_field_put(p_height->word, PRIMED_SHIFT,
                                                   FLAG_BITS, 1u);
    }
    else
    {
        p_height->pitch_deg +=
            p_limits->alpha * (pitch_deg - p_height->pitch_deg);
    }
    float const smooth_deg = p_height->pitch_deg;

    /* Flat ground between humps: start again, so wobbles never add up. */
    if ((state == TERRAIN_HUMP_LEVEL) &&
        (fabsf(smooth_deg) <= p_limits->level_deg))
    {
        terrain_height_clear_hump(p_height);
        return;
    }

    float const rise = sinf(smooth_deg * RAD_PER_DEG);
    if (b_step_known)
    {
        p_height->sum_mm += rise * step_mm;
    }

    /* Larger of the two estimates; negative ones round to 0. */
    float best_mm = p_limits->wheelbase_mm * rise;
    if (p_height->sum_mm > best_mm)
    {
        best_mm = p_height->sum_mm;
    }

    uint32_t const code = terrain_process_encode_unsigned(best_mm, MM_PER_STEP,
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
