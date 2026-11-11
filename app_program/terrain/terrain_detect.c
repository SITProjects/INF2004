/**
 * @file terrain_detect.c
 * @brief Terrain detectors: the hump, turning and speed state machines,
 *        and the lift and overturn check.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_detect.h"
#include "../robot_types.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Detector word: hump state 2 | climb counter 4 | other counter 4 |
 * peak lean 8 | turning state 1 | turn counter 5 | spare 8. */
#define HUMP_SHIFT 0u
#define HUMP_BITS 2u
#define CLIMB_SHIFT 2u
#define OTHER_SHIFT 6u
#define COUNT_BITS 4u
#define PEAK_SHIFT 10u
#define PEAK_BITS 8u
#define HUMP_FIELDS_MASK 0x0003FFFFu /* Bits 0-17: every hump field. */
#define TURN_SHIFT 18u
#define TURN_BITS 1u
#define TURN_COUNT_SHIFT 19u
#define TURN_COUNT_BITS 5u

/* Lean and peak in tenths of a degree. */
#define DDEG_PER_DEG 10.0f

/* terrain_lift_t word fields, "shift, bits": next slot 4 | samples held 4 |
 * over-tilt run 8 | cooldown 8 | overturn reported 1. */
#define LIFT_NEXT 0u, 4u
#define LIFT_HELD 4u, 4u
#define LIFT_RUN 8u, 8u
#define LIFT_COOLDOWN 16u, 8u
#define LIFT_OVER_REPORTED 0x01000000u
#define LIFT_COUNT_MAX 255u
/* Window samples in cm/s^2, 16 bits; event values 12 bits. */
#define CM_PER_M 100.0f
#define LIFT_SAMPLE_BITS 16u
#define LIFT_VALUE_BITS 12u

/* Speed word: speed state 2 | window count 4 | candidate 2 |
 * confirm count 2 | speed at the last check 8 | spare 14. */
#define SPEED_STATE_SHIFT 0u
#define SPEED_STATE_BITS 2u
#define WINDOW_SHIFT 2u
#define WINDOW_BITS 4u
#define CANDIDATE_SHIFT 6u
#define CONFIRM_SHIFT 8u
#define CONFIRM_BITS 2u
#define LAST_SPEED_SHIFT 10u
#define LAST_SPEED_BITS 8u

/* Wheel speed in 2 mm/s steps. */
#define WHEEL_STEPS_PER_MM_S 0.5f
#define WHEEL_MM_S_PER_STEP 2.0f

_Static_assert(TERRAIN_HUMP_DESCENDING < (1u << HUMP_BITS),
               "hump states must fit their field");
_Static_assert(TERRAIN_DETECT_COUNT_MAX == ((1u << COUNT_BITS) - 1u),
               "count limit must match the counter width");
_Static_assert(TERRAIN_DETECT_LIFT_WINDOW < 16u,
               "lift window must fit the 4-bit slot and count fields");
_Static_assert(HUMP_FIELDS_MASK == ((1u << TURN_SHIFT) - 1u),
               "hump fields must end where the turn fields start");
_Static_assert(TERRAIN_TURN_TURNING < (1u << TURN_BITS),
               "turning states must fit their field");
_Static_assert(TERRAIN_DETECT_TURN_COUNT_MAX == ((1u << TURN_COUNT_BITS) - 1u),
               "turn count limit must match the counter width");
_Static_assert(TERRAIN_SPEED_DECELERATING < (1u << SPEED_STATE_BITS),
               "speed states must fit their field");
_Static_assert(TERRAIN_DETECT_WINDOW_MAX == ((1u << WINDOW_BITS) - 1u),
               "window limit must match the window count width");
_Static_assert(TERRAIN_DETECT_CONFIRM_MAX == ((1u << CONFIRM_BITS) - 1u),
               "confirm limit must match the confirm count width");
_Static_assert(((uint32_t)TERRAIN_EVENT_DECELERATING -
                (uint32_t)TERRAIN_EVENT_STATIONARY) ==
                   (uint32_t)TERRAIN_SPEED_DECELERATING,
               "speed events must follow the speed states' order");

/* Static data. */

/* Private function declarations. */
static uint32_t count_if (bool b_condition, uint32_t count, uint32_t width);
static uint32_t samples_needed (uint8_t samples);
static uint32_t samples_capped (uint8_t samples, uint32_t max);
static uint32_t to_ddeg (float deg);
static int16_t to_cm (float mps2);

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
            climb =
                count_if(lean > to_ddeg(p_limits->up_deg), climb, COUNT_BITS);
            if (climb >= enter)
            {
                next = TERRAIN_HUMP_ASCENDING;
                event = TERRAIN_EVENT_ASCENDING;
                peak = lean;
            }
            break;
        case TERRAIN_HUMP_ASCENDING:
            other = count_if(!b_new_peak, other, COUNT_BITS);
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
            other = count_if(b_dropped, other, COUNT_BITS);
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
            other = count_if(lean < to_ddeg(p_limits->level_deg), other,
                             COUNT_BITS);
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

uint32_t
terrain_detect_hump_clear (uint32_t word)
{
    return word & ~HUMP_FIELDS_MASK;
}

bool
terrain_detect_hump_lasted (int32_t start_mm, int32_t end_mm, int32_t min_mm)
{
    return (int32_t)((uint32_t)end_mm - (uint32_t)start_mm) >= min_mm;
}

int32_t
terrain_detect_hump_min_mm (int32_t speed_mm_s, int32_t slow_mm_s,
                            int32_t slow_min_mm, int32_t min_mm)
{
    return (speed_mm_s <= slow_mm_s) ? slow_min_mm : min_mm;
}

terrain_hump_sample_t
terrain_detect_hump_sample (bool b_following, bool b_roll_held,
                            int32_t speed_mm_s, int32_t min_speed_mm_s)
{
    if (!b_following)
    {
        return TERRAIN_HUMP_SAMPLE_DROP;
    }
    return (b_roll_held || (speed_mm_s < min_speed_mm_s))
               ? TERRAIN_HUMP_SAMPLE_PAUSE
               : TERRAIN_HUMP_SAMPLE_USE;
}

terrain_event_type_t
terrain_detect_turn_step (uint32_t * p_word, float turn_dps,
                          terrain_turn_limits_t const * p_limits)
{
    if ((p_word == NULL) || (p_limits == NULL))
    {
        return TERRAIN_EVENT_NONE;
    }

    uint32_t word = *p_word;
    terrain_turn_state_t const state = terrain_detect_turn_state(word);
    uint32_t count =
        terrain_process_field_get(word, TURN_COUNT_SHIFT, TURN_COUNT_BITS);

    /* Left or right turns alike: only the size of the rate counts. */
    float const rate_dps = fabsf(turn_dps);

    terrain_turn_state_t next = state;
    terrain_event_type_t event = TERRAIN_EVENT_NONE;

    if (state == TERRAIN_TURN_STRAIGHT)
    {
        count = count_if(rate_dps > p_limits->on_dps, count, TURN_COUNT_BITS);
        if (count >= samples_needed(p_limits->on_samples))
        {
            next = TERRAIN_TURN_TURNING;
            event = TERRAIN_EVENT_TURNING;
        }
    }
    else
    {
        count = count_if(rate_dps < p_limits->off_dps, count, TURN_COUNT_BITS);
        if (count >= samples_needed(p_limits->off_samples))
        {
            next = TERRAIN_TURN_STRAIGHT;
            event = TERRAIN_EVENT_STRAIGHT;
        }
    }

    if (next != state)
    {
        count = 0u;
    }

    word =
        terrain_process_field_put(word, TURN_SHIFT, TURN_BITS, (uint32_t)next);
    word = terrain_process_field_put(word, TURN_COUNT_SHIFT, TURN_COUNT_BITS,
                                     count);
    *p_word = word;
    return event;
}

terrain_turn_state_t
terrain_detect_turn_state (uint32_t word)
{
    return (terrain_turn_state_t)terrain_process_field_get(word, TURN_SHIFT,
                                                           TURN_BITS);
}

terrain_event_type_t
terrain_detect_speed_step (uint32_t * p_word, float wheel_mm_s,
                           terrain_speed_limits_t const * p_limits)
{
    if ((p_word == NULL) || (p_limits == NULL))
    {
        return TERRAIN_EVENT_NONE;
    }

    uint32_t word = *p_word;
    uint32_t const counted =
        terrain_process_field_get(word, WINDOW_SHIFT, WINDOW_BITS) + 1u;
    if (counted < samples_capped(p_limits->window, TERRAIN_DETECT_WINDOW_MAX))
    {
        *p_word =
            terrain_process_field_put(word, WINDOW_SHIFT, WINDOW_BITS, counted);
        return TERRAIN_EVENT_NONE;
    }

    /* A check: the speed in 2 mm/s steps (NaN and negative give 0). */
    uint32_t const code = terrain_process_encode_unsigned(
        wheel_mm_s, WHEEL_STEPS_PER_MM_S, LAST_SPEED_BITS, false);
    uint32_t const last =
        terrain_process_field_get(word, LAST_SPEED_SHIFT, LAST_SPEED_BITS);
    float const speed_mm_s = (float)code * WHEEL_MM_S_PER_STEP;
    float const change_mm_s =
        (float)((int32_t)code - (int32_t)last) * WHEEL_MM_S_PER_STEP;

    terrain_speed_state_t answer = TERRAIN_SPEED_CRUISING;
    if (speed_mm_s < p_limits->stop_mm_s)
    {
        answer = TERRAIN_SPEED_STATIONARY;
    }
    else if (change_mm_s >= p_limits->change_mm_s)
    {
        answer = TERRAIN_SPEED_ACCELERATING;
    }
    else if (change_mm_s <= -p_limits->change_mm_s)
    {
        answer = TERRAIN_SPEED_DECELERATING;
    }
    else
    {
        /* Moving at a steady speed: CRUISING. */
    }

    terrain_speed_state_t const state = terrain_detect_speed_state(word);
    terrain_speed_state_t const held =
        (terrain_speed_state_t)terrain_process_field_get(word, CANDIDATE_SHIFT,
                                                         SPEED_STATE_BITS);
    uint32_t confirm =
        terrain_process_field_get(word, CONFIRM_SHIFT, CONFIRM_BITS);

    /* Count checks in a row giving the same new answer. */
    if (answer == state)
    {
        confirm = 0u;
    }
    else if ((answer == held) && (confirm > 0u))
    {
        confirm = terrain_process_count_add_sat(confirm, 1u, CONFIRM_BITS);
    }
    else
    {
        confirm = 1u;
    }

    uint8_t const checks = (answer == TERRAIN_SPEED_STATIONARY)
                               ? p_limits->stop_checks
                               : p_limits->confirm_checks;
    terrain_speed_state_t next = state;
    terrain_event_type_t event = TERRAIN_EVENT_NONE;
    if ((confirm > 0u) &&
        (confirm >= samples_capped(checks, TERRAIN_DETECT_CONFIRM_MAX)))
    {
        next = answer;
        event = (terrain_event_type_t)((uint32_t)TERRAIN_EVENT_STATIONARY +
                                       (uint32_t)answer);
        confirm = 0u;
    }

    word = terrain_process_field_put(word, SPEED_STATE_SHIFT, SPEED_STATE_BITS,
                                     (uint32_t)next);
    word = terrain_process_field_put(word, WINDOW_SHIFT, WINDOW_BITS, 0u);
    word = terrain_process_field_put(word, CANDIDATE_SHIFT, SPEED_STATE_BITS,
                                     (uint32_t)answer);
    word =
        terrain_process_field_put(word, CONFIRM_SHIFT, CONFIRM_BITS, confirm);
    word = terrain_process_field_put(word, LAST_SPEED_SHIFT, LAST_SPEED_BITS,
                                     code);
    *p_word = word;
    return event;
}

terrain_speed_state_t
terrain_detect_speed_state (uint32_t word)
{
    return (terrain_speed_state_t)terrain_process_field_get(
        word, SPEED_STATE_SHIFT, SPEED_STATE_BITS);
}

terrain_event_type_t
terrain_detect_lift_step (terrain_lift_t * p_state,
                          terrain_vec3_t const * p_sample,
                          terrain_vec3_t const * p_level,
                          terrain_lift_limits_t const * p_limits,
                          uint32_t * p_value)
{
    if ((p_state == NULL) || (p_sample == NULL) || (p_level == NULL) ||
        (p_limits == NULL) || (p_value == NULL))
    {
        return TERRAIN_EVENT_NONE;
    }
    float const level_sq = (p_level->x * p_level->x) +
                           (p_level->y * p_level->y) +
                           (p_level->z * p_level->z);
    if (!(level_sq > 0.0f))
    {
        return TERRAIN_EVENT_NONE;
    }

    uint32_t word = p_state->word;
    uint32_t next = robot_field_get(word, LIFT_NEXT);
    uint32_t held = robot_field_get(word, LIFT_HELD);
    if (next >= TERRAIN_DETECT_LIFT_WINDOW)
    {
        next = 0u; /* Not a state this code writes: start the window again. */
    }
    p_state->window[next][0] = to_cm(p_sample->x);
    p_state->window[next][1] = to_cm(p_sample->y);
    p_state->window[next][2] = to_cm(p_sample->z);
    next = (next + 1u) % TERRAIN_DETECT_LIFT_WINDOW;
    held = (held < TERRAIN_DETECT_LIFT_WINDOW) ? (held + 1u) : held;
    word = robot_field_put(word, LIFT_NEXT, next);
    word = robot_field_put(word, LIFT_HELD, held);
    if (held < TERRAIN_DETECT_LIFT_WINDOW)
    {
        p_state->word = word;
        return TERRAIN_EVENT_NONE;
    }

    int32_t sum[3] = {0, 0, 0};
    for (uint32_t slot = 0u; slot < TERRAIN_DETECT_LIFT_WINDOW; ++slot)
    {
        sum[0] += p_state->window[slot][0];
        sum[1] += p_state->window[slot][1];
        sum[2] += p_state->window[slot][2];
    }
    /* Divide, not multiply by a reciprocal: whole readings stay exact. */
    float const divisor = CM_PER_M * (float)TERRAIN_DETECT_LIFT_WINDOW;
    terrain_vec3_t const mean = {(float)sum[0] / divisor,
                                 (float)sum[1] / divisor,
                                 (float)sum[2] / divisor};
    float const up = ((mean.x * p_level->x) + (mean.y * p_level->y) +
                      (mean.z * p_level->z)) /
                     level_sq;
    float const tilt_deg = terrain_process_angle_deg(&mean, p_level);

    uint32_t run = robot_field_get(word, LIFT_RUN);
    if (tilt_deg > p_limits->over_deg)
    {
        run = (run < LIFT_COUNT_MAX) ? (run + 1u) : run;
    }
    else
    {
        run = 0u;
        word &= ~LIFT_OVER_REPORTED;
    }

    terrain_event_type_t event = TERRAIN_EVENT_NONE;
    uint32_t cooldown = robot_field_get(word, LIFT_COOLDOWN);
    if (cooldown > 0u)
    {
        --cooldown;
    }
    else if (up > p_limits->up_ratio)
    {
        event = TERRAIN_EVENT_LIFTED;
        *p_value = terrain_process_encode_unsigned(up, CM_PER_M,
                                                   LIFT_VALUE_BITS, false);
        cooldown = samples_needed(p_limits->cooldown_samples);
    }
    else if ((run >= samples_needed(p_limits->over_samples)) &&
             ((word & LIFT_OVER_REPORTED) == 0u))
    {
        event = TERRAIN_EVENT_OVERTURNED;
        *p_value = terrain_process_encode_unsigned(tilt_deg, 1.0f,
                                                   LIFT_VALUE_BITS, false);
        word |= LIFT_OVER_REPORTED;
        cooldown = samples_needed(p_limits->cooldown_samples);
    }
    else
    {
        /* Nothing to report. */
    }
    word = robot_field_put(word, LIFT_RUN, run);
    word = robot_field_put(word, LIFT_COOLDOWN, cooldown);
    p_state->word = word;
    return event;
}

/* Private function definitions. */

/* One more sample in a row while the condition holds, else start again. */
static uint32_t
count_if (bool b_condition, uint32_t count, uint32_t width)
{
    if (!b_condition)
    {
        return 0u;
    }
    return terrain_process_count_add_sat(count, 1u, width);
}

/* Degrees (0 or more) to tenths, rounded, stopping at the peak's 25.5. */
static uint32_t
to_ddeg (float deg)
{
    return terrain_process_encode_unsigned(deg, DDEG_PER_DEG, PEAK_BITS, false);
}

/* A wait of 0 samples would fire with no sample at all: use 1. */
/* A value in m/s^2 as cm/s^2 in 16 bits, rounded and clamped. */
static int16_t
to_cm (float mps2)
{
    return (int16_t)terrain_process_decode_signed(
        terrain_process_encode_signed(mps2, CM_PER_M, LIFT_SAMPLE_BITS),
        LIFT_SAMPLE_BITS);
}

static uint32_t
samples_needed (uint8_t samples)
{
    return (samples == 0u) ? 1u : (uint32_t)samples;
}

/* As samples_needed(), and no more than a countdown field holds. */
static uint32_t
samples_capped (uint8_t samples, uint32_t max)
{
    uint32_t const needed = samples_needed(samples);
    return (needed > max) ? max : needed;
}
