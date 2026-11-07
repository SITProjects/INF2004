/**
 * @file terrain_detect.c
 * @brief Terrain detectors: the hump, collision, turning and speed state
 *        machines.
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

/* Collision word: cooldown 6 | check countdown 4 | speed at the push 8 |
 * biggest push 7. */
#define COOLDOWN_SHIFT 0u
#define COOLDOWN_BITS 6u
#define CHECK_SHIFT 6u
#define CHECK_BITS 4u
#define SPEED_SHIFT 10u
#define SPEED_BITS 8u
#define PUSH_SHIFT 18u
#define PUSH_BITS 7u

/* Speed in 4 mm/s steps; push in tenths of a g. */
#define SPEED_STEPS_PER_MM_S 0.25f
#define SPEED_MM_S_PER_STEP 4.0f
#define DG_PER_G 10.0f

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
_Static_assert(TERRAIN_DETECT_CHECK_MAX == ((1u << CHECK_BITS) - 1u),
               "check limit must match the countdown width");
_Static_assert(TERRAIN_DETECT_COOLDOWN_MAX == ((1u << COOLDOWN_BITS) - 1u),
               "cooldown limit must match the cooldown width");
_Static_assert(TERRAIN_DETECT_PUSH_MAX_DG == ((1u << PUSH_BITS) - 1u),
               "push limit must match the push width");
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

float
terrain_detect_collision_push (float push_g, int32_t speed_mm_s, float lean_deg,
                               int32_t driving_mm_s, float max_lean_deg)
{
    return ((speed_mm_s >= driving_mm_s) && (lean_deg < max_lean_deg)) ? push_g
                                                                       : 0.0f;
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
terrain_detect_collision_step (uint32_t * p_word,
                               terrain_collision_input_t const * p_in,
                               terrain_collision_limits_t const * p_limits,
                               uint32_t * p_value)
{
    if ((p_word == NULL) || (p_in == NULL) || (p_limits == NULL) ||
        (p_value == NULL))
    {
        return TERRAIN_EVENT_NONE;
    }

    uint32_t word = *p_word;
    uint32_t cooldown =
        terrain_process_field_get(word, COOLDOWN_SHIFT, COOLDOWN_BITS);
    uint32_t check = terrain_process_field_get(word, CHECK_SHIFT, CHECK_BITS);
    uint32_t speed_code =
        terrain_process_field_get(word, SPEED_SHIFT, SPEED_BITS);
    uint32_t biggest = terrain_process_field_get(word, PUSH_SHIFT, PUSH_BITS);

    uint32_t const push = terrain_process_encode_unsigned(
        p_in->push_g, DG_PER_G, PUSH_BITS, false);
    float const speed = fabsf((float)p_in->speed_mm_s);

    /* Ramps push the car about too: only flat ground counts. */
    bool const b_level = (p_in->hump == TERRAIN_HUMP_LEVEL);
    bool const b_pushed = b_level && (p_in->push_g >= p_limits->limit_g);
    bool const b_sure = b_level && (p_in->push_g >= p_limits->sure_g);
    bool b_hit = false;

    if (cooldown > 0u)
    {
        /* COOLDOWN: the same hit may still be shaking the car. */
        cooldown--;
    }
    else if (check > 0u)
    {
        /* CHECKING: did the push stop the car, or was it a jolt? */
        if (!b_level)
        {
            check = 0u;
        }
        else
        {
            if (push > biggest)
            {
                biggest = push;
            }
            float const kept_mm_s = (float)speed_code * SPEED_MM_S_PER_STEP;
            bool const b_halved =
                !p_in->b_speed_known || ((speed * 2.0f) <= kept_mm_s);
            if (b_sure || b_halved)
            {
                b_hit = true;
            }
            else
            {
                check--;
            }
        }
    }
    else if (b_pushed)
    {
        /* READY, and pushed hard enough to be a hit. */
        biggest = push;
        if (b_sure || !p_in->b_speed_known ||
            (speed < p_limits->min_speed_mm_s))
        {
            /* Sure, or no speed to watch drop: the push decides. */
            b_hit = true;
        }
        else
        {
            check = samples_capped(p_limits->check_samples,
                                   TERRAIN_DETECT_CHECK_MAX);
            speed_code = terrain_process_encode_unsigned(
                speed, SPEED_STEPS_PER_MM_S, SPEED_BITS, false);
        }
    }
    else
    {
        /* READY, no hit. */
    }

    if (b_hit)
    {
        *p_value = biggest;
        cooldown = samples_capped(p_limits->cooldown_samples,
                                  TERRAIN_DETECT_COOLDOWN_MAX);
        check = 0u;
    }
    if (check == 0u)
    {
        /* Not checking: nothing to remember, so the word rests at 0. */
        speed_code = 0u;
        biggest = 0u;
    }

    word = terrain_process_field_put(word, COOLDOWN_SHIFT, COOLDOWN_BITS,
                                     cooldown);
    word = terrain_process_field_put(word, CHECK_SHIFT, CHECK_BITS, check);
    word = terrain_process_field_put(word, SPEED_SHIFT, SPEED_BITS, speed_code);
    word = terrain_process_field_put(word, PUSH_SHIFT, PUSH_BITS, biggest);
    *p_word = word;
    return b_hit ? TERRAIN_EVENT_COLLISION : TERRAIN_EVENT_NONE;
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
