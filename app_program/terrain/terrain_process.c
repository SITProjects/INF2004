/**
 * @file terrain_process.c
 * @brief IMU processing: axis mapping, calibration, tilt, motion state,
 *        humps.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_process.h"
#include <math.h>
#include <stddef.h>

/* Private types, constants, and macros. */
#define DEG_PER_RAD 57.2957795f

/* Widest field and counter the word helpers handle, bits. */
#define WORD_MAX_FIELD_BITS 31u
#define WORD_BITS 32u

/* Static data. */

/* Private function declarations. */
static float map_one (terrain_axis_map_t const * p_map,
                      float const gain[TERRAIN_AXES], uint32_t car_axis,
                      int16_t const board[TERRAIN_AXES], float scale);
static float length (terrain_vec3_t const * p_v);
static float largest_spread (terrain_calib_acc_t const * p_acc);
static bool field_ok (uint32_t shift, uint32_t width);
static uint32_t field_mask (uint32_t width);
static void tilt_step (terrain_lowpass_t * p_filter,
                       terrain_vec3_t const * p_accel, float alpha,
                       uint32_t * p_flags, uint32_t primed_flag);

/* Public function definitions. */

terrain_vec3_t
terrain_process_map_axes (terrain_axis_map_t const * p_map,
                          float const gain[TERRAIN_AXES],
                          int16_t const board[TERRAIN_AXES], float scale)
{
    terrain_vec3_t car = {0.0f, 0.0f, 0.0f};

    if ((p_map == NULL) || (gain == NULL) || (board == NULL))
    {
        return car;
    }

    for (uint32_t i = 0u; i < TERRAIN_AXES; i++)
    {
        if (p_map->from[i] >= TERRAIN_AXES)
        {
            return car;
        }
    }

    car.x = map_one(p_map, gain, 0u, board, scale);
    car.y = map_one(p_map, gain, 1u, board, scale);
    car.z = map_one(p_map, gain, 2u, board, scale);
    return car;
}

void
terrain_process_calib_reset (terrain_calib_acc_t * p_acc)
{
    if (p_acc == NULL)
    {
        return;
    }

    terrain_calib_acc_t const empty = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0u};
    *p_acc = empty;
}

void
terrain_process_calib_add (terrain_calib_acc_t * p_acc,
                           terrain_vec3_t const * p_sample)
{
    if ((p_acc == NULL) || (p_sample == NULL) || (p_acc->count == UINT8_MAX))
    {
        return;
    }

    /* The first sample sets min and max; later ones widen them. */
    if (p_acc->count == 0u)
    {
        p_acc->min = *p_sample;
        p_acc->max = *p_sample;
    }
    else
    {
        p_acc->min.x = fminf(p_acc->min.x, p_sample->x);
        p_acc->min.y = fminf(p_acc->min.y, p_sample->y);
        p_acc->min.z = fminf(p_acc->min.z, p_sample->z);
        p_acc->max.x = fmaxf(p_acc->max.x, p_sample->x);
        p_acc->max.y = fmaxf(p_acc->max.y, p_sample->y);
        p_acc->max.z = fmaxf(p_acc->max.z, p_sample->z);
    }

    p_acc->sum.x += p_sample->x;
    p_acc->sum.y += p_sample->y;
    p_acc->sum.z += p_sample->z;
    p_acc->count++;
}

terrain_calib_result_t
terrain_process_calib_finish (terrain_calib_acc_t const * p_acc,
                              terrain_vec3_t const * p_level_ref,
                              terrain_calib_limits_t const * p_limits,
                              terrain_calib_out_t * p_out)
{
    if (p_out == NULL)
    {
        return TERRAIN_CALIB_RESULT_INVALID;
    }

    terrain_calib_out_t const empty = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f};
    *p_out = empty;

    if ((p_acc == NULL) || (p_level_ref == NULL) || (p_limits == NULL) ||
        (p_acc->count == 0u))
    {
        return TERRAIN_CALIB_RESULT_INVALID;
    }

    float const n = (float)p_acc->count;
    p_out->mean.x = p_acc->sum.x / n;
    p_out->mean.y = p_acc->sum.y / n;
    p_out->mean.z = p_acc->sum.z / n;

    /* Level ground should read 0, 0, g; the difference is the bias. */
    p_out->bias.x = p_out->mean.x;
    p_out->bias.y = p_out->mean.y;
    p_out->bias.z = p_out->mean.z - TERRAIN_GRAVITY_MPS2;

    p_out->spread_mps2 = largest_spread(p_acc);
    p_out->tilt_deg = terrain_process_angle_deg(&p_out->mean, p_level_ref);

    /*
     * Compare sizes with the reference, not with g: before calibration
     * the readings still include the chip's offset.
     */
    float gravity_err = fabsf(length(&p_out->mean) - length(p_level_ref));

    if ((p_out->spread_mps2 > p_limits->max_spread_mps2) ||
        (gravity_err > p_limits->max_gravity_err_mps2))
    {
        return TERRAIN_CALIB_RESULT_MOVED;
    }
    if (p_out->tilt_deg > p_limits->max_tilt_deg)
    {
        return TERRAIN_CALIB_RESULT_NOT_LEVEL;
    }
    return TERRAIN_CALIB_RESULT_OK;
}

float
terrain_process_angle_deg (terrain_vec3_t const * p_a,
                           terrain_vec3_t const * p_b)
{
    if ((p_a == NULL) || (p_b == NULL))
    {
        return 0.0f;
    }

    terrain_vec3_t const cross = {(p_a->y * p_b->z) - (p_a->z * p_b->y),
                                  (p_a->z * p_b->x) - (p_a->x * p_b->z),
                                  (p_a->x * p_b->y) - (p_a->y * p_b->x)};
    float dot = (p_a->x * p_b->x) + (p_a->y * p_b->y) + (p_a->z * p_b->z);

    /* atan2 stays accurate for small angles, where acos does not. */
    return atan2f(length(&cross), dot) * DEG_PER_RAD;
}

terrain_vec3_t
terrain_process_correct (terrain_vec3_t const * p_sample,
                         terrain_vec3_t const * p_bias)
{
    terrain_vec3_t out = {0.0f, 0.0f, 0.0f};

    if ((p_sample == NULL) || (p_bias == NULL))
    {
        return out;
    }

    out.x = p_sample->x - p_bias->x;
    out.y = p_sample->y - p_bias->y;
    out.z = p_sample->z - p_bias->z;
    return out;
}

float
terrain_process_lowpass_alpha (float tau_s, float period_s)
{
    if (tau_s <= 0.0f)
    {
        return 1.0f;
    }
    if (period_s <= 0.0f)
    {
        return 0.0f;
    }
    return TERRAIN_PROCESS_LOWPASS_ALPHA(tau_s, period_s);
}

void
terrain_process_lowpass_reset (terrain_lowpass_t * p_filter)
{
    if (p_filter == NULL)
    {
        return;
    }

    terrain_lowpass_t const empty = {{0.0f, 0.0f, 0.0f}};
    *p_filter = empty;
}

void
terrain_process_lowpass_step (terrain_lowpass_t * p_filter,
                              terrain_vec3_t const * p_sample, float alpha,
                              bool b_first)
{
    if ((p_filter == NULL) || (p_sample == NULL))
    {
        return;
    }

    if (b_first)
    {
        p_filter->value = *p_sample;
        return;
    }

    p_filter->value.x += alpha * (p_sample->x - p_filter->value.x);
    p_filter->value.y += alpha * (p_sample->y - p_filter->value.y);
    p_filter->value.z += alpha * (p_sample->z - p_filter->value.z);
}

float
terrain_process_pitch_deg (terrain_vec3_t const * p_accel)
{
    if (p_accel == NULL)
    {
        return 0.0f;
    }

    /* Forward part of gravity against the rest, so roll cancels out. */
    float rest = sqrtf((p_accel->x * p_accel->x) + (p_accel->z * p_accel->z));
    return atan2f(p_accel->y, rest) * DEG_PER_RAD;
}

float
terrain_process_roll_deg (terrain_vec3_t const * p_accel)
{
    if (p_accel == NULL)
    {
        return 0.0f;
    }
    return atan2f(p_accel->x, p_accel->z) * DEG_PER_RAD;
}

float
terrain_process_turn_rate_dps (int16_t left_mm_s, int16_t right_mm_s,
                               float track_mm)
{
    if (!(track_mm > 0.0f))
    {
        return 0.0f;
    }

    /*
     * Each second the right wheel gets (right - left) mm further than the
     * left; across the track that turns the car by (right - left) / track
     * radians, like pushing one end of a door.
     */
    float const diff_mm_s = (float)right_mm_s - (float)left_mm_s;
    return (diff_mm_s / track_mm) * DEG_PER_RAD;
}

void
terrain_process_tilt_reset (terrain_tilt_t * p_tilt)
{
    if (p_tilt == NULL)
    {
        return;
    }

    terrain_process_lowpass_reset(&p_tilt->pitch_filter);
    terrain_process_lowpass_reset(&p_tilt->roll_filter);
    p_tilt->pitch_deg = 0.0f;
    p_tilt->roll_deg = 0.0f;
    p_tilt->flags = 0u;
}

void
terrain_process_tilt_update (terrain_tilt_t * p_tilt,
                             terrain_vec3_t const * p_accel, float alpha,
                             bool b_hold_pitch, bool b_hold_roll)
{
    if ((p_tilt == NULL) || (p_accel == NULL))
    {
        return;
    }

    if (!b_hold_pitch)
    {
        tilt_step(&p_tilt->pitch_filter, p_accel, alpha, &p_tilt->flags,
                  TERRAIN_TILT_PITCH_PRIMED);
        p_tilt->pitch_deg =
            terrain_process_pitch_deg(&p_tilt->pitch_filter.value);
    }
    if (!b_hold_roll)
    {
        tilt_step(&p_tilt->roll_filter, p_accel, alpha, &p_tilt->flags,
                  TERRAIN_TILT_ROLL_PRIMED);
        p_tilt->roll_deg = terrain_process_roll_deg(&p_tilt->roll_filter.value);
    }

    p_tilt->flags &= ~(TERRAIN_TILT_PITCH_HELD | TERRAIN_TILT_ROLL_HELD);
    if (b_hold_pitch)
    {
        p_tilt->flags |= TERRAIN_TILT_PITCH_HELD;
    }
    if (b_hold_roll)
    {
        p_tilt->flags |= TERRAIN_TILT_ROLL_HELD;
    }
}

uint32_t
terrain_process_field_put (uint32_t word, uint32_t shift, uint32_t width,
                           uint32_t code)
{
    if (!field_ok(shift, width))
    {
        return word;
    }

    uint32_t const mask = field_mask(width);
    return (word & ~(mask << shift)) | ((code & mask) << shift);
}

uint32_t
terrain_process_field_get (uint32_t word, uint32_t shift, uint32_t width)
{
    if (!field_ok(shift, width))
    {
        return 0u;
    }
    return (word >> shift) & field_mask(width);
}

uint32_t
terrain_process_encode_signed (float value, float steps_per_unit,
                               uint32_t width)
{
    if ((width < 2u) || (width > TERRAIN_FIELD_MAX_BITS))
    {
        return 0u;
    }

    float steps = value * steps_per_unit;
    if (isnan(steps))
    {
        return 0u;
    }

    /* Clamp, never wrap: a value past the limit reads as the limit. */
    float const half = (float)(1u << (width - 1u));
    steps = roundf(steps);
    if (steps > (half - 1.0f))
    {
        steps = half - 1.0f;
    }
    if (steps < -half)
    {
        steps = -half;
    }

    /* Two's complement in unsigned maths only: -1 -> 2^width - 1. */
    uint32_t const span = 1u << width;
    int32_t const signed_steps = (int32_t)steps;
    return (uint32_t)(signed_steps + (int32_t)span) % span;
}

int32_t
terrain_process_decode_signed (uint32_t code, uint32_t width)
{
    if ((width < 2u) || (width > TERRAIN_FIELD_MAX_BITS))
    {
        return 0;
    }

    uint32_t const span = 1u << width;
    uint32_t const unsigned_code = code % span;
    if (unsigned_code >= (span / 2u))
    {
        return (int32_t)unsigned_code - (int32_t)span;
    }
    return (int32_t)unsigned_code;
}

uint32_t
terrain_process_encode_unsigned (float value, float steps_per_unit,
                                 uint32_t width, bool b_round_up)
{
    if ((width < 1u) || (width > TERRAIN_FIELD_MAX_BITS))
    {
        return 0u;
    }

    float steps = value * steps_per_unit;
    if (isnan(steps) || (steps <= 0.0f))
    {
        return 0u;
    }

    /* Rounding up can read one step high, never low. */
    steps = b_round_up ? ceilf(steps) : roundf(steps);
    float const largest = (float)field_mask(width);
    if (steps > largest)
    {
        steps = largest;
    }
    return (uint32_t)steps;
}

uint32_t
terrain_process_count_add_sat (uint32_t count, uint32_t add, uint32_t width)
{
    if ((width < 1u) || (width > WORD_MAX_FIELD_BITS))
    {
        return count;
    }

    uint32_t const largest = field_mask(width);
    if ((count >= largest) || (add >= (largest - count)))
    {
        return largest;
    }
    return count + add;
}

uint32_t
terrain_process_count_add_wrap (uint32_t count, uint32_t add, uint32_t width)
{
    if ((width < 1u) || (width > WORD_MAX_FIELD_BITS))
    {
        return count;
    }

    /* Deliberate wrap: readers only use differences. */
    return (count + add) & field_mask(width);
}

/* Private function definitions. */

static float
map_one (terrain_axis_map_t const * p_map, float const gain[TERRAIN_AXES],
         uint32_t car_axis, int16_t const board[TERRAIN_AXES], float scale)
{
    uint8_t const board_axis = p_map->from[car_axis];
    float value = (float)board[board_axis] * scale * gain[board_axis];
    return (p_map->sign[car_axis] < 0) ? -value : value;
}

static float
length (terrain_vec3_t const * p_v)
{
    return sqrtf((p_v->x * p_v->x) + (p_v->y * p_v->y) + (p_v->z * p_v->z));
}

static float
largest_spread (terrain_calib_acc_t const * p_acc)
{
    float spread = p_acc->max.x - p_acc->min.x;
    spread = fmaxf(spread, p_acc->max.y - p_acc->min.y);
    spread = fmaxf(spread, p_acc->max.z - p_acc->min.z);
    return spread;
}

static bool
field_ok (uint32_t shift, uint32_t width)
{
    return (width >= 1u) && (width <= WORD_MAX_FIELD_BITS) &&
           (shift < WORD_BITS) && ((shift + width) <= WORD_BITS);
}

/* Lowest width bits set; width is 1 to 31. */
static uint32_t
field_mask (uint32_t width)
{
    return (1u << width) - 1u;
}

/* One filter step; the first sample after a reset seeds the filter. */
static void
tilt_step (terrain_lowpass_t * p_filter, terrain_vec3_t const * p_accel,
           float alpha, uint32_t * p_flags, uint32_t primed_flag)
{
    bool const b_first = ((*p_flags & primed_flag) == 0u);
    terrain_process_lowpass_step(p_filter, p_accel, alpha, b_first);
    *p_flags |= primed_flag;
}
