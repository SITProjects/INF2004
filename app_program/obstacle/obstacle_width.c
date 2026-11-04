/**
 * @file obstacle_width.c
 * @brief Hardware-free sweep classification and width geometry.
 */

#include "obstacle_width.h"
#include "obstacle_config.h"
#include <math.h>
#include <stddef.h>

/* Private types, constants, and macros. */

#define LAST_INDEX ((uint8_t)(OBSTACLE_SWEEP_POINTS - 1u))
#define CENTRE_INDEX ((uint8_t)OBSTACLE_CENTRE_INDEX)
#define DEG_TO_RAD (3.14159265f / 180.0f)
#define HALF_STEP_DEG ((float)OBSTACLE_SWEEP_STEP_DEG / 2.0f)

/* Rounding to whole millimetres reaches the field maximum from here. */
#define WIDTH_SATURATE_MM ((float)ROBOT_OBSTACLE_WIDTH_MAX_MM + 0.5f)

/* Static data. */

/* Private function declarations. */

static bool config_valid (obstacle_width_config_t const * p_config);
static float index_angle_deg (uint8_t index);
static uint16_t angle_ddeg (float angle_deg);
static int16_t lateral_mm (float range_mm, float angle_deg);

/* Public function definitions. */

obstacle_width_class_t
obstacle_width_classify (ultrasonic_result_t measurement, uint16_t echo_us,
                         uint16_t centre_echo_us)
{
    if (measurement == ULTRASONIC_NO_ECHO)
    {
        return OBSTACLE_WIDTH_MISS;
    }
    if (measurement != ULTRASONIC_VALID)
    {
        return OBSTACLE_WIDTH_INVALID;
    }
    uint32_t limit_us = (uint32_t)centre_echo_us + OBSTACLE_EDGE_JUMP_US;
    return ((uint32_t)echo_us < limit_us) ? OBSTACLE_WIDTH_HIT
                                          : OBSTACLE_WIDTH_MISS;
}

obstacle_width_result_t
obstacle_width_estimate (obstacle_width_edge_t left,
                         obstacle_width_edge_t right,
                         obstacle_width_config_t const * p_config)
{
    if (!config_valid(p_config) || (left.index > CENTRE_INDEX) ||
        (right.index < CENTRE_INDEX) || (right.index > LAST_INDEX))
    {
        return obstacle_width_unavailable();
    }
    float left_range_mm;
    float right_range_mm;
    if (!obstacle_width_range_mm(left.echo_us, p_config, &left_range_mm) ||
        !obstacle_width_range_mm(right.echo_us, p_config, &right_range_mm))
    {
        return obstacle_width_unavailable();
    }

    bool b_left_found = left.index > 0u;
    bool b_right_found = right.index < LAST_INDEX;
    float left_edge_deg = index_angle_deg(left.index);
    float right_edge_deg = index_angle_deg(right.index);
    if (b_left_found)
    {
        left_edge_deg -= HALF_STEP_DEG;
    }
    if (b_right_found)
    {
        right_edge_deg += HALF_STEP_DEG;
    }

    float left_deg = left_edge_deg;
    float right_deg = right_edge_deg;
    if (b_left_found)
    {
        left_deg += p_config->edge_correction_deg;
    }
    if (b_right_found)
    {
        right_deg -= p_config->edge_correction_deg;
    }
    bool b_collapsed = left_deg > right_deg;
    if (b_collapsed)
    {
        left_deg = (left_edge_deg + right_edge_deg) / 2.0f;
        right_deg = left_deg;
    }

    float dx = (left_range_mm * cosf(left_deg * DEG_TO_RAD)) -
               (right_range_mm * cosf(right_deg * DEG_TO_RAD));
    float dy = (left_range_mm * sinf(left_deg * DEG_TO_RAD)) -
               (right_range_mm * sinf(right_deg * DEG_TO_RAD));
    float width_mm = sqrtf((dx * dx) + (dy * dy));
    if (!isfinite(width_mm))
    {
        return obstacle_width_unavailable();
    }

    /* Saturate before converting, so the conversion is always in range. */
    uint32_t rounded_mm = (width_mm >= WIDTH_SATURATE_MM)
                              ? ROBOT_OBSTACLE_WIDTH_MAX_MM
                              : (uint32_t)(width_mm + 0.5f);
    if (rounded_mm < 1u)
    {
        rounded_mm = 1u;
    }

    obstacle_width_result_t result = {
        .width_mm = robot_obstacle_clamp_width(rounded_mm),
        .edges = robot_obstacle_pack_edges(left.index, right.index),
        .b_left_found = b_left_found,
        .b_right_found = b_right_found,
        .b_collapsed = b_collapsed,
        .left_bound_ddeg = angle_ddeg(left_deg),
        .right_bound_ddeg = angle_ddeg(right_deg),
        .left_x_mm = lateral_mm(left_range_mm, left_deg),
        .right_x_mm = lateral_mm(right_range_mm, right_deg)};
    return result;
}

obstacle_width_result_t
obstacle_width_unavailable (void)
{
    obstacle_width_result_t result = {
        .width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE,
        .edges = robot_obstacle_pack_edges(ROBOT_OBSTACLE_EDGE_NONE,
                                           ROBOT_OBSTACLE_EDGE_NONE),
        .b_left_found = false,
        .b_right_found = false,
        .b_collapsed = false,
        .left_bound_ddeg = 0u,
        .right_bound_ddeg = 0u,
        .left_x_mm = 0,
        .right_x_mm = 0};
    return result;
}

bool
obstacle_width_range_mm (uint16_t echo_us,
                         obstacle_width_config_t const * p_config,
                         float * p_range_mm)
{
    if ((echo_us == 0u) || (p_range_mm == NULL) || (p_config == NULL) ||
        (p_config->scale_mm_per_10k == 0u))
    {
        return false;
    }
    float range_mm =
        (((float)echo_us * (float)p_config->scale_mm_per_10k) / 10000.0f) +
        (float)p_config->echo_offset_mm + p_config->radial_offset_mm;
    if (!isfinite(range_mm) || !(range_mm > 0.0f))
    {
        return false;
    }
    *p_range_mm = range_mm;
    return true;
}

/* Private function definitions. */

static bool
config_valid (obstacle_width_config_t const * p_config)
{
    return (p_config != NULL) && (p_config->scale_mm_per_10k != 0u) &&
           isfinite(p_config->radial_offset_mm) &&
           isfinite(p_config->edge_correction_deg) &&
           (p_config->edge_correction_deg >= 0.0f) &&
           (p_config->edge_correction_deg <= OBSTACLE_EDGE_CORRECTION_MAX_DEG);
}

static float
index_angle_deg (uint8_t index)
{
    return (float)(OBSTACLE_SWEEP_FIRST_DEG +
                   (OBSTACLE_SWEEP_STEP_DEG * index));
}

/* Boundary angles lie within the sweep, 60..120 degrees, so this fits. */
static uint16_t
angle_ddeg (float angle_deg)
{
    return (uint16_t)((angle_deg * 10.0f) + 0.5f);
}

/* Lateral offset of a boundary, + left; saturated to the int16_t range. */
static int16_t
lateral_mm (float range_mm, float angle_deg)
{
    float x_mm = range_mm * cosf(angle_deg * DEG_TO_RAD);
    if (!(x_mm < (float)INT16_MAX))
    {
        return INT16_MAX;
    }
    if (!(x_mm > (float)INT16_MIN))
    {
        return INT16_MIN;
    }
    return (int16_t)((x_mm >= 0.0f) ? (x_mm + 0.5f) : (x_mm - 0.5f));
}
