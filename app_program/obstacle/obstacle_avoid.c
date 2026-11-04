/**
 * @file obstacle_avoid.c
 * @brief Hardware-free bypass planning and corridor checks.
 */

#include "obstacle_avoid.h"
#include "obstacle_config.h"
#include <math.h>
#include <stddef.h>

/* Private types, constants, and macros. */

#define DEG_TO_RAD (3.14159265f / 180.0f)

/* Static data. */

/* Private function declarations. */

static bool plan_usable (robot_obstacle_result_t result, bool * p_gap_right);

/* Public function definitions. */

obstacle_avoid_plan_t
obstacle_avoid_plan (robot_obstacle_result_t result,
                     obstacle_scan_geometry_t geometry,
                     obstacle_avoid_geometry_t const * p_geometry)
{
    obstacle_avoid_plan_t plan = {.status = OBSTACLE_AVOID_PLAN_NOT_PATH,
                                  .b_gap_right = false,
                                  .b_edge_unknown = false,
                                  .sidestep_mm = 0u,
                                  .pass_mm = 0u};
    if ((p_geometry == NULL) || !plan_usable(result, &plan.b_gap_right))
    {
        return plan;
    }
    if (geometry.centre_mm == 0u)
    {
        plan.status = OBSTACLE_AVOID_PLAN_NO_GEOMETRY;
        return plan;
    }

    uint8_t const edge_flag = plan.b_gap_right ? ROBOT_OBSTACLE_RIGHT_EDGE_FOUND
                                               : ROBOT_OBSTACLE_LEFT_EDGE_FOUND;
    bool const b_edge_found =
        ((result.flags & edge_flag) != 0u) &&
        (result.width_mm != ROBOT_OBSTACLE_WIDTH_UNAVAILABLE);
    if (b_edge_found)
    {
        int32_t edge_mm = plan.b_gap_right ? -(int32_t)geometry.right_x_mm
                                           : (int32_t)geometry.left_x_mm;
        int32_t need_mm = edge_mm + (int32_t)p_geometry->half_width_mm +
                          (int32_t)p_geometry->margin_mm;
        if (need_mm > (int32_t)p_geometry->sidestep_max_mm)
        {
            plan.status = OBSTACLE_AVOID_PLAN_TOO_WIDE;
            return plan;
        }
        plan.sidestep_mm = (need_mm < (int32_t)p_geometry->sidestep_min_mm)
                               ? p_geometry->sidestep_min_mm
                               : (uint16_t)need_mm;
    }
    else
    {
        plan.b_edge_unknown = true;
        plan.sidestep_mm = p_geometry->sidestep_max_mm;
    }

    uint32_t pass_mm = (uint32_t)geometry.centre_mm +
                       p_geometry->axle_to_pivot_mm + p_geometry->depth_mm +
                       p_geometry->rear_mm + p_geometry->margin_mm;
    if (pass_mm > p_geometry->pass_max_mm)
    {
        plan.status = OBSTACLE_AVOID_PLAN_TOO_FAR;
        return plan;
    }
    plan.pass_mm = (uint16_t)pass_mm;
    plan.status = OBSTACLE_AVOID_PLAN_OK;
    return plan;
}

uint8_t
obstacle_avoid_split_count (uint32_t distance_mm)
{
    uint32_t count = (distance_mm + (ROBOT_MOTION_CMD_DISTANCE_MAX_MM - 1u)) /
                     ROBOT_MOTION_CMD_DISTANCE_MAX_MM;
    return (count > UINT8_MAX) ? UINT8_MAX : (uint8_t)count;
}

uint16_t
obstacle_avoid_next_part (uint32_t remaining_mm)
{
    uint32_t count = obstacle_avoid_split_count(remaining_mm);
    if (count == 0u)
    {
        return 0u;
    }
    uint32_t part_mm = (remaining_mm + (count - 1u)) / count;
    return (part_mm > ROBOT_MOTION_CMD_DISTANCE_MAX_MM)
               ? (uint16_t)ROBOT_MOTION_CMD_DISTANCE_MAX_MM
               : (uint16_t)part_mm;
}

obstacle_avoid_corridor_t
obstacle_avoid_check_corridor (obstacle_scan_corridor_t const * p_corridor,
                               obstacle_width_config_t const * p_width,
                               obstacle_avoid_geometry_t const * p_geometry,
                               uint16_t pass_mm, bool b_gap_right,
                               uint16_t * p_shift_mm)
{
    if (p_shift_mm == NULL)
    {
        return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
    }
    *p_shift_mm = 0u;
    if ((p_corridor == NULL) || (p_width == NULL) || (p_geometry == NULL))
    {
        return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
    }

    float const half_mm =
        (float)p_geometry->half_width_mm + (float)p_geometry->margin_mm;
    float const length_mm =
        (float)pass_mm + (float)p_geometry->axle_to_pivot_mm;
    float shift_mm = 0.0f;
    float limit_mm = INFINITY;
    for (uint8_t index = 0u; index < OBSTACLE_SWEEP_POINTS; ++index)
    {
        if ((p_corridor->valid_mask & (uint16_t)(1u << index)) == 0u)
        {
            return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
        }
        uint16_t const echo_us = p_corridor->echo_us[index];
        if (echo_us == 0u)
        {
            continue; /* No echo: nothing at this angle. */
        }
        float range_mm;
        if (!obstacle_width_range_mm(echo_us, p_width, &range_mm))
        {
            return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
        }
        float const angle_rad =
            (float)(OBSTACLE_SWEEP_FIRST_DEG +
                    (OBSTACLE_SWEEP_STEP_DEG * (uint32_t)index)) *
            DEG_TO_RAD;
        float const x_mm = range_mm * cosf(angle_rad);
        float const y_mm =
            (float)p_geometry->axle_to_pivot_mm + (range_mm * sinf(angle_rad));
        if (!(y_mm < length_mm))
        {
            continue; /* Beyond the end of the pass. */
        }
        float const gap_mm = b_gap_right ? -x_mm : x_mm;
        if (fabsf(x_mm) < half_mm)
        {
            float const need_mm = gap_mm + half_mm;
            if (need_mm > shift_mm)
            {
                shift_mm = need_mm;
            }
        }
        else if (gap_mm >= half_mm)
        {
            float const room_mm = gap_mm - half_mm;
            if (room_mm < limit_mm)
            {
                limit_mm = room_mm;
            }
        }
        else
        {
            /* Already clear on the far side. */
        }
    }

    if (!(shift_mm > 0.0f))
    {
        return OBSTACLE_AVOID_CORRIDOR_CLEAR;
    }
    if ((shift_mm > limit_mm) || !(shift_mm < (float)UINT16_MAX))
    {
        return OBSTACLE_AVOID_CORRIDOR_BLOCKED;
    }
    *p_shift_mm = (uint16_t)ceilf(shift_mm);
    return OBSTACLE_AVOID_CORRIDOR_SHIFT;
}

/* Private function definitions. */

/* Only PATH_FOUND toward one side can be bypassed. */
static bool
plan_usable (robot_obstacle_result_t result, bool * p_gap_right)
{
    if (robot_obstacle_get_status(result.flags) != ROBOT_OBSTACLE_PATH_FOUND)
    {
        return false;
    }
    robot_obstacle_direction_t const direction =
        robot_obstacle_get_direction(result.flags);
    if (direction == ROBOT_OBSTACLE_DIR_LEFT_60)
    {
        *p_gap_right = false;
        return true;
    }
    if (direction == ROBOT_OBSTACLE_DIR_RIGHT_120)
    {
        *p_gap_right = true;
        return true;
    }
    return false;
}
