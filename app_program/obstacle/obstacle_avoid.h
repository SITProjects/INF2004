/**
 * @file obstacle_avoid.h
 * @brief Hardware-free bypass planning and corridor checks for obstacles.
 *
 * The bypass is a box: turn 90 degrees toward the gap, sidestep, turn back,
 * verify the corridor ahead, pass the obstacle, turn toward the line, drive
 * back to it with stop-at-line, turn to the original heading and follow.
 *
 * Every function works only on its arguments: no hardware, kernel calls or
 * shared state, so they may run on either core and obstacle_test.c drives
 * them directly. Distances are millimetres. Ranges are from the servo pivot
 * (obstacle_width_range_mm()); lateral offsets are + left. Turns are in
 * place about the axle midpoint, so the pass distance measured before the
 * sidestep still holds after it.
 */

#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H

/* Public includes. */
#include "../robot_types.h"
#include "obstacle_scan.h"
#include "obstacle_width.h"
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** Car dimensions and bypass limits; production passes the config macros. */
typedef struct
{
    uint16_t half_width_mm;    /**< Half the car's width, W / 2. */
    uint16_t margin_mm;        /**< Clearance kept from the obstacle. */
    uint16_t axle_to_pivot_mm; /**< Axle midpoint to servo pivot, forward. */
    uint16_t rear_mm;          /**< Axle midpoint to rear bumper. */
    uint16_t depth_mm;         /**< Assumed obstacle depth (not measured). */
    uint16_t sidestep_min_mm;  /**< Smallest first sidestep. */
    uint16_t sidestep_max_mm;  /**< Largest first sidestep. */
    uint16_t lateral_max_mm;   /**< Largest total offset from the line. */
    uint16_t pass_max_mm;      /**< Largest pass distance accepted. */
    uint16_t return_extra_mm;  /**< Drive past the offset to find the line. */
    uint16_t overshoot_mm;     /**< IR row to axle; 0 skips that leg. */
} obstacle_avoid_geometry_t;

/** Why obstacle_avoid_plan() accepted or refused a result. */
typedef enum
{
    OBSTACLE_AVOID_PLAN_OK,          /**< A bypass is possible. */
    OBSTACLE_AVOID_PLAN_NOT_PATH,    /**< Not PATH_FOUND toward 60 or 120. */
    OBSTACLE_AVOID_PLAN_NO_GEOMETRY, /**< No centre distance published. */
    OBSTACLE_AVOID_PLAN_TOO_WIDE,    /**< Needed sidestep > sidestep_max. */
    OBSTACLE_AVOID_PLAN_TOO_FAR      /**< Pass distance > pass_max. */
} obstacle_avoid_plan_status_t;

/** A planned bypass; the distances are valid only when status is OK. */
typedef struct
{
    obstacle_avoid_plan_status_t status;
    bool b_gap_right;     /**< Bypass on the right (DIR_RIGHT_120). */
    bool b_edge_unknown;  /**< Gap-side edge not found: sidestep is maximal. */
    uint16_t sidestep_mm; /**< First sidestep. */
    uint16_t pass_mm;     /**< Axle travel parallel to the line. */
} obstacle_avoid_plan_t;

/** Result of checking a corridor sweep against the path ahead. */
typedef enum
{
    OBSTACLE_AVOID_CORRIDOR_CLEAR,  /**< Nothing in the path ahead. */
    OBSTACLE_AVOID_CORRIDOR_SHIFT,  /**< Sidestep the given extra first. */
    OBSTACLE_AVOID_CORRIDOR_BLOCKED /**< Invalid data, or no shift fits. */
} obstacle_avoid_corridor_t;

/* Public function declarations. */

/**
 * @brief Plan a box bypass from a published side result.
 *
 * Gap on the left (the right mirrors it with -right_x_mm): with the left
 * edge found and the width available, the needed sidestep is
 * left_x + half_width + margin; it is raised to sidestep_min and refused
 * (TOO_WIDE) above sidestep_max, never clamped down. Without that edge the
 * sidestep is sidestep_max and b_edge_unknown is set: a corridor check must
 * then decide. The pass distance is centre + axle_to_pivot + depth + rear
 * + margin, never shortened; above pass_max it is refused (TOO_FAR).
 *
 * @param result Published result; must be PATH_FOUND toward 60 or 120.
 * @param geometry Geometry published with result.
 * @param p_geometry Non-null car dimensions and limits.
 * @return The plan; check status before using the distances.
 */
obstacle_avoid_plan_t
obstacle_avoid_plan (robot_obstacle_result_t result,
                     obstacle_scan_geometry_t geometry,
                     obstacle_avoid_geometry_t const * p_geometry);

/**
 * @brief Count the motion commands needed to drive a distance.
 * @param distance_mm Distance in millimetres.
 * @return 0 for 0, else the number of parts of at most
 *         ROBOT_MOTION_CMD_DISTANCE_MAX_MM each.
 */
uint8_t obstacle_avoid_split_count (uint32_t distance_mm);

/**
 * @brief Size the next command of a split move.
 * @param remaining_mm Distance still to drive.
 * @return The remaining distance divided evenly over
 *         obstacle_avoid_split_count() parts, rounded up; 0 for 0.
 */
uint16_t obstacle_avoid_next_part (uint32_t remaining_mm);

/**
 * @brief Check a corridor sweep against the path the car will pass along.
 *
 * Each usable non-zero echo at angle 60 + 5 i degrees gives a point at
 * x = r cos(angle), y = axle_to_pivot + r sin(angle) from the axle. Points
 * with y below pass_mm + axle_to_pivot are in range. Let g be x measured
 * toward the gap. A point in range with |x| < half_width + margin is in the
 * path; the shift is the largest g + half_width + margin over those, which
 * puts each one on the far side. A point in range on the gap side outside
 * the path (g >= half_width + margin) limits the shift to
 * g - half_width - margin. The 60-120 degree sweep cannot see beside the
 * front wheels; the first sweep's edges cover that area, this does not.
 *
 * @param p_corridor Non-null corridor sweep, taken with the car stopped.
 * @param p_width Non-null range corrections.
 * @param p_geometry Non-null car dimensions.
 * @param pass_mm Axle travel still to come parallel to the line.
 * @param b_gap_right True when the bypass is on the right.
 * @param p_shift_mm Non-null; set to the shift for SHIFT, else 0.
 * @return CLEAR with nothing in the path; SHIFT with *p_shift_mm; BLOCKED
 *         if any position is invalid, a range cannot be computed, or the
 *         shift would move the car into a gap-side point.
 */
obstacle_avoid_corridor_t
obstacle_avoid_check_corridor (obstacle_scan_corridor_t const * p_corridor,
                               obstacle_width_config_t const * p_width,
                               obstacle_avoid_geometry_t const * p_geometry,
                               uint16_t pass_mm, bool b_gap_right,
                               uint16_t * p_shift_mm);

#endif /* OBSTACLE_AVOID_H */
