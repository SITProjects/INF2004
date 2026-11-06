/**
 * @file obstacle_width.h
 * @brief Hardware-free edge classification and width estimate for the sweep.
 *
 * Every function works only on its arguments: no hardware, kernel calls or
 * shared state. They may run on either core and are exercised directly by
 * obstacle_test.c. Echo values are durations in microseconds; zero means no
 * echo was returned. Angles use the shared grid in robot_types.h: index 0 is
 * 50 degrees (left), the centre index is 90 degrees and the last index is
 * 130 degrees (right).
 */

#ifndef OBSTACLE_WIDTH_H
#define OBSTACLE_WIDTH_H

/* Public includes. */
#include "../robot_types.h"
#include "drv_ultrasonic.h"
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** How one completed sweep position relates to the centre obstacle. */
typedef enum
{
    OBSTACLE_WIDTH_HIT,    /**< VALID and nearer than centre echo + jump. */
    OBSTACLE_WIDTH_MISS,   /**< NO_ECHO, or VALID at or beyond that range. */
    OBSTACLE_WIDTH_INVALID /**< Any other result: faults exhausted retries. */
} obstacle_width_class_t;

/** Outermost grid position on the obstacle, for one side. */
typedef struct
{
    uint8_t index;    /**< Grid index of the outermost HIT. */
    uint16_t echo_us; /**< Echo at that index in microseconds, nonzero. */
} obstacle_width_edge_t;

/** Range and geometry corrections; production passes the config macros. */
typedef struct
{
    uint32_t scale_mm_per_10k; /**< Millimetres per 10000 us, nonzero. */
    int32_t echo_offset_mm;    /**< Fitted range offset, signed mm. */
    float radial_offset_mm;    /**< Servo pivot to sensor face, signed mm. */
    float edge_correction_deg; /**< Inward move of found edges, degrees. */
} obstacle_width_config_t;

/** Packed fields for robot_obstacle_result_t, plus evidence for the log. */
typedef struct
{
    uint16_t width_mm;         /**< 1..511 mm, or 0 when unavailable. */
    uint8_t edges;             /**< robot_obstacle_pack_edges() byte. */
    bool b_left_found;         /**< A MISS lies just outside the left hit. */
    bool b_right_found;        /**< A MISS lies just outside the right hit. */
    bool b_collapsed;          /**< Correction exceeded the edge gap. */
    uint16_t left_bound_ddeg;  /**< Left boundary, 0.1 degree; 0 if none. */
    uint16_t right_bound_ddeg; /**< Right boundary, 0.1 degree; 0 if none. */
    int16_t left_x_mm;  /**< Left boundary, lateral mm from pivot, + left. */
    int16_t right_x_mm; /**< Right boundary, lateral mm from pivot, + left. */
} obstacle_width_result_t;

/* Public function declarations. */

/**
 * @brief First echo that is not on the obstacle at one grid position.
 * @param centre_echo_us Median near-vote echo of the confirming centre group.
 * @param index Grid index; one beyond the grid counts as the centre.
 * @return centre_echo_us / cos(angle off centre), rounded, plus
 *         OBSTACLE_EDGE_JUMP_US, in 32 bits: the echo a flat face square to
 *         the heading would give there, plus the margin. At the centre it
 *         is exactly centre_echo_us + OBSTACLE_EDGE_JUMP_US.
 */
uint32_t obstacle_width_hit_limit_us (uint16_t centre_echo_us, uint8_t index);

/**
 * @brief Classify one completed sweep position against the centre echo.
 * @param measurement Position result after retries: VALID, NO_ECHO or a
 *        fault. PENDING is not a completed result and classifies INVALID.
 * @param echo_us Echo in microseconds, used only when VALID.
 * @param centre_echo_us Median near-vote echo of the confirming centre group.
 * @param index Grid index of the position (see obstacle_width_hit_limit_us).
 * @return HIT when VALID and echo_us is below
 *         obstacle_width_hit_limit_us(centre_echo_us, index); MISS for
 *         NO_ECHO or a VALID echo at or beyond it; INVALID otherwise.
 */
obstacle_width_class_t obstacle_width_classify (ultrasonic_result_t measurement,
                                                uint16_t echo_us,
                                                uint16_t centre_echo_us,
                                                uint8_t index);

/**
 * @brief Estimate the obstacle width from the outermost hit on each side.
 *
 * A side's edge is found when its hit is inside the sweep (left index above
 * 0, right index below the last); a found edge is placed half a step outside
 * the hit and then moved inward by the edge correction. An edge that was not
 * found stays at the sweep end, so the width is a lower bound. Each edge
 * point uses r = echo * scale / 10000 + echo offset + radial offset, and the
 * width is the distance between the two points. If the correction makes the
 * edges cross, both move to the midpoint of the uncorrected edges and
 * b_collapsed is set. A width that rounds below 1 mm is reported as 1 mm,
 * meaning narrower than the sweep can resolve. Each boundary's lateral
 * offset is r * cos(angle), positive to the left, rounded and saturated to
 * the int16_t range.
 *
 * @param left Left hit: index 0 up to the centre index.
 * @param right Right hit: centre index up to the last index.
 * @param p_config Non-null corrections. The scale must be nonzero, both
 *        float fields finite, and the correction within
 *        0..OBSTACLE_EDGE_CORRECTION_MAX_DEG degrees.
 * @return Width saturated at ROBOT_OBSTACLE_WIDTH_MAX_MM with packed edges
 *         and flags, or obstacle_width_unavailable() for invalid input, a
 *         zero echo, a range at or below zero or a non-finite width.
 */
obstacle_width_result_t
obstacle_width_estimate (obstacle_width_edge_t left,
                         obstacle_width_edge_t right,
                         obstacle_width_config_t const * p_config);

/**
 * @brief Result for a sweep whose width cannot be measured.
 * @return Width ROBOT_OBSTACLE_WIDTH_UNAVAILABLE, both edges
 *         ROBOT_OBSTACLE_EDGE_NONE, every flag false and both offsets 0.
 */
obstacle_width_result_t obstacle_width_unavailable (void);

/**
 * @brief Convert an echo to the range from the servo pivot.
 *
 * r = echo * scale / 10000 + echo offset + radial offset, so the result is
 * measured from the servo pivot, not the sensor face.
 *
 * @param echo_us Echo in microseconds; zero means no echo.
 * @param p_config Corrections; NULL or a zero scale is rejected.
 * @param p_range_mm Non-null output, written only on success.
 * @return true with the range; false for a zero echo, invalid config, or a
 *         range that is not finite or not above zero.
 * Pure: any task, either core.
 */
bool obstacle_width_range_mm (uint16_t echo_us,
                              obstacle_width_config_t const * p_config,
                              float * p_range_mm);

#endif /* OBSTACLE_WIDTH_H */
