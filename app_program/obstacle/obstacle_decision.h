/**
 * @file obstacle_decision.h
 * @brief Hardware-free obstacle decisions: ping median, position result,
 *        centre action and side selection.
 *
 * Every function works only on its arguments: no hardware, kernel calls or
 * shared state. They may run on either core and are exercised directly by
 * obstacle_test.c. Echo values are durations in microseconds; zero means no
 * echo was returned.
 */

#ifndef OBSTACLE_DECISION_H
#define OBSTACLE_DECISION_H

/* Public includes. */
#include "../robot_types.h"
#include "drv_ultrasonic.h"
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** Largest sample count obstacle_decision_median() examines. */
#define OBSTACLE_DECISION_MEDIAN_MAX 3u

/** Next step after a completed centre measurement. */
typedef enum
{
    OBSTACLE_CENTRE_CLEAR,     /**< Publish CLEAR with direction 90 degrees. */
    OBSTACLE_CENTRE_UNKNOWN,   /**< Publish UNKNOWN without a direction. */
    OBSTACLE_CENTRE_SCAN_SIDES /**< Obstacle ahead: measure both sides. */
} obstacle_centre_action_t;

/** Status and candidate direction decided from both side positions. */
typedef struct
{
    robot_obstacle_status_t status;       /**< PATH_FOUND, BLOCKED, UNKNOWN. */
    robot_obstacle_direction_t direction; /**< LEFT_60, RIGHT_120 or NONE. */
} obstacle_side_decision_t;

/* Public function declarations. */

/**
 * @brief Median of the nonzero echo samples.
 *
 * This filter does not use a majority vote. One nonzero echo determines the
 * value even when every other ping is NO_ECHO: {0, 0, 300} returns 300 us.
 * For a position without exhausted faults, that single echo is VALID and
 * starts a side scan if it is below the forward detection threshold.
 * @param echo_us Samples in microseconds; zero marks no echo. May be NULL
 *        only when count is zero.
 * @param count Number of samples; values above OBSTACLE_DECISION_MEDIAN_MAX
 *        examine only the first OBSTACLE_DECISION_MEDIAN_MAX samples.
 * @return Middle nonzero sample; the average of two nonzero samples; zero
 *         when no sample is nonzero.
 */
uint16_t obstacle_decision_median (uint16_t const * echo_us, uint8_t count);

/**
 * @brief Classify one position after its pings complete.
 * @param median_us Median from obstacle_decision_median(), in microseconds.
 * @param b_invalid True when a fault exhausted that position's retries.
 * @return ULTRASONIC_FAULT when invalid, ULTRASONIC_NO_ECHO for a zero
 *         median, otherwise ULTRASONIC_VALID.
 */
ultrasonic_result_t obstacle_decision_position (uint16_t median_us,
                                                bool b_invalid);

/**
 * @brief Decide the next step from the forward (90 degree) position.
 * @param measurement Position result; anything other than VALID or NO_ECHO
 *        is treated as a fault.
 * @param echo_us Median echo in microseconds, used only when VALID.
 * @return CLEAR for NO_ECHO or an echo at or beyond
 *         OBSTACLE_DETECT_ECHO_US; SCAN_SIDES for a nearer echo; UNKNOWN
 *         for a fault.
 */
obstacle_centre_action_t
obstacle_decision_centre (ultrasonic_result_t measurement, uint16_t echo_us);

/**
 * @brief Choose a direction once both side positions have completed.
 *
 * A side is clear for NO_ECHO or a VALID echo at or beyond
 * OBSTACLE_DETECT_ECHO_US. NO_ECHO ranks beyond any finite echo, the farther
 * clear side wins and ties select left. With no clear side the result is
 * UNKNOWN if either side faulted, otherwise BLOCKED.
 * @param left Left (60 degree) position result.
 * @param left_echo_us Left median echo in microseconds.
 * @param right Right (120 degree) position result.
 * @param right_echo_us Right median echo in microseconds.
 * @return PATH_FOUND with LEFT_60 or RIGHT_120, or BLOCKED/UNKNOWN with NONE.
 */
obstacle_side_decision_t obstacle_decision_sides (ultrasonic_result_t left,
                                                  uint16_t left_echo_us,
                                                  ultrasonic_result_t right,
                                                  uint16_t right_echo_us);

#endif /* OBSTACLE_DECISION_H */
