/**
 * @file obstacle_decision.c
 * @brief Hardware-free obstacle decisions shared by the scanner and tests.
 */

#include "obstacle_decision.h"
#include "obstacle_config.h"
#include <stddef.h>

/* Private types, constants, and macros. */

/* A no-echo side ranks above every 16-bit finite echo. */
#define NO_ECHO_RANK ((uint32_t)UINT16_MAX + 1u)

_Static_assert(OBSTACLE_PINGS_PER_ANGLE <= OBSTACLE_DECISION_MEDIAN_MAX,
               "Ping count exceeds the median helper's sample limit");

/* Static data. */

/* Private function declarations. */

static bool is_fault (ultrasonic_result_t measurement);
static uint32_t clear_rank (ultrasonic_result_t measurement, uint16_t echo_us);

/* Public function definitions. */

uint16_t
obstacle_decision_median (uint16_t const * echo_us, uint8_t count)
{
    uint16_t valid_us[OBSTACLE_DECISION_MEDIAN_MAX];
    uint8_t valid_count = 0u;
    uint8_t limit = (count > OBSTACLE_DECISION_MEDIAN_MAX)
                        ? (uint8_t)OBSTACLE_DECISION_MEDIAN_MAX
                        : count;

    if ((echo_us == NULL) || (limit == 0u))
    {
        return 0u;
    }

    /* Insertion sort of the nonzero samples. */
    for (uint8_t index = 0u; index < limit; ++index)
    {
        if (echo_us[index] != 0u)
        {
            uint8_t insert = valid_count;
            while ((insert > 0u) && (valid_us[insert - 1u] > echo_us[index]))
            {
                valid_us[insert] = valid_us[insert - 1u];
                --insert;
            }
            valid_us[insert] = echo_us[index];
            ++valid_count;
        }
    }

    if (valid_count == 0u)
    {
        return 0u;
    }
    if (valid_count == 2u)
    {
        return (uint16_t)(((uint32_t)valid_us[0] + valid_us[1]) / 2u);
    }
    return valid_us[valid_count / 2u];
}

ultrasonic_result_t
obstacle_decision_position (uint16_t median_us, bool b_invalid)
{
    if (b_invalid)
    {
        return ULTRASONIC_FAULT;
    }
    return (median_us == 0u) ? ULTRASONIC_NO_ECHO : ULTRASONIC_VALID;
}

obstacle_centre_action_t
obstacle_decision_centre (ultrasonic_result_t measurement, uint16_t echo_us)
{
    if (is_fault(measurement))
    {
        return OBSTACLE_CENTRE_UNKNOWN;
    }
    if ((measurement == ULTRASONIC_NO_ECHO) ||
        (echo_us >= OBSTACLE_DETECT_ECHO_US))
    {
        return OBSTACLE_CENTRE_CLEAR;
    }
    return OBSTACLE_CENTRE_SCAN_SIDES;
}

obstacle_side_decision_t
obstacle_decision_sides (ultrasonic_result_t left, uint16_t left_echo_us,
                         ultrasonic_result_t right, uint16_t right_echo_us)
{
    obstacle_side_decision_t decision = {.status = ROBOT_OBSTACLE_BLOCKED,
                                         .direction = ROBOT_OBSTACLE_DIR_NONE};
    uint32_t left_rank = clear_rank(left, left_echo_us);
    uint32_t right_rank = clear_rank(right, right_echo_us);

    if ((left_rank == 0u) && (right_rank == 0u))
    {
        if (is_fault(left) || is_fault(right))
        {
            decision.status = ROBOT_OBSTACLE_UNKNOWN;
        }
        return decision;
    }

    /* Strictly farther wins, so equal ranks keep the left side. */
    decision.status = ROBOT_OBSTACLE_PATH_FOUND;
    decision.direction = (right_rank > left_rank) ? ROBOT_OBSTACLE_DIR_RIGHT_120
                                                  : ROBOT_OBSTACLE_DIR_LEFT_60;
    return decision;
}

/* Private function definitions. */

/* Only VALID and NO_ECHO are usable results; all others count as faults. */
static bool
is_fault (ultrasonic_result_t measurement)
{
    return (measurement != ULTRASONIC_VALID) &&
           (measurement != ULTRASONIC_NO_ECHO);
}

/* Zero means not clear; larger values mean more clearance. */
static uint32_t
clear_rank (ultrasonic_result_t measurement, uint16_t echo_us)
{
    if (measurement == ULTRASONIC_NO_ECHO)
    {
        return NO_ECHO_RANK;
    }
    if ((measurement == ULTRASONIC_VALID) &&
        (echo_us >= OBSTACLE_DETECT_ECHO_US))
    {
        return echo_us;
    }
    return 0u;
}
