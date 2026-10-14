/**
 * @file line_follow.h
 * @brief Line-following decisions using two IR sensors.
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "line_sensor.h"

/* Public types, constants, and macros. */

typedef enum
{
    LINE_COMMAND_SEARCH = 0,
    LINE_COMMAND_FORWARD,
    LINE_COMMAND_STEER_LEFT,
    LINE_COMMAND_STEER_RIGHT,
    LINE_COMMAND_JUNCTION
} line_command_t;

/* Public function declarations. */

/**
 * @brief Reset the module to SEARCHING state.
 * @note State belongs to one sensor pair; calls must be serialized.
 */
void line_follow_init (void);

/**
 * @brief Convert IR sensor readings into a navigation decision.
 *
 * @pre Call line_follow_init() before the first update or to restart seeking.
 * @param reading Left and right ADC counts, each in the range 0 to 4095.
 * @note Counts >= 700 mean BLACK; lower counts mean WHITE. Any BLACK
 *       enters FOLLOWING. Left-only BLACK steers left; right-only BLACK
 *       steers right; both BLACK returns JUNCTION without confirmation.
 *       Both WHITE returns SEARCH in SEARCHING, or FORWARD in FOLLOWING.
 *       FOLLOWING assumes the line is between the spaced sensors; it cannot
 *       distinguish that condition from a lost line. Only init resets state.
 *       No motors are controlled.
 * @return Navigation decision for the motion controller.
 */
line_command_t line_follow_update (line_sensor_reading_t reading);

/**
 * @brief Get a printable name for a line command.
 * @param command Decision to name.
 * @return Static, immutable string; UNKNOWN for an unrecognized command.
 *         The caller must not free the returned pointer.
 */
char const * line_follow_command_name (line_command_t command);

#endif /* LINE_FOLLOW_H */
