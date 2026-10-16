/**
 * @file line_follow.h
 * @brief Line-following decisions using two IR sensors.
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "ir_sensor.h"

/* Public types, constants, and macros. */

/**
 * @brief Navigation requests; this module does not operate the motors.
 */
typedef enum
{
    /** Both WHITE with alignment unknown; request line acquisition. */
    LINE_COMMAND_SEARCH = 0,
    /** Both WHITE after the caller has established line alignment. */
    LINE_COMMAND_FORWARD,
    /** Left BLACK and right WHITE; request steering left. */
    LINE_COMMAND_STEER_LEFT,
    /** Left WHITE and right BLACK; request steering right. */
    LINE_COMMAND_STEER_RIGHT,
    /** Sustained both BLACK; junction candidate, not confirmed geometry. */
    LINE_COMMAND_JUNCTION,
    /** Both BLACK awaiting consecutive-sample confirmation; request stop. */
    LINE_COMMAND_STOP
} line_command_t;

/* Public function declarations. */

/**
 * @brief Reset line alignment to unknown and clear junction confirmation.
 * @note Both WHITE returns SEARCH until line_follow_start_following() is
 *       called. Sensor readings alone never establish alignment.
 *       State belongs to one sensor pair; calls must be serialized.
 */
void line_follow_init (void);

/**
 * @brief Enter FOLLOWING after the caller has established line alignment.
 * @pre Call line_follow_init() first; serialize with all other module calls.
 * @note This records the caller's decision; it does not detect alignment.
 *       A future acquisition controller must decide when to call it. Do not
 *       call solely because one or more sensor samples contain BLACK.
 *       Repeated calls are harmless and preserve junction confirmation.
 */
void line_follow_start_following (void);

/**
 * @brief Convert IR sensor readings into a navigation decision.
 *
 * @pre Call line_follow_init() before the first update or to restart seeking.
 * @param reading Left and right ADC counts, each in the range 0 to 4095.
 * @note Counts >= 700 mean BLACK; lower counts mean WHITE. Left-only BLACK
 *       steers left and right-only BLACK steers right in either state.
 *       Sensor readings never switch UNKNOWN to FOLLOWING automatically.
 *       Both BLACK returns STOP for the first two consecutive samples, then
 *       JUNCTION while both remain BLACK. Any non-BB sample resets this count.
 *       JUNCTION indicates sustained black coverage, not confirmed geometry.
 *       Both WHITE returns SEARCH in UNKNOWN, or FORWARD in FOLLOWING.
 *       SEARCH requests acquisition; this module implements no search motion.
 *       FOLLOWING records caller-established alignment; both WHITE can still
 *       mean centred or lost. Call init to reset the state to UNKNOWN.
 *       Call once per fresh reading pair at a regular sampling interval.
 *       Three BB samples span two intervals (about 40 ms at 20 ms cadence);
 *       confirmation counts require validation at the actual control rate.
 *       Commands, including STOP, are requests only; no motors are controlled.
 * @return Navigation decision for the motion controller.
 */
line_command_t line_follow_update (ir_sensor_reading_t reading);

/**
 * @brief Get a printable name for a line command.
 * @param command Decision to name.
 * @return Static, immutable string; UNKNOWN for an unrecognized command.
 *         The caller must not free the returned pointer.
 */
char const * line_follow_command_name (line_command_t command);

#endif /* LINE_FOLLOW_H */
