/**
 * @file line_command.h
 * @brief Line-following decisions using two IR sensors.
 */

#ifndef LINE_COMMAND_H
#define LINE_COMMAND_H

#include "drv_ir_sensor.h"

/* Public types, constants, and macros. */

/** Provisional hand-measured ADC bands; revalidate at fixed 6-7 mm height. */
#define LINE_LEFT_EDGE_MIN 700u
#define LINE_LEFT_EDGE_MAX 900u
#define LINE_RIGHT_EDGE_MIN 1100u
#define LINE_RIGHT_EDGE_MAX 1200u
/** Lower bounds of the measured full-black ranges, in ADC counts. */
#define LINE_LEFT_BLACK_MIN 1300u
#define LINE_RIGHT_BLACK_MIN 3200u

/**
 * @brief Navigation requests; this module does not operate the motors.
 */
typedef enum
{
    /** Both readings below their edge bands; request acquisition. */
    LINE_COMMAND_SEARCH = 0,
    /** Both readings inside their inclusive edge bands; on course. */
    LINE_COMMAND_FORWARD,
    /** Left above its band or right below its band; steer left. */
    LINE_COMMAND_STEER_LEFT,
    /** Left below its band or right above its band; steer right. */
    LINE_COMMAND_STEER_RIGHT,
    /** Sustained both BLACK; junction candidate, not confirmed geometry. */
    LINE_COMMAND_JUNCTION,
    /** Both above their bands without confirmed full BLACK; stop. */
    LINE_COMMAND_STOP
} line_command_t;

/* Public function declarations. */

/**
 * @brief Clear consecutive full-black junction confirmation.
 * @note State belongs to one sensor pair; calls must be serialized.
 */
void line_command_init (void);

/**
 * @brief Compare two raw readings with their measured edge bands.
 * @pre Call line_command_init() before the first update.
 * @param reading Left and right ADC counts, each in the range 0 to 4095.
 * @note Assumes the left sensor straddles the left boundary and the right
 *       sensor straddles the right boundary of the same 18 mm black line.
 *       Both inside their inclusive bands returns FORWARD, including at
 *       startup. Both below returns SEARCH, including after tracking.
 *       Both above returns STOP, or JUNCTION after three consecutive pairs
 *       at or above both full-black minimums. Other pairs steer left when
 *       left is above or right is below; otherwise they steer right.
 *       A non-full-black pair resets junction confirmation. JUNCTION is a
 *       candidate, not proof of junction geometry. SEARCH requests acquisition
 *       only; autonomous search and recovery motion are not implemented.
 *       These provisional readings require validation on the actual robot;
 *       an edge-band pair alone does not prove robot position or heading.
 *       Call once per fresh pair at a regular interval; three samples span
 *       two intervals (about 40 ms at the nominal 20 ms cadence).
 *       Commands, including STOP, are requests only; no motors are controlled.
 * @return Requested navigation command.
 */
line_command_t line_command_update (ir_sensor_reading_t reading);

/**
 * @brief Get a printable name for a line command.
 * @param command Decision to name.
 * @return Static, immutable string; UNKNOWN for an unrecognized command.
 *         The caller must not free the returned pointer.
 */
char const * line_command_name (line_command_t command);

#endif /* LINE_COMMAND_H */
