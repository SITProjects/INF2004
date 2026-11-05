/**
 * @file motion_command_config.h
 * @brief Continuous line-following adapter settings.
 */

#ifndef MOTION_COMMAND_CONFIG_H
#define MOTION_COMMAND_CONFIG_H

#include "motion_config.h"

/** Normal builds drive the motors. APP_TEST always disables adapter actuation.
 * Set this to 0 for a dry run. tMotion owns the motors in every profile.
 * IR_MOTOR_TEST isolates line and motion tasks
 * and adds stop traces; it uses the same continuous settings as normal mode.
 */
#ifndef MOTION_COMMAND_ACTUATION_ENABLED
#define MOTION_COMMAND_ACTUATION_ENABLED 1
#endif

/** Unitless corrections: KP=0.3 gives 15 to 129 deg/s at 120 mm/s, capped
 * at MOTION_FOLLOW_MAX_DEG_S; full steering holds the tightest test curves.
 * Per-sensor calibration ramps from first detection to full black.
 */
#define MOTION_COMMAND_EDGE_ERROR 50
#define MOTION_COMMAND_STEER_ERROR 430
/** Small remembered bias between detections (9 deg/s before damping). */
#define MOTION_COMMAND_PROBE_ERROR 30
/** Search at 80 mm/s and 51 deg/s towards the last edge; no fail timer. */
#define MOTION_COMMAND_SEARCH_ERROR 170
#define MOTION_COMMAND_SEARCH_SPEED_MM_S 80u
#define MOTION_COMMAND_FOLLOW_SPEED_MM_S MOTION_FOLLOW_SPEED_MM_S
/** Preparation delay before continuous operation; there is no run cutoff. */
#define MOTION_COMMAND_START_DELAY_MS 5000u

#endif /* MOTION_COMMAND_CONFIG_H */
