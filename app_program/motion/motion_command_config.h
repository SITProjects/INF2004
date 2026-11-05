/**
 * @file motion_command_config.h
 * @brief Line-following adapter settings.
 */

#ifndef MOTION_COMMAND_CONFIG_H
#define MOTION_COMMAND_CONFIG_H

#include "motion_config.h"

/** Normal builds drive the motors. APP_TEST always disables adapter actuation.
 * Set this to 0 for a dry run. tMotion owns the motors in every profile.
 * IR_MOTOR_TEST only isolates the line and motion tasks.
 */
#ifndef MOTION_COMMAND_ACTUATION_ENABLED
#define MOTION_COMMAND_ACTUATION_ENABLED 1
#endif

/** Constant tracking speed, so barcodes are read at a steady speed. */
#define MOTION_COMMAND_FOLLOW_SPEED_MM_S MOTION_FOLLOW_SPEED_MM_S
/** Preparation delay before continuous operation; there is no run cutoff. */
#define MOTION_COMMAND_START_DELAY_MS 5000u
/** A stall is cleared, and following retried, after this long. */
#define MOTION_COMMAND_STALL_RETRY_MS 500u

#endif /* MOTION_COMMAND_CONFIG_H */
