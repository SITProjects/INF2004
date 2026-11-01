/**
 * @file motion_move.h
 * @brief Odometry, distance moves, turns and stall detection (plan steps
 * 3 and 4).
 *
 * Open loop: a move drives both wheels at MOTION_MOVE_SPEED_MM_S, and a
 * turn spins the car in place at MOTION_TURN_SPEED_DEG_S, through
 * motion_drive_set_velocity(); each brakes once the encoders show the
 * distance or angle. The module is RTOS-free; its owner calls motion_move_update()
 * every MOTION_UPDATE_PERIOD_MS. All functions must be called from that
 * one task, because reading the encoders empties their FIFOs.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef MOTION_MOVE_H
#define MOTION_MOVE_H

/* Public includes. */
#include <stdint.h>
#include <stdbool.h>
#include "motion_drive.h"

/* Public types, constants, and macros. */

/**
 * @brief Motion mode. Values fit the 3-bit mode field (Tribunal 1); the
 * follow mode joins in a later plan step.
 */
typedef enum
{
    MOTION_STATE_STOPPED = 0, /**< Braked, no command. */
    MOTION_STATE_MOVE,        /**< Distance move in progress. */
    MOTION_STATE_STALLED,     /**< Braked after a stall; needs a stop. */
    MOTION_STATE_TURN,        /**< Turn in place in progress. */
    MOTION_STATE_TIMED_OUT    /**< Braked, target not reached in time. */
} motion_state_t;

/** @brief Chassis odometry since motion_move_init(). */
typedef struct
{
    int32_t distance_mm; /**< Mean of both wheels' travel; + is forward. */
    int16_t speed_mm_s;  /**< Mean wheel speed over the last update. */
    int32_t heading_deg; /**< Total turn, not wrapped; + is left (CCW). */
} motion_odometry_t;

/* Public function declarations. */

/**
 * @brief Start odometry from zero in MOTION_STATE_STOPPED.
 * @return MOTION_OK, or MOTION_ERR_NOT_INIT if motion_drive_init() has not
 * succeeded.
 * Call once, after motion_drive_init(), from the task that will call
 * motion_move_update().
 */
motion_status_t motion_move_init (void);

/**
 * @brief Read the encoders, update odometry, and run the current move or
 * turn. Ends it once its distance or angle is reached (state STOPPED). It
 * brakes and enters MOTION_STATE_STALLED when a wheel shows no encoder
 * change for MOTION_STALL_TIMEOUT_MS during it, or MOTION_STATE_TIMED_OUT
 * when it runs MOTION_TIMEOUT_FACTOR times longer than expected.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Call every MOTION_UPDATE_PERIOD_MS: speeds and the stall timeout assume
 * that period.
 */
motion_status_t motion_move_update (void);

/**
 * @brief Drive forward @p distance_mm, then brake.
 * @param distance_mm Distance in mm, 1 or more.
 * @return MOTION_OK, MOTION_ERR_PARAM for 0, MOTION_ERR_STALLED while
 * stalled or MOTION_ERR_TIMED_OUT after a timeout (call motion_move_stop()
 * first), or MOTION_ERR_NOT_INIT.
 * Replaces a move or turn already in progress, measuring from the current
 * place.
 */
motion_status_t motion_move_forward (uint16_t distance_mm);

/**
 * @brief Drive backward @p distance_mm, then brake.
 * @param distance_mm Distance in mm, 1 or more.
 * @return As motion_move_forward().
 */
motion_status_t motion_move_backward (uint16_t distance_mm);

/**
 * @brief Turn left (counter-clockwise) in place by @p angle_deg, then brake.
 * @param angle_deg Angle in degrees, 1 to MOTION_MAX_TURN_DEG; above 180
 * is for calibration (MBF_MOTION_CMD itself carries only 0-180).
 * @return MOTION_OK, MOTION_ERR_PARAM for 0 or above MOTION_MAX_TURN_DEG,
 * MOTION_ERR_STALLED or MOTION_ERR_TIMED_OUT (call motion_move_stop()
 * first), or MOTION_ERR_NOT_INIT.
 * Replaces a move or turn already in progress, measuring from the current
 * heading. The angle is right, but a turn that replaces a fast move pivots
 * about a point shifted ahead while the car's momentum dies away.
 */
motion_status_t motion_move_turn_left (uint16_t angle_deg);

/**
 * @brief Turn right (clockwise) in place by @p angle_deg, then brake.
 * @param angle_deg Angle in degrees, 1 or more.
 * @return As motion_move_turn_left().
 */
motion_status_t motion_move_turn_right (uint16_t angle_deg);

/**
 * @brief Turn 180 degrees in place (to the left), then brake.
 * @return As motion_move_turn_left().
 */
motion_status_t motion_move_u_turn (void);

/**
 * @brief Brake both wheels, end any move or turn, and clear a stall or
 * timeout.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Leaves the state MOTION_STATE_STOPPED with the move and stall timers
 * reset; odometry keeps counting.
 */
motion_status_t motion_move_stop (void);

/** @brief True while a move or turn is in progress. */
bool motion_move_is_busy (void);

/** @brief Current motion mode. */
motion_state_t motion_move_get_state (void);

/**
 * @brief Read the odometry as of the last motion_move_update().
 * @param p_odometry Caller-owned snapshot to fill.
 * @return MOTION_OK, MOTION_ERR_PARAM for a NULL pointer, or
 * MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_move_get_odometry (motion_odometry_t * p_odometry);

#endif /* MOTION_MOVE_H */
