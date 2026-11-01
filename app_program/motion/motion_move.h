/**
 * @file motion_move.h
 * @brief Odometry, distance moves and stall detection (plan step 3).
 *
 * Open loop: a move drives both wheels at MOTION_MOVE_SPEED_MM_S through
 * motion_drive_set_velocity() and brakes once the encoders show the
 * distance. The module is RTOS-free; its owner calls motion_move_update()
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
 * @brief Motion mode. Values fit the 3-bit mode field (Tribunal 1); turn
 * and follow modes join in later plan steps.
 */
typedef enum
{
    MOTION_STATE_STOPPED = 0, /**< Braked, no command. */
    MOTION_STATE_MOVE,        /**< Distance move in progress. */
    MOTION_STATE_STALLED      /**< Braked after a stall; needs a stop. */
} motion_state_t;

/** @brief Chassis odometry since motion_move_init(). */
typedef struct
{
    int32_t distance_mm; /**< Mean of both wheels' travel; + is forward. */
    int16_t speed_mm_s;  /**< Mean wheel speed over the last update. */
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
 * @brief Read the encoders, update odometry, and run the current move.
 * Ends a move once its distance is reached (state STOPPED), and brakes and
 * enters MOTION_STATE_STALLED when a wheel shows no encoder change for
 * MOTION_STALL_TIMEOUT_MS during a move.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Call every MOTION_UPDATE_PERIOD_MS: speeds and the stall timeout assume
 * that period.
 */
motion_status_t motion_move_update (void);

/**
 * @brief Drive forward @p distance_mm, then brake.
 * @param distance_mm Distance in mm, 1 or more.
 * @return MOTION_OK, MOTION_ERR_PARAM for 0, MOTION_ERR_STALLED while
 * stalled (call motion_move_stop() first), or MOTION_ERR_NOT_INIT.
 * Replaces a move already in progress, measuring from the current place.
 */
motion_status_t motion_move_forward (uint16_t distance_mm);

/**
 * @brief Drive backward @p distance_mm, then brake.
 * @param distance_mm Distance in mm, 1 or more.
 * @return As motion_move_forward().
 */
motion_status_t motion_move_backward (uint16_t distance_mm);

/**
 * @brief Brake both wheels, end any move, and clear a stall.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Leaves the state MOTION_STATE_STOPPED with the move and stall timers
 * reset; odometry keeps counting.
 */
motion_status_t motion_move_stop (void);

/** @brief True while a move is in progress. */
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
