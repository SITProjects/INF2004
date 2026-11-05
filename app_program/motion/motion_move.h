/**
 * @file motion_move.h
 * @brief Odometry, wheel speeds, distance moves, turns, line following and
 * stall detection (plan steps 3 to 6b).
 *
 * A move drives both wheels at MOTION_MOVE_SPEED_MM_S, and a turn spins
 * the car in place at MOTION_TURN_SPEED_DEG_S, through
 * motion_drive_set_velocity(); each brakes once the encoders show the
 * distance or angle, or, if asked, once the line comes into view. Every
 * update feeds the measured wheel speeds to the drive's speed loop
 * (motion_drive_control()), so speeds hold under load.
 * During a move, a straight-line (sync) PI loop also trims a small turn
 * rate into the wheel setpoints to hold the heading the move started on.
 * In follow mode a heading PI loop instead holds the target heading the
 * owner passes in with motion_move_set_follow_heading(), so the car turns
 * by each step while driving and then runs straight; this module never
 * reads the line sensors itself. motion_move_set_line() passes in only
 * whether the line is seen, for stop-at-line and for the freshness of
 * following.
 * The module is RTOS-free; its owner calls
 * motion_move_update() every MOTION_UPDATE_PERIOD_MS. All functions must
 * be called from that one task, because reading the encoders empties their
 * FIFOs.
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

/** @brief Motion mode. Values fit the 3-bit mode field (Tribunal 1). */
typedef enum
{
    MOTION_STATE_STOPPED = 0, /**< Braked, no command. */
    MOTION_STATE_MOVE,        /**< Distance move in progress. */
    MOTION_STATE_STALLED,     /**< Braked after a stall; needs a stop. */
    MOTION_STATE_TURN,        /**< Turn in place in progress. */
    MOTION_STATE_TIMED_OUT,   /**< Braked, target not reached in time. */
    MOTION_STATE_FOLLOW       /**< Following the line, no end point. */
} motion_state_t;

/**
 * @brief Why the last move or turn ended, for MOVE_DONE (design review
 * 6.4: DIST or LINE).
 */
typedef enum
{
    MOTION_DONE_NONE = 0, /**< Not ended by itself: in progress, stopped,
                               stalled, timed out, or following. */
    MOTION_DONE_TARGET,   /**< Its distance or angle was reached (DIST). */
    MOTION_DONE_LINE      /**< Ended early as the line came into view. */
} motion_done_t;

/**
 * @brief Chassis odometry since motion_move_init(). Speeds are averaged
 * over the last MOTION_SPEED_WINDOW_UPDATES updates.
 */
typedef struct
{
    int32_t distance_mm;      /**< Mean of both wheels' travel; + forward. */
    int16_t speed_mm_s;       /**< Mean of the two wheel speeds. */
    int16_t left_speed_mm_s;  /**< Left wheel speed; + is forward. */
    int16_t right_speed_mm_s; /**< Right wheel speed; + is forward. */
    int32_t heading_deg;      /**< Total turn, not wrapped; + is left. */
} motion_odometry_t;

/**
 * @brief Straight-line (sync) loop of the current move, as of the last
 * motion_move_update(). Both are 0 when no move is in progress.
 */
typedef struct
{
    int32_t error_ticks;      /**< Right - left count change since the move
                                   started; + means the car turned left. */
    int16_t correction_deg_s; /**< Turn rate added to the setpoints; + is
                                   left. 0 while the loop is off. */
} motion_sync_t;

/**
 * @brief Line input and the follow loop, as of the last
 * motion_move_update().
 */
typedef struct
{
    int32_t target_heading_deg; /**< Heading following holds; odometry
                                     degrees, not wrapped, + is left. */
    int16_t heading_error_deg;  /**< Target - heading; + means the car
                                     must turn left. 0 when not following. */
    bool b_line_seen;           /**< Line in view, after debouncing. */
    int16_t turn_deg_s;         /**< Follow loop's turn rate; + is left. 0
                                     when not following. */
} motion_follow_t;

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
 * @brief Read the encoders, update odometry and wheel speeds, run one
 * straight-line step during a move or one line step while following, one
 * speed-loop step, and run the current command. Ends a move or turn once
 * its distance or angle is reached, or the line comes into view if
 * motion_move_stop_at_line() asked for that (state STOPPED). Ends
 * following (state STOPPED) when no line error has arrived for
 * MOTION_LINE_TIMEOUT_MS. It brakes and enters MOTION_STATE_STALLED when a
 * driven wheel shows no encoder change for MOTION_STALL_TIMEOUT_MS, or,
 * for a move or turn, MOTION_STATE_TIMED_OUT when it runs
 * MOTION_TIMEOUT_FACTOR times longer than expected.
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
 * @brief Drive forward at @p speed_mm_s holding a target heading until
 * replaced or stopped (FOLLOW mode). The target starts as the current
 * heading; motion_move_set_follow_heading() moves it. Each update steers
 * by the heading PI loop. The turn rate is limited to
 * MOTION_FOLLOW_MAX_DEG_S and to
 * 2 * @p speed_mm_s / MOTION_TRACK_WIDTH_MM (in deg/s), so the inner wheel
 * never reverses.
 * @param speed_mm_s Chassis speed in mm/s, 1 to MOTION_MAX_SPEED_MM_S.
 * @return MOTION_OK, MOTION_ERR_PARAM for 0, above the maximum, or no
 * line reading within MOTION_LINE_TIMEOUT_MS (nothing driven),
 * MOTION_ERR_STALLED or MOTION_ERR_TIMED_OUT (call motion_move_stop()
 * first), or MOTION_ERR_NOT_INIT.
 * Replaces a move or turn already in progress. Call
 * motion_move_set_line() first, then once per reading: following stops if
 * the last one is older than MOTION_LINE_TIMEOUT_MS. Losing the line does
 * not stop it; the caller decides when to search.
 */
motion_status_t motion_move_follow (uint16_t speed_mm_s);

/**
 * @brief Set the heading following holds.
 * @param target_deg Odometry heading (motion_odometry_t.heading_deg, not
 * wrapped, + is left). The loop turns towards it at up to the follow turn
 * limit while driving, then holds it.
 * @return MOTION_OK, MOTION_ERR_PARAM if not following, or
 * MOTION_ERR_NOT_INIT.
 * Call after motion_move_follow(), which resets the target to the current
 * heading.
 */
motion_status_t motion_move_set_follow_heading (int32_t target_deg);

/**
 * @brief Pass in the latest line reading, for following and for
 * motion_move_stop_at_line().
 * @param b_seen True when either line sensor reads black. It counts as
 * seen, or as gone, only after MOTION_LINE_SEEN_SAMPLES calls in a row
 * agree.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Call from the owner task (as every function here) once per new sensor
 * reading, in every state, before motion_move_update(). Following stops
 * once no reading has arrived for MOTION_LINE_TIMEOUT_MS.
 */
motion_status_t motion_move_set_line (bool b_seen);

/**
 * @brief End the move or turn in progress early when the line comes into
 * view (the stop-at-line flag of MBF_MOTION_CMD).
 * @return MOTION_OK, MOTION_ERR_PARAM if no move or turn is in progress,
 * or MOTION_ERR_NOT_INIT.
 * Call right after starting the move or turn; it lasts until that one
 * ends. Only a change from not seen to seen ends it, so a turn that
 * starts on the line first turns off it. A move or turn that never sees
 * the line ends as usual at its distance or angle.
 */
motion_status_t motion_move_stop_at_line (void);

/**
 * @brief End the current move/turn at a fresh, confirmed visible line.
 * @return As motion_move_stop_at_line(). Only the motion owner may call.
 * A line already visible when armed also counts; the next update brakes
 * and reports MOTION_DONE_LINE. Stale visibility never counts. If no line
 * is seen, the command ends at its distance/angle as usual. The setting
 * lasts for this command only. Used by queued stop-at-line commands for
 * obstacle reacquisition; the lower-level edge-only API stays available.
 */
motion_status_t motion_move_acquire_line (void);

/**
 * @brief Brake both wheels, end any move, turn or following, and clear a
 * stall or timeout.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Leaves the state MOTION_STATE_STOPPED with the move and stall timers
 * reset; odometry keeps counting.
 */
motion_status_t motion_move_stop (void);

/** @brief True while a move, turn or following is in progress. */
bool motion_move_is_busy (void);

/**
 * @brief Why the last move or turn ended.
 * @return MOTION_DONE_TARGET or MOTION_DONE_LINE once a move or turn has
 * ended by itself (state STOPPED), until the next command; otherwise
 * MOTION_DONE_NONE.
 */
motion_done_t motion_move_get_done_reason (void);

/** @brief Current motion mode. */
motion_state_t motion_move_get_state (void);

/**
 * @brief Read the odometry as of the last motion_move_update().
 * @param p_odometry Caller-owned snapshot to fill.
 * @return MOTION_OK, MOTION_ERR_PARAM for a NULL pointer, or
 * MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_move_get_odometry (motion_odometry_t * p_odometry);

/**
 * @brief Turn the straight-line (sync) loop of moves on or off; it is on
 * after motion_move_init(). For comparing runs with and without it.
 * @param b_enable True to correct the heading during moves.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Clears the loop's integral. During a move, turning it off returns both
 * wheels to the plain move speed at once; turning it on corrects the
 * heading change since the move started.
 */
motion_status_t motion_move_set_sync (bool b_enable);

/**
 * @brief Read the straight-line loop's error and correction.
 * @param p_sync Caller-owned snapshot to fill.
 * @return MOTION_OK, MOTION_ERR_PARAM for a NULL pointer, or
 * MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_move_get_sync (motion_sync_t * p_sync);

/**
 * @brief Read the line input and the follow loop's target and turn rate.
 * @param p_follow Caller-owned snapshot to fill.
 * @return MOTION_OK, MOTION_ERR_PARAM for a NULL pointer, or
 * MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_move_get_follow (motion_follow_t * p_follow);

#endif /* MOTION_MOVE_H */
