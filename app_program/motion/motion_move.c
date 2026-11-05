/**
 * @file motion_move.c
 * @brief Odometry, wheel speeds, distance moves, turns and stall detection.
 *
 * Odometry adds the wrap-safe 16-bit encoder change of each update to
 * this module's own 32-bit running totals (Tribunal 1), so the raw 16-bit
 * counts never need to be compared directly. A move is measured by the
 * sum of both wheels' totals, a turn by their difference (right - left),
 * which is the heading change times the track width. Each update also
 * measures each wheel's speed over the last MOTION_SPEED_WINDOW_UPDATES
 * updates and passes both speeds to the drive's speed loop. During a move,
 * a straight-line PI loop first turns the change in that difference since
 * the move started (a heading error) into a turn rate for
 * motion_drive_set_velocity(), so the speed loops steer the car back.
 * While following, a line PD loop does the same from the line error the
 * owner passes in. The owner's line readings are debounced here into a
 * seen/gone state; a move or turn asked to stop at the line ends when it
 * changes from gone to seen.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "motion_move.h"
#include <stddef.h>
#include "motion_config.h"
#include "motion_pid.h"

/* Private types, constants, and macros. */

/* Wheel travel per encoder count, in mm. */
#define MM_PER_TICK                                                            \
    ((3.14159265f * MOTION_WHEEL_DIAMETER_MM) /                                \
     (float)MOTION_ENCODER_TICKS_PER_REV)

/* Multiply degrees by this to get radians, and radians by its inverse. */
#define DEG_TO_RAD (3.14159265f / 180.0f)

/* Updates without an encoder change, during a move, that mean a stall. */
#define STALL_UPDATES (MOTION_STALL_TIMEOUT_MS / MOTION_UPDATE_PERIOD_MS)

/* Time between motion_move_update() calls, in seconds. */
#define UPDATE_PERIOD_S ((float)MOTION_UPDATE_PERIOD_MS / 1000.0f)

/*
 * Slowest wheel setpoint the stall check applies to, in mm/s: it should
 * give at least two counts within the stall timeout. A slower wheel (the
 * inner wheel of a tight follow curve) may rightly show no change.
 */
#define STALL_MIN_SPEED_MM_S                                                   \
    ((2.0f * MM_PER_TICK * 1000.0f) / (float)MOTION_STALL_TIMEOUT_MS)

/* Updates without a new line error after which following stops. */
#define LINE_TIMEOUT_UPDATES (MOTION_LINE_TIMEOUT_MS / MOTION_UPDATE_PERIOD_MS)

typedef enum
{
    WHEEL_LEFT = 0,
    WHEEL_RIGHT,
    WHEEL_COUNT
} wheel_t;

/* Static data. */
static bool b_initialised = false;
static motion_state_t state = MOTION_STATE_STOPPED;

/* Last raw counts, and running totals since motion_move_init(). */
static motion_counts_t last_counts;
static int32_t total_ticks[WHEEL_COUNT];

/*
 * Each wheel's count changes over the last MOTION_SPEED_WINDOW_UPDATES
 * updates, their sum, and the speed that sum gives.
 */
static int16_t window_deltas[WHEEL_COUNT][MOTION_SPEED_WINDOW_UPDATES];
static int32_t window_sum[WHEEL_COUNT];
static uint32_t window_index = 0u;
static int16_t wheel_speed_mm_s[WHEEL_COUNT];

/*
 * Current move or turn: start, direction and target, in the sum of both
 * totals for a move or their difference (right - left) for a turn.
 */
static int32_t move_start = 0;
static int32_t move_dir = 1;
static int32_t move_target = 0;
static uint32_t still_updates[WHEEL_COUNT];

/* Updates the current move or turn has run, and its timeout in updates. */
static uint32_t move_updates = 0u;
static uint32_t move_limit_updates = 0u;

/*
 * Straight-line loop: whether it corrects, the right - left difference
 * the current move started at, and its last error and correction.
 */
static bool b_sync = true;
static motion_pid_t sync_pid;
static int32_t heading_start = 0;
static int32_t sync_right_start = 0;
static int32_t sync_error_ticks = 0;
static int16_t sync_correction_deg_s = 0;

/*
 * Why the last move or turn ended, and whether the current one ends at
 * the line.
 */
static motion_done_t done_reason = MOTION_DONE_NONE;
static bool b_stop_at_line = false;
static bool b_acquire_line = false;

/*
 * Line input: last error, updates since it arrived, the debounced seen
 * state, readings in a row that disagree with it, and the state at the
 * last update (to spot the line coming into view).
 */
static int16_t line_error = 0;
static uint32_t line_age_updates = 0u;
static bool b_line_seen = false;
static uint32_t line_change_samples = 0u;
static bool b_line_seen_last = false;

/*
 * Line following: chassis speed, PD loop, its last turn rate, and updates
 * since its last step (the step's dt).
 */
static uint16_t follow_speed_mm_s = 0u;
static motion_pid_t follow_pid;
static int16_t follow_turn_deg_s = 0;
static uint32_t follow_step_updates = 0u;

/* Private function declarations. */
static motion_status_t start_move (uint16_t distance_mm, int32_t dir);
static motion_status_t start_turn (uint16_t angle_deg, int32_t dir);
static motion_status_t check_can_start (uint16_t amount, uint16_t maximum);
static void begin (motion_state_t new_state, int32_t start, int32_t dir,
                   int32_t target, uint32_t expected_ms);
static int32_t tick_sum (void);
static int32_t tick_difference (void);
static void enter_stopped (void);
static bool wheel_stalled (wheel_t wheel, int16_t delta, int16_t setpoint);
static void update_speeds (int16_t delta_left, int16_t delta_right);
static void correct_heading (void);
static void reset_sync (void);
static void follow_line (void);
static void reset_follow (void);
static int16_t round_rate (float rate);

/* Public function definitions. */

motion_status_t
motion_move_init (void)
{
    if (motion_drive_get_counts(&last_counts) != MOTION_OK)
    {
        return MOTION_ERR_NOT_INIT;
    }

    for (uint32_t wheel = 0u; wheel < (uint32_t)WHEEL_COUNT; wheel++)
    {
        total_ticks[wheel] = 0;
        window_sum[wheel] = 0;
        wheel_speed_mm_s[wheel] = 0;
        for (uint32_t i = 0u; i < MOTION_SPEED_WINDOW_UPDATES; i++)
        {
            window_deltas[wheel][i] = 0;
        }
    }
    window_index = 0u;
    motion_pid_init(&sync_pid, MOTION_SYNC_KP, MOTION_SYNC_KI, 0.0f,
                    -MOTION_SYNC_MAX_DEG_S, MOTION_SYNC_MAX_DEG_S);
    motion_pid_init(&follow_pid, MOTION_FOLLOW_KP, 0.0f, MOTION_FOLLOW_KD,
                    -MOTION_FOLLOW_MAX_DEG_S, MOTION_FOLLOW_MAX_DEG_S);
    b_sync = true;

    /* No line reading yet: stale, and not seen. */
    line_error = 0;
    line_age_updates = LINE_TIMEOUT_UPDATES + 1u;
    b_line_seen = false;
    line_change_samples = 0u;
    b_line_seen_last = false;
    b_initialised = true;
    enter_stopped();
    return MOTION_OK;
}

motion_status_t
motion_move_update (void)
{
    motion_counts_t now;

    if (!b_initialised || (motion_drive_get_counts(&now) != MOTION_OK))
    {
        return MOTION_ERR_NOT_INIT;
    }

    /* Correct across a 16-bit wrap: far fewer than 32768 counts/update. */
    int16_t delta_left = (int16_t)(now.left_count - last_counts.left_count);
    int16_t delta_right = (int16_t)(now.right_count - last_counts.right_count);
    last_counts = now;
    total_ticks[WHEEL_LEFT] += delta_left;
    total_ticks[WHEEL_RIGHT] += delta_right;
    update_speeds(delta_left, delta_right);

    /* Age the line reading (saturating) and spot the line coming into view. */
    if (line_age_updates <= LINE_TIMEOUT_UPDATES)
    {
        line_age_updates++;
    }
    bool b_line_found = b_line_seen && (!b_line_seen_last || b_acquire_line) &&
                        (line_age_updates <= LINE_TIMEOUT_UPDATES);
    b_line_seen_last = b_line_seen;

    if (MOTION_STATE_MOVE == state)
    {
        correct_heading();
    }
    else if (MOTION_STATE_FOLLOW == state)
    {
        follow_step_updates++;
        if (line_age_updates > LINE_TIMEOUT_UPDATES)
        {
            enter_stopped();
        }
        else if (1u == line_age_updates)
        {
            /* A new reading since the last update: one line step. */
            follow_line();
        }
        else
        {
            /* No new reading: hold the turn rate, so D sees no step. */
        }
    }
    else
    {
        /* Not driving under a command: nothing to steer. */
    }
    (void)motion_drive_control(wheel_speed_mm_s[WHEEL_LEFT],
                               wheel_speed_mm_s[WHEEL_RIGHT]);

    if (!motion_move_is_busy())
    {
        return MOTION_OK;
    }

    /* Evaluate both wheels so each one's timer stays current. */
    motion_command_t command;
    (void)motion_drive_get_command(&command);
    bool b_left_stalled =
        wheel_stalled(WHEEL_LEFT, delta_left, command.left_setpoint_mm_s);
    bool b_right_stalled =
        wheel_stalled(WHEEL_RIGHT, delta_right, command.right_setpoint_mm_s);
    if (b_left_stalled || b_right_stalled)
    {
        (void)motion_drive_stop();
        state = MOTION_STATE_STALLED;
        return MOTION_OK;
    }

    /* Following has no end point and no expected time. */
    if (MOTION_STATE_FOLLOW == state)
    {
        return MOTION_OK;
    }

    if (b_stop_at_line && b_line_found)
    {
        enter_stopped();
        done_reason = MOTION_DONE_LINE;
        return MOTION_OK;
    }

    int32_t position =
        (MOTION_STATE_TURN == state) ? tick_difference() : tick_sum();
    if ((move_dir * (position - move_start)) >= move_target)
    {
        enter_stopped();
        done_reason = MOTION_DONE_TARGET;
        return MOTION_OK;
    }

    /* Wheels turning but no progress: e.g. an encoder counting backward. */
    move_updates++;
    if (move_updates >= move_limit_updates)
    {
        (void)motion_drive_stop();
        state = MOTION_STATE_TIMED_OUT;
    }
    return MOTION_OK;
}

motion_status_t
motion_move_forward (uint16_t distance_mm)
{
    return start_move(distance_mm, 1);
}

motion_status_t
motion_move_backward (uint16_t distance_mm)
{
    return start_move(distance_mm, -1);
}

motion_status_t
motion_move_turn_left (uint16_t angle_deg)
{
    return start_turn(angle_deg, 1);
}

motion_status_t
motion_move_turn_right (uint16_t angle_deg)
{
    return start_turn(angle_deg, -1);
}

motion_status_t
motion_move_u_turn (void)
{
    return start_turn(180u, 1);
}

motion_status_t
motion_move_follow (uint16_t speed_mm_s)
{
    motion_status_t status =
        check_can_start(speed_mm_s, (uint16_t)MOTION_MAX_SPEED_MM_S);

    if (status != MOTION_OK)
    {
        return status;
    }
    if (line_age_updates > LINE_TIMEOUT_UPDATES)
    {
        /* It would stop on the next update: refuse before driving. */
        return MOTION_ERR_PARAM;
    }

    /*
     * Turn no faster than 2 * speed / track (whole deg/s, rounded down),
     * so the inner wheel slows at most to 0 and never reverses: a reversal
     * resets that wheel's speed loop and jerks the car.
     */
    float max_deg_s = (float)(int32_t)((2.0f * (float)speed_mm_s) /
                                       (MOTION_TRACK_WIDTH_MM * DEG_TO_RAD));
    if (max_deg_s > MOTION_FOLLOW_MAX_DEG_S)
    {
        max_deg_s = MOTION_FOLLOW_MAX_DEG_S;
    }
    motion_pid_set_limits(&follow_pid, -max_deg_s, max_deg_s);

    /* No target or timeout: following runs until replaced or stopped. */
    begin(MOTION_STATE_FOLLOW, 0, 1, 0, 0u);
    follow_speed_mm_s = speed_mm_s;
    (void)motion_drive_set_velocity((int16_t)speed_mm_s, 0);
    return MOTION_OK;
}

motion_status_t
motion_move_set_line (int16_t error, bool b_seen)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if ((error < -MOTION_LINE_ERROR_MAX) || (error > MOTION_LINE_ERROR_MAX))
    {
        return MOTION_ERR_PARAM;
    }

    line_error = error;
    line_age_updates = 0u;

    /* Change the seen state only after enough readings in a row agree. */
    if (b_seen == b_line_seen)
    {
        line_change_samples = 0u;
    }
    else
    {
        line_change_samples++;
        if (line_change_samples >= MOTION_LINE_SEEN_SAMPLES)
        {
            b_line_seen = b_seen;
            line_change_samples = 0u;
        }
    }
    return MOTION_OK;
}

motion_status_t
motion_move_stop_at_line (void)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if ((MOTION_STATE_MOVE != state) && (MOTION_STATE_TURN != state))
    {
        return MOTION_ERR_PARAM;
    }

    b_stop_at_line = true;
    b_acquire_line = false;
    return MOTION_OK;
}

motion_status_t
motion_move_acquire_line (void)
{
    motion_status_t const result = motion_move_stop_at_line();
    if (result == MOTION_OK)
    {
        /* A line visible before this command is useful for reacquisition;
         * do not require leaving it and finding a second edge first. */
        b_acquire_line = true;
    }
    return result;
}

motion_status_t
motion_move_stop (void)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    enter_stopped();
    return MOTION_OK;
}

bool
motion_move_is_busy (void)
{
    return (MOTION_STATE_MOVE == state) || (MOTION_STATE_TURN == state) ||
           (MOTION_STATE_FOLLOW == state);
}

motion_done_t
motion_move_get_done_reason (void)
{
    return done_reason;
}

motion_state_t
motion_move_get_state (void)
{
    return state;
}

motion_status_t
motion_move_get_odometry (motion_odometry_t * p_odometry)
{
    if (NULL == p_odometry)
    {
        return MOTION_ERR_PARAM;
    }
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    p_odometry->distance_mm =
        (int32_t)(((float)tick_sum() * MM_PER_TICK) / 2.0f);
    p_odometry->speed_mm_s =
        (int16_t)(((int32_t)wheel_speed_mm_s[WHEEL_LEFT] +
                   (int32_t)wheel_speed_mm_s[WHEEL_RIGHT]) /
                  2);
    p_odometry->left_speed_mm_s = wheel_speed_mm_s[WHEEL_LEFT];
    p_odometry->right_speed_mm_s = wheel_speed_mm_s[WHEEL_RIGHT];
    p_odometry->heading_deg =
        (int32_t)(((float)tick_difference() * MM_PER_TICK) /
                  (MOTION_TRACK_WIDTH_MM * DEG_TO_RAD));
    return MOTION_OK;
}

motion_status_t
motion_move_set_sync (bool b_enable)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    b_sync = b_enable;
    motion_pid_reset(&sync_pid);
    sync_correction_deg_s = 0;
    if (MOTION_STATE_MOVE == state)
    {
        (void)motion_drive_set_velocity(
            (int16_t)(move_dir * (int32_t)MOTION_MOVE_SPEED_MM_S), 0);
    }
    return MOTION_OK;
}

motion_status_t
motion_move_get_sync (motion_sync_t * p_sync)
{
    if (NULL == p_sync)
    {
        return MOTION_ERR_PARAM;
    }
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    p_sync->error_ticks = sync_error_ticks;
    p_sync->correction_deg_s = sync_correction_deg_s;
    return MOTION_OK;
}

motion_status_t
motion_move_get_follow (motion_follow_t * p_follow)
{
    if (NULL == p_follow)
    {
        return MOTION_ERR_PARAM;
    }
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    p_follow->line_error = line_error;
    p_follow->b_line_seen = b_line_seen;
    p_follow->turn_deg_s = follow_turn_deg_s;
    return MOTION_OK;
}

/* Private function definitions. */

/* Begin a move of @p distance_mm in direction @p dir (+1 or -1). */
static motion_status_t
start_move (uint16_t distance_mm, int32_t dir)
{
    motion_status_t status = check_can_start(distance_mm, UINT16_MAX);

    if (status != MOTION_OK)
    {
        return status;
    }

    /*
     * Target in the sum of both wheels' counts, rounded to nearest. Brake
     * early by the measured coast, but always move at least one count.
     */
    float travel_mm = (float)distance_mm - MOTION_MOVE_STOP_EARLY_MM;
    int32_t target = (int32_t)(((2.0f * travel_mm) / MM_PER_TICK) + 0.5f);
    if (target < 1)
    {
        target = 1;
    }
    begin(MOTION_STATE_MOVE, tick_sum(), dir, target,
          ((uint32_t)distance_mm * 1000u) / (uint32_t)MOTION_MOVE_SPEED_MM_S);
    (void)motion_drive_set_velocity(
        (int16_t)(dir * (int32_t)MOTION_MOVE_SPEED_MM_S), 0);
    return MOTION_OK;
}

/* Begin a turn in place of @p angle_deg, left for @p dir +1, right for -1. */
static motion_status_t
start_turn (uint16_t angle_deg, int32_t dir)
{
    motion_status_t status = check_can_start(angle_deg, MOTION_MAX_TURN_DEG);

    if (status != MOTION_OK)
    {
        return status;
    }

    /*
     * Target in the right - left count difference: each wheel travels
     * angle * track / 2, in opposite directions. Brake early by the
     * measured coast, but always turn at least one count.
     */
    float angle_rad =
        ((float)angle_deg - MOTION_TURN_STOP_EARLY_DEG) * DEG_TO_RAD;
    int32_t target =
        (int32_t)(((angle_rad * MOTION_TRACK_WIDTH_MM) / MM_PER_TICK) + 0.5f);
    if (target < 1)
    {
        target = 1;
    }
    begin(MOTION_STATE_TURN, tick_difference(), dir, target,
          ((uint32_t)angle_deg * 1000u) / (uint32_t)MOTION_TURN_SPEED_DEG_S);
    (void)motion_drive_set_velocity(
        0, (int16_t)(dir * (int32_t)MOTION_TURN_SPEED_DEG_S));
    return MOTION_OK;
}

/*
 * Refuse a move or turn of @p amount before init, of 0 or above
 * @p maximum, or while stalled or timed out.
 */
static motion_status_t
check_can_start (uint16_t amount, uint16_t maximum)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if ((0u == amount) || (amount > maximum))
    {
        return MOTION_ERR_PARAM;
    }
    if (MOTION_STATE_STALLED == state)
    {
        return MOTION_ERR_STALLED;
    }
    if (MOTION_STATE_TIMED_OUT == state)
    {
        return MOTION_ERR_TIMED_OUT;
    }
    return MOTION_OK;
}

/*
 * Replace any move, turn or following with a new one, measured from
 * @p start and expected to take @p expected_ms; the stall and timeout
 * timers restart so the old command's time does not carry over, and only
 * a line seen after this counts for stopping at the line.
 */
static void
begin (motion_state_t new_state, int32_t start, int32_t dir, int32_t target,
       uint32_t expected_ms)
{
    move_updates = 0u;
    move_limit_updates =
        ((MOTION_TIMEOUT_FACTOR * expected_ms) + MOTION_TIMEOUT_MARGIN_MS) /
        MOTION_UPDATE_PERIOD_MS;
    move_start = start;
    move_dir = dir;
    move_target = target;
    still_updates[WHEEL_LEFT] = 0u;
    still_updates[WHEEL_RIGHT] = 0u;
    heading_start = tick_difference();
    sync_right_start = total_ticks[WHEEL_RIGHT];
    reset_sync();
    reset_follow();
    done_reason = MOTION_DONE_NONE;
    b_stop_at_line = false;
    b_acquire_line = false;
    b_line_seen_last = b_line_seen;
    state = new_state;
}

/* Sum of both wheels' totals: twice the chassis travel, in counts. */
static int32_t
tick_sum (void)
{
    return total_ticks[WHEEL_LEFT] + total_ticks[WHEEL_RIGHT];
}

/* Right - left totals: heading times track width, in counts; + is left. */
static int32_t
tick_difference (void)
{
    return total_ticks[WHEEL_RIGHT] - total_ticks[WHEEL_LEFT];
}

/* Brake, end any move, turn, stall or timeout; reset the command state. */
static void
enter_stopped (void)
{
    (void)motion_drive_stop();
    state = MOTION_STATE_STOPPED;
    move_start = 0;
    move_dir = 1;
    move_target = 0;
    move_updates = 0u;
    move_limit_updates = 0u;
    still_updates[WHEEL_LEFT] = 0u;
    still_updates[WHEEL_RIGHT] = 0u;
    reset_sync();
    reset_follow();
    done_reason = MOTION_DONE_NONE;
    b_stop_at_line = false;
    b_acquire_line = false;
}

/*
 * Count updates without an encoder change while the wheel is asked to
 * turn at STALL_MIN_SPEED_MM_S or more; true at the stall timeout.
 */
static bool
wheel_stalled (wheel_t wheel, int16_t delta, int16_t setpoint)
{
    float speed = (setpoint < 0) ? -(float)setpoint : (float)setpoint;

    if ((0 != delta) || (speed < STALL_MIN_SPEED_MM_S))
    {
        still_updates[wheel] = 0u;
        return false;
    }

    still_updates[wheel]++;
    return (still_updates[wheel] >= STALL_UPDATES);
}

/* Slide each wheel's speed window on by this update's count changes. */
static void
update_speeds (int16_t delta_left, int16_t delta_right)
{
    int16_t const deltas[WHEEL_COUNT] = {delta_left, delta_right};

    for (uint32_t wheel = 0u; wheel < (uint32_t)WHEEL_COUNT; wheel++)
    {
        window_sum[wheel] += (int32_t)deltas[wheel] -
                             (int32_t)window_deltas[wheel][window_index];
        window_deltas[wheel][window_index] = deltas[wheel];
        wheel_speed_mm_s[wheel] =
            (int16_t)(((float)window_sum[wheel] * MM_PER_TICK * 1000.0f) /
                      (float)(MOTION_SPEED_WINDOW_UPDATES *
                              MOTION_UPDATE_PERIOD_MS));
    }
    window_index = (window_index + 1u) % MOTION_SPEED_WINDOW_UPDATES;
}

/*
 * One straight-line step of a move: measure the heading change since the
 * move started and, with the loop on, steer against it. Heading does not
 * depend on the direction of travel, so the same sign corrects forward and
 * backward moves. With the loop on, the error also counts the right wheel's
 * extra travel per count (MOTION_SYNC_RIGHT_SCALE).
 */
static void
correct_heading (void)
{
    sync_error_ticks = tick_difference() - heading_start;
    if (!b_sync)
    {
        return;
    }

    float extra_ticks = (float)(total_ticks[WHEEL_RIGHT] - sync_right_start) *
                        MOTION_SYNC_RIGHT_SCALE;
    sync_error_ticks += (int32_t)extra_ticks;

    float error_deg = ((float)sync_error_ticks * MM_PER_TICK) /
                      (MOTION_TRACK_WIDTH_MM * DEG_TO_RAD);
    int16_t rate =
        round_rate(motion_pid_update(&sync_pid, -error_deg, UPDATE_PERIOD_S));
    if (rate != sync_correction_deg_s)
    {
        sync_correction_deg_s = rate;
        (void)motion_drive_set_velocity(
            (int16_t)(move_dir * (int32_t)MOTION_MOVE_SPEED_MM_S), rate);
    }
}

/* Clear the straight-line loop's integral, error and correction. */
static void
reset_sync (void)
{
    motion_pid_reset(&sync_pid);
    sync_error_ticks = 0;
    sync_correction_deg_s = 0;
}

/*
 * One line step while following: steer towards the line. A + error (line
 * to the left) needs a + (left) turn, so the error goes in as it is. The
 * step's dt is the time since the last one, so D stays right when the
 * line task reads slower than motion_move_update() runs.
 */
static void
follow_line (void)
{
    float dt_s = (float)follow_step_updates * UPDATE_PERIOD_S;

    follow_step_updates = 0u;
    int16_t rate =
        round_rate(motion_pid_update(&follow_pid, (float)line_error, dt_s));
    if (rate != follow_turn_deg_s)
    {
        follow_turn_deg_s = rate;
        (void)motion_drive_set_velocity((int16_t)follow_speed_mm_s, rate);
    }
}

/* Clear the line loop's history, turn rate and step timing. */
static void
reset_follow (void)
{
    motion_pid_reset(&follow_pid);
    follow_turn_deg_s = 0;
    follow_step_updates = 0u;
}

/* The drive takes whole deg/s: round @p rate to nearest. */
static int16_t
round_rate (float rate)
{
    return (int16_t)((rate >= 0.0f) ? (rate + 0.5f) : (rate - 0.5f));
}
