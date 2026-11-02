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
static int32_t sync_error_ticks = 0;
static int16_t sync_correction_deg_s = 0;

/* Private function declarations. */
static motion_status_t start_move (uint16_t distance_mm, int32_t dir);
static motion_status_t start_turn (uint16_t angle_deg, int32_t dir);
static motion_status_t check_can_start (uint16_t amount, uint16_t maximum);
static void begin (motion_state_t new_state, int32_t start, int32_t dir,
                   int32_t target, uint32_t expected_ms);
static int32_t tick_sum (void);
static int32_t tick_difference (void);
static void enter_stopped (void);
static bool wheel_stalled (wheel_t wheel, int16_t delta);
static void update_speeds (int16_t delta_left, int16_t delta_right);
static void correct_heading (void);
static void reset_sync (void);

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
    b_sync = true;
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
    if (MOTION_STATE_MOVE == state)
    {
        correct_heading();
    }
    (void)motion_drive_control(wheel_speed_mm_s[WHEEL_LEFT],
                               wheel_speed_mm_s[WHEEL_RIGHT]);

    if (!motion_move_is_busy())
    {
        return MOTION_OK;
    }

    /* Evaluate both wheels so each one's timer stays current. */
    bool b_left_stalled = wheel_stalled(WHEEL_LEFT, delta_left);
    bool b_right_stalled = wheel_stalled(WHEEL_RIGHT, delta_right);
    if (b_left_stalled || b_right_stalled)
    {
        (void)motion_drive_stop();
        state = MOTION_STATE_STALLED;
        return MOTION_OK;
    }

    int32_t position =
        (MOTION_STATE_TURN == state) ? tick_difference() : tick_sum();
    if ((move_dir * (position - move_start)) >= move_target)
    {
        enter_stopped();
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
    return (MOTION_STATE_MOVE == state) || (MOTION_STATE_TURN == state);
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

    /* Target in the sum of both wheels' counts, rounded to nearest. */
    begin(MOTION_STATE_MOVE, tick_sum(), dir,
          (int32_t)(((2.0f * (float)distance_mm) / MM_PER_TICK) + 0.5f),
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
 * Replace any move or turn with a new one, measured from @p start and
 * expected to take @p expected_ms; the stall and timeout timers restart so
 * the old command's time does not carry over.
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
    reset_sync();
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
}

/* Count updates without an encoder change; true at the stall timeout. */
static bool
wheel_stalled (wheel_t wheel, int16_t delta)
{
    if (0 != delta)
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
 * backward moves.
 */
static void
correct_heading (void)
{
    sync_error_ticks = tick_difference() - heading_start;
    if (!b_sync)
    {
        return;
    }

    float error_deg = ((float)sync_error_ticks * MM_PER_TICK) /
                      (MOTION_TRACK_WIDTH_MM * DEG_TO_RAD);
    float correction =
        motion_pid_update(&sync_pid, -error_deg, UPDATE_PERIOD_S);

    /* The drive takes whole deg/s: round to nearest. */
    int16_t rate = (int16_t)((correction >= 0.0f) ? (correction + 0.5f)
                                                  : (correction - 0.5f));
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
