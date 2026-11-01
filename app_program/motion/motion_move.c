/**
 * @file motion_move.c
 * @brief Odometry, open-loop distance moves, turns and stall detection.
 *
 * Odometry adds the wrap-safe 16-bit encoder change of each update to
 * this module's own 32-bit running totals (Tribunal 1), so the raw 16-bit
 * counts never need to be compared directly. A move is measured by the
 * sum of both wheels' totals, a turn by their difference (right - left),
 * which is the heading change times the track width.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "motion_move.h"
#include <stddef.h>
#include "motion_config.h"

/* Private types, constants, and macros. */

/* Wheel travel per encoder count, in mm. */
#define MM_PER_TICK                                                            \
    ((3.14159265f * MOTION_WHEEL_DIAMETER_MM) /                                \
     (float)MOTION_ENCODER_TICKS_PER_REV)

/* Multiply degrees by this to get radians, and radians by its inverse. */
#define DEG_TO_RAD (3.14159265f / 180.0f)

/* Updates without an encoder change, during a move, that mean a stall. */
#define STALL_UPDATES (MOTION_STALL_TIMEOUT_MS / MOTION_UPDATE_PERIOD_MS)

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
static int16_t speed_mm_s = 0;

/*
 * Current move or turn: start, direction and target, in the sum of both
 * totals for a move or their difference (right - left) for a turn.
 */
static int32_t move_start = 0;
static int32_t move_dir = 1;
static int32_t move_target = 0;
static uint32_t still_updates[WHEEL_COUNT];

/* Private function declarations. */
static motion_status_t start_move (uint16_t distance_mm, int32_t dir);
static motion_status_t start_turn (uint16_t angle_deg, int32_t dir);
static motion_status_t check_can_start (uint16_t amount);
static void begin (motion_state_t new_state, int32_t start, int32_t dir,
                   int32_t target);
static int32_t tick_sum (void);
static int32_t tick_difference (void);
static void enter_stopped (void);
static bool wheel_stalled (wheel_t wheel, int16_t delta);

/* Public function definitions. */

motion_status_t
motion_move_init (void)
{
    if (motion_drive_get_counts(&last_counts) != MOTION_OK)
    {
        return MOTION_ERR_NOT_INIT;
    }

    total_ticks[WHEEL_LEFT] = 0;
    total_ticks[WHEEL_RIGHT] = 0;
    speed_mm_s = 0;
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
    speed_mm_s =
        (int16_t)(((float)(delta_left + delta_right) * MM_PER_TICK * 1000.0f) /
                  (2.0f * (float)MOTION_UPDATE_PERIOD_MS));

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
    p_odometry->speed_mm_s = speed_mm_s;
    p_odometry->heading_deg =
        (int32_t)(((float)tick_difference() * MM_PER_TICK) /
                  (MOTION_TRACK_WIDTH_MM * DEG_TO_RAD));
    return MOTION_OK;
}

/* Private function definitions. */

/* Begin a move of @p distance_mm in direction @p dir (+1 or -1). */
static motion_status_t
start_move (uint16_t distance_mm, int32_t dir)
{
    motion_status_t status = check_can_start(distance_mm);

    if (status != MOTION_OK)
    {
        return status;
    }

    /* Target in the sum of both wheels' counts, rounded to nearest. */
    begin(MOTION_STATE_MOVE, tick_sum(), dir,
          (int32_t)(((2.0f * (float)distance_mm) / MM_PER_TICK) + 0.5f));
    (void)motion_drive_set_velocity(
        (int16_t)(dir * (int32_t)MOTION_MOVE_SPEED_MM_S), 0);
    return MOTION_OK;
}

/* Begin a turn in place of @p angle_deg, left for @p dir +1, right for -1. */
static motion_status_t
start_turn (uint16_t angle_deg, int32_t dir)
{
    motion_status_t status = check_can_start(angle_deg);

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
    begin(MOTION_STATE_TURN, tick_difference(), dir, target);
    (void)motion_drive_set_velocity(
        0, (int16_t)(dir * (int32_t)MOTION_TURN_SPEED_DEG_S));
    return MOTION_OK;
}

/* Refuse a move or turn of @p amount before init, of 0, or while stalled. */
static motion_status_t
check_can_start (uint16_t amount)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if (0u == amount)
    {
        return MOTION_ERR_PARAM;
    }
    if (MOTION_STATE_STALLED == state)
    {
        return MOTION_ERR_STALLED;
    }
    return MOTION_OK;
}

/*
 * Replace any move or turn with a new one, measured from @p start; the
 * stall timers restart so the old command's still time does not carry
 * over.
 */
static void
begin (motion_state_t new_state, int32_t start, int32_t dir, int32_t target)
{
    move_start = start;
    move_dir = dir;
    move_target = target;
    still_updates[WHEEL_LEFT] = 0u;
    still_updates[WHEEL_RIGHT] = 0u;
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

/* Brake, end any move, turn or stall, and reset the command state. */
static void
enter_stopped (void)
{
    (void)motion_drive_stop();
    state = MOTION_STATE_STOPPED;
    move_start = 0;
    move_dir = 1;
    move_target = 0;
    still_updates[WHEEL_LEFT] = 0u;
    still_updates[WHEEL_RIGHT] = 0u;
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
