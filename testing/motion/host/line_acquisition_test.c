/* Exercise the real motion module with a stationary simulated drive. */
#include "../../../app_program/motion/motion_move.h"
#include "../../../app_program/motion/motion_config.h"
#include <stdio.h>
#include <string.h>

#ifdef BASELINE_ACQUIRE
#define motion_move_acquire_line motion_move_stop_at_line
#endif

static motion_command_t command;
static unsigned int stopped;
static unsigned int failures;
static motion_counts_t counts;
static unsigned int loop_resets;

motion_status_t
motion_drive_get_counts (motion_counts_t * p_counts)
{
    *p_counts = counts;
    return MOTION_OK;
}

motion_status_t
motion_drive_set_velocity (int16_t linear, int16_t angular)
{
    command.left_setpoint_mm_s = linear - angular;
    command.right_setpoint_mm_s = linear + angular;
    return MOTION_OK;
}

motion_status_t
motion_drive_control (int16_t left, int16_t right)
{
    (void)left;
    (void)right;
    return MOTION_OK;
}

motion_status_t
motion_drive_get_command (motion_command_t * p_command)
{
    *p_command = command;
    return MOTION_OK;
}

motion_status_t
motion_drive_reset_speed_loops (void)
{
    ++loop_resets;
    return MOTION_OK;
}

motion_status_t
motion_drive_stop (void)
{
    memset(&command, 0, sizeof(command));
    ++stopped;
    return MOTION_OK;
}

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

static void
feed (bool seen, unsigned int samples)
{
    for (unsigned int i = 0; i < samples; ++i)
    {
        (void)motion_move_set_line(seen);
        (void)motion_move_update();
    }
}

/* Run @p updates updates with the wheels moving these counts per update. */
static void
spin (int left, int right, unsigned int updates)
{
    for (unsigned int i = 0; i < updates; ++i)
    {
        counts.left_count = (uint16_t)(counts.left_count + left);
        counts.right_count = (uint16_t)(counts.right_count + right);
        (void)motion_move_set_line(true);
        (void)motion_move_update();
    }
}

/* Counts per update at @p speed_mm_s. */
static int
ticks_at (float speed_mm_s)
{
    float const mm_per_tick =
        (3.14159265f * MOTION_WHEEL_DIAMETER_MM) / MOTION_ENCODER_TICKS_PER_REV;
    return (int)(speed_mm_s * ((float)MOTION_UPDATE_PERIOD_MS / 1000.0f) /
                 mm_per_tick);
}

/* From rest, so no earlier speed is left in the window. */
static bool
runaway_caught (bool b_left_turn, int left, int right)
{
    spin(0, 0, 2u * MOTION_SPEED_WINDOW_UPDATES);
    loop_resets = 0u;
    (void)(b_left_turn ? motion_move_turn_left(720u)
                       : motion_move_turn_right(720u));
    spin(left, right, 2u * MOTION_SPEED_WINDOW_UPDATES);
    (void)motion_move_stop();
    return loop_resets > 0u;
}

static void
test_turn_runaway (void)
{
    int const normal = ticks_at(87.0f);
    int const runaway = ticks_at(186.0f);

    (void)motion_move_turn_left(720u);
    spin(-normal, normal, 2u * MOTION_SPEED_WINDOW_UPDATES);
    check("a turn at its own speed keeps its speed loops",
          (0u == loop_resets) && (motion_move_get_state() == MOTION_STATE_TURN));
    spin(-runaway, runaway, 2u * MOTION_SPEED_WINDOW_UPDATES);
    check("a wheel running away in a turn drops the speed loops",
          (loop_resets > 0u) && (motion_move_get_state() == MOTION_STATE_TURN));
    (void)motion_move_stop();

    /* Each wheel alone, forward and backward. */
    check("either wheel alone, in either direction, counts as running away",
          runaway_caught(true, -runaway, normal) &&
              runaway_caught(true, -normal, runaway) &&
              runaway_caught(false, runaway, -normal) &&
              runaway_caught(false, normal, -runaway));

    /* Only turns: a move and following need their speed loops. */
    spin(0, 0, 2u * MOTION_SPEED_WINDOW_UPDATES);
    loop_resets = 0u;
    (void)motion_move_forward(1000u);
    spin(ticks_at(300.0f), ticks_at(300.0f), MOTION_SPEED_WINDOW_UPDATES);
    bool const b_move_alone = 0u == loop_resets;
    (void)motion_move_stop();
    spin(0, 0, 2u * MOTION_SPEED_WINDOW_UPDATES);
    (void)motion_move_follow(MOTION_FOLLOW_SPEED_MM_S);
    spin(runaway, runaway, 2u * MOTION_SPEED_WINDOW_UPDATES);
    check("moves and following keep their speed loops",
          b_move_alone && (0u == loop_resets));
    (void)motion_move_stop();
}

/* Hold a target @p ahead_deg from the heading, one update at a time. */
static void
steer (int32_t ahead_deg, int left, int right, unsigned int updates)
{
    for (unsigned int i = 0; i < updates; ++i)
    {
        motion_odometry_t odometry;
        (void)motion_move_get_odometry(&odometry);
        (void)motion_move_set_follow_heading(odometry.heading_deg + ahead_deg);
        spin(left, right, 1u);
    }
}

/* Follow from rest with the wheels moving these counts per update. */
static motion_follow_t
hold_heading (int left, int right, unsigned int updates)
{
    motion_follow_t follow;
    spin(0, 0, 2u * MOTION_SPEED_WINDOW_UPDATES);
    (void)motion_move_follow(MOTION_FOLLOW_SPEED_MM_S);
    spin(left, right, updates);
    (void)motion_move_get_follow(&follow);
    return follow;
}

static void
test_follow_drift (void)
{
    /* The right wheel goes MOTION_SYNC_RIGHT_SCALE further per count, so
     * equal counts are really a curve to the left, 13 degrees a metre. */
    motion_follow_t follow = hold_heading(20, 20, 800u);
    check("equal counts are a left curve, so a held heading steers right",
          (follow.heading_error_deg <= -15) && (follow.turn_deg_s < 0));
    (void)motion_move_stop();

    follow = hold_heading(41, 40, 400u);
    check("a really straight run needs no correction",
          (follow.heading_error_deg >= -1) && (follow.heading_error_deg <= 1));
    (void)motion_move_stop();

    (void)hold_heading(20, 20, 800u);
    motion_odometry_t odometry;
    (void)motion_move_get_odometry(&odometry);
    (void)motion_move_set_follow_heading(odometry.heading_deg + 5);
    spin(20, 20, 1u);
    motion_follow_t fresh;
    (void)motion_move_get_follow(&fresh);
    check("a new target starts from the heading it was given",
          (fresh.heading_error_deg >= 4) && (fresh.heading_error_deg <= 6));
    (void)motion_move_stop();
}

static void
test_slow_wheel_stall (void)
{
    unsigned int const long_enough =
        (3u * MOTION_STALL_TIMEOUT_MS) / MOTION_UPDATE_PERIOD_MS;

    spin(0, 0, 2u * MOTION_SPEED_WINDOW_UPDATES);
    (void)motion_move_follow(30u);
    steer(10, 0, ticks_at(45.0f), long_enough);
    check("a stuck inner wheel in a slow turn is no stall",
          (motion_move_get_state() == MOTION_STATE_FOLLOW) &&
              ((float)command.left_setpoint_mm_s <
               MOTION_STALL_MIN_SPEED_MM_S));
    steer(0, 0, ticks_at(30.0f), long_enough);
    check("a wheel that should be driving and is not is a stall",
          motion_move_get_state() == MOTION_STATE_STALLED);
    (void)motion_move_stop();
}

static bool
braked_on_line (void)
{
    return (motion_move_get_state() == MOTION_STATE_STOPPED) &&
           (motion_move_get_done_reason() == MOTION_DONE_LINE) &&
           (command.left_setpoint_mm_s == 0) &&
           (command.right_setpoint_mm_s == 0);
}

int
main (void)
{
    check("uninitialized acquisition refused",
          motion_move_acquire_line() == MOTION_ERR_NOT_INIT);
    check("motion initializes", motion_move_init() == MOTION_OK);
    check("idle acquisition refused",
          motion_move_acquire_line() == MOTION_ERR_PARAM);

    feed(true, MOTION_LINE_SEEN_SAMPLES);
    (void)motion_move_forward(1000u);
    (void)motion_move_acquire_line();
    unsigned int before = stopped;
    (void)motion_move_update();
    check("return stops on line already visible before arming",
          braked_on_line() && stopped > before);
    (void)motion_move_stop();

    feed(true, MOTION_LINE_SEEN_SAMPLES);
    (void)motion_move_turn_right(90u);
    (void)motion_move_acquire_line();
    (void)motion_move_update();
    check("inward turn stops on already-visible line", braked_on_line());
    (void)motion_move_stop();

    feed(false, MOTION_LINE_SEEN_SAMPLES);
    (void)motion_move_forward(1000u);
    (void)motion_move_acquire_line();
    feed(true, MOTION_LINE_SEEN_SAMPLES - 1u);
    check("unconfirmed single detection does not stop",
          motion_move_get_state() == MOTION_STATE_MOVE);
    feed(true, 1u);
    check("fresh confirmed detection brakes", braked_on_line());
    (void)motion_move_stop();

    feed(true, MOTION_LINE_SEEN_SAMPLES);
    for (unsigned int i = 0;
         i <= MOTION_LINE_TIMEOUT_MS / MOTION_UPDATE_PERIOD_MS; ++i)
    {
        (void)motion_move_update();
    }
    (void)motion_move_forward(1000u);
    (void)motion_move_acquire_line();
    (void)motion_move_update();
    check("stale visible line does not stop the return",
          motion_move_get_state() == MOTION_STATE_MOVE);
    feed(true, 1u);
    check("fresh refresh of visible line brakes", braked_on_line());
    (void)motion_move_stop();

    feed(true, MOTION_LINE_SEEN_SAMPLES);
    (void)motion_move_turn_left(90u);
    (void)motion_move_stop_at_line();
    feed(true, 2u);
    check("edge-only API still turns off the starting line",
          motion_move_get_state() == MOTION_STATE_TURN);
    feed(false, MOTION_LINE_SEEN_SAMPLES);
    feed(true, MOTION_LINE_SEEN_SAMPLES);
    check("edge-only API stops when the line reappears", braked_on_line());
    (void)motion_move_stop();

    feed(false, MOTION_LINE_SEEN_SAMPLES);
    (void)motion_move_forward(1000u);
    (void)motion_move_acquire_line();
    (void)motion_move_forward(1000u);
    feed(true, MOTION_LINE_SEEN_SAMPLES);
    check("new ordinary command clears acquisition mode",
          motion_move_get_state() == MOTION_STATE_MOVE);
    (void)motion_move_stop();

    /* Heading-hold following (the stub never turns, so the error stays). */
    check("follow heading refused while not following",
          motion_move_set_follow_heading(10) == MOTION_ERR_PARAM);
    check("follow speed refused while not following",
          motion_move_set_follow_speed(30u) == MOTION_ERR_PARAM);
    feed(true, MOTION_LINE_SEEN_SAMPLES);
    check("follow starts on a fresh line",
          motion_move_follow(MOTION_FOLLOW_SPEED_MM_S) == MOTION_OK);
    (void)motion_move_set_line(true);
    (void)motion_move_update();
    check(
        "on target: both wheels at follow speed",
        (command.left_setpoint_mm_s == (int16_t)MOTION_FOLLOW_SPEED_MM_S) &&
            (command.right_setpoint_mm_s == (int16_t)MOTION_FOLLOW_SPEED_MM_S));
    /* 30 deg x KP 4 asks for 120 deg/s, past the cap: the follow limit,
     * or 2 * speed / track if lower (motion_move_follow()). */
    float cap_deg_s =
        (float)(int32_t)((2.0f * (float)MOTION_FOLLOW_SPEED_MM_S) /
                         (MOTION_TRACK_WIDTH_MM * (3.14159265f / 180.0f)));
    if (cap_deg_s > MOTION_FOLLOW_MAX_DEG_S)
    {
        cap_deg_s = MOTION_FOLLOW_MAX_DEG_S;
    }
    /* Slow over a barcode: the same target, a lower cap and half the gain
     * by speed, then back. 8 deg at KP 4 asks 32 deg/s; at 30 mm/s 12. */
    check("follow speed refuses zero",
          motion_move_set_follow_speed(0u) == MOTION_ERR_PARAM);
    (void)motion_move_set_follow_heading(8);
    check("follow speed changes while following",
          motion_move_set_follow_speed(30u) == MOTION_OK);
    (void)motion_move_set_line(true);
    (void)motion_move_update();
    motion_follow_t slow;
    (void)motion_move_get_follow(&slow);
    check("slow: the gain scales with speed and the target stays",
          (slow.turn_deg_s == 12) && (slow.target_heading_deg == 8) &&
              (((command.left_setpoint_mm_s + command.right_setpoint_mm_s) /
                2) == 30));
    (void)motion_move_set_follow_heading(90);
    (void)motion_move_set_line(true);
    (void)motion_move_update();
    (void)motion_move_get_follow(&slow);
    float const slow_cap =
        (float)(int32_t)((2.0f * 30.0f) /
                         (MOTION_TRACK_WIDTH_MM * (3.14159265f / 180.0f)));
    check("slow: the turn cap scales with speed",
          slow.turn_deg_s == (int16_t)slow_cap);
    check("follow speed goes back up",
          motion_move_set_follow_speed(MOTION_FOLLOW_SPEED_MM_S) == MOTION_OK);
    (void)motion_move_set_follow_heading(0);
    (void)motion_move_set_line(true);
    (void)motion_move_update();
    check("full speed: both wheels at follow speed again",
          (command.left_setpoint_mm_s == (int16_t)MOTION_FOLLOW_SPEED_MM_S) &&
              (command.right_setpoint_mm_s ==
               (int16_t)MOTION_FOLLOW_SPEED_MM_S));

    (void)motion_move_set_follow_heading(30);
    (void)motion_move_set_line(true);
    (void)motion_move_update();
    motion_follow_t follow;
    (void)motion_move_get_follow(&follow);
    check("left target turns left, capped at the follow limit",
          (follow.turn_deg_s == (int16_t)cap_deg_s) &&
              (follow.heading_error_deg == 30) &&
              (command.right_setpoint_mm_s > command.left_setpoint_mm_s));
    (void)motion_move_set_follow_heading(-1);
    (void)motion_move_set_line(true);
    (void)motion_move_update();
    (void)motion_move_get_follow(&follow);
    check("small right error turns right gently",
          (follow.turn_deg_s < 0) &&
              (follow.turn_deg_s > -(int16_t)MOTION_FOLLOW_MAX_DEG_S));
    feed(true, 0u);
    for (unsigned int i = 0;
         i <= MOTION_LINE_TIMEOUT_MS / MOTION_UPDATE_PERIOD_MS; ++i)
    {
        (void)motion_move_update();
    }
    check("stale line input ends following",
          motion_move_get_state() == MOTION_STATE_STOPPED);
    test_turn_runaway();
    test_slow_wheel_stall();
    test_follow_drift();

    printf("%u failures\n", failures);
    return failures ? 1 : 0;
}
