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

motion_status_t
motion_drive_get_counts (motion_counts_t * p_counts)
{
    *p_counts = (motion_counts_t){0};
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
    printf("%u failures\n", failures);
    return failures ? 1 : 0;
}
