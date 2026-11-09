/* Drive the real motion_command.c with stand-ins for the kernel and motion. */
#include "../../../app_program/motion/motion_command.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../../../app_program/motion/motion_command_config.h"
#include "../../../app_program/motion/motion_move.h"

static uint32_t clock_ms;
static motion_state_t state = MOTION_STATE_STOPPED;
static unsigned int follows;
static unsigned int stops;
static unsigned int line_feeds;
static int32_t heading_deg;
static unsigned int braking_lines;
static unsigned int failures;

ER
tk_get_otm (SYSTIM * p_time)
{
    p_time->hi = 0;
    p_time->lo = clock_ms;
    return E_OK;
}

INT
tm_printf (UB const * p_format, ...)
{
    braking_lines +=
        (strstr((char const *)p_format, "braking") != NULL) ? 1u : 0u;
    return 0;
}

motion_state_t
motion_move_get_state (void)
{
    return state;
}

bool
motion_move_is_busy (void)
{
    return state != MOTION_STATE_STOPPED;
}

motion_status_t
motion_move_stop (void)
{
    stops++;
    state = MOTION_STATE_STOPPED;
    return MOTION_OK;
}

motion_status_t
motion_move_set_line (bool b_seen)
{
    (void)b_seen;
    line_feeds++;
    return MOTION_OK;
}

motion_status_t
motion_move_follow (uint16_t speed_mm_s)
{
    (void)speed_mm_s;
    follows++;
    state = MOTION_STATE_FOLLOW;
    return MOTION_OK;
}

motion_status_t
motion_move_set_follow_speed (uint16_t speed_mm_s)
{
    (void)speed_mm_s;
    return MOTION_OK;
}

motion_status_t
motion_move_set_follow_heading (int32_t target_deg)
{
    heading_deg = target_deg;
    return MOTION_OK;
}

motion_status_t
motion_move_backward (uint16_t distance_mm)
{
    (void)distance_mm;
    return MOTION_OK;
}

motion_status_t
motion_move_turn_left (uint16_t angle_deg)
{
    (void)angle_deg;
    return MOTION_OK;
}

motion_status_t
motion_move_turn_right (uint16_t angle_deg)
{
    (void)angle_deg;
    return MOTION_OK;
}

motion_status_t
motion_move_get_odometry (motion_odometry_t * p_odometry)
{
    memset(p_odometry, 0, sizeof(*p_odometry));
    return MOTION_OK;
}

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

/* One 20 ms update of tMotion. */
static void
update (void)
{
    clock_ms += MOTION_UPDATE_PERIOD_MS;
    motion_command_service(true);
}

static ER
submit (int32_t target_deg)
{
    line_request_t request = {.mode = LINE_MODE_TRACK,
                              .target_heading_deg = target_deg};
    return motion_command_submit(&request);
}

int
main (void)
{
    line_request_t none = {.mode = LINE_MODE_TRACK};
    check("CS01 no request before the slots are ready",
          motion_command_submit(&none) == E_OBJ);
    check("CS02 ready once, then refused",
          (motion_command_init() == E_OK) &&
              (motion_command_init() == E_OBJ) &&
              (motion_command_start() == E_OK));
    clock_ms = MOTION_COMMAND_START_DELAY_MS + 1000u;

    /* Every request arrives: nothing can be busy, nothing is skipped. */
    bool b_every = true;
    for (int32_t i = 1; i <= 200; i++)
    {
        b_every = b_every && (submit(i) == E_OK);
        update();
        b_every = b_every && (heading_deg == i);
    }
    check("CS03 200 requests in a row each reach motion on the next update",
          b_every && (1u == follows) && (0u == stops));

    uint32_t requests = 0u;
    uint32_t taken = 0u;
    motion_command_get_counts(&requests, &taken);
    check("CS04 the counts show them all published and taken",
          (200u == requests) && (200u == taken));

    /* The two slots in turn: the later of two requests is the one used. */
    (void)submit(1000);
    (void)submit(1001);
    update();
    motion_command_get_counts(&requests, &taken);
    check("CS05 of two requests between updates, the later wins",
          (1001 == heading_deg) && (202u == requests) && (201u == taken));

    unsigned int const lines_before = braking_lines;
    update();
    update();
    update();
    motion_command_get_counts(&requests, &taken);
    check("CS06 under the timeout it keeps following, and takes nothing twice",
          (MOTION_STATE_FOLLOW == state) && (0u == stops) && (201u == taken));
    update();
    update();
    check("CS07 past the timeout it brakes and says so, once",
          (MOTION_STATE_STOPPED == state) && (1u == stops) &&
              (lines_before + 1u == braking_lines));
    update();
    update();
    check("CS08 stopped, it stays quiet",
          (1u == stops) && (lines_before + 1u == braking_lines));

    (void)submit(7);
    update();
    check("CS09 the next request drives again",
          (MOTION_STATE_FOLLOW == state) && (7 == heading_deg));

    check("CS10 a sensor failure stores a stop",
          motion_command_submit(NULL) == E_OK);
    update();
    check("CS11 and motion brakes on it", MOTION_STATE_STOPPED == state);

    /* Control off: visibility still feeds stop-at-line, nothing drives. */
    unsigned int const feeds = line_feeds;
    unsigned int const started = follows;
    (void)submit(9);
    clock_ms += MOTION_UPDATE_PERIOD_MS;
    motion_command_service(false);
    check("CS12 with control off a request feeds the line and drives nothing",
          (feeds + 1u == line_feeds) && (started == follows));

    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
