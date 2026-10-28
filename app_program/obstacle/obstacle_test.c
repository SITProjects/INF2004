/**
 * @file obstacle_test.c
 * @brief On-target tests for the obstacle subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/obstacle/logs/ and record the result in
 * testing/obstacle/TEST_LOG.md. Test IDs: OBS-U01, OBS-U02, ...
 */

#include "obstacle.h"
#include "obstacle_decision.h"
#include "../test_report.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS 97 /* Not a multiple of the scan period. */
#define PWM_SLICE_STRIDE 0x14u
#define SERVO_SLICE (((UW)OBSTACLE_SERVO_PIN / 2u) % 8u)
#define SERVO_WRAP 19999u
#define SERVO_DIV_1MHZ ((UW)SYSCLK << 4) /* clk_sys MHz in 8.4 fixed point. */

/* Echo durations around the configured detection threshold. */
#define THRESHOLD_US ((uint16_t)OBSTACLE_DETECT_ECHO_US)
#define NEAR_US ((uint16_t)(OBSTACLE_DETECT_ECHO_US - 1u))
#define CLEAR_US ((uint16_t)(OBSTACLE_DETECT_ECHO_US + 100u))
#define FARTHER_US ((uint16_t)(OBSTACLE_DETECT_ECHO_US + 200u))

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

/** One side-selection case: both inputs and the expected decision. */
typedef struct
{
    ultrasonic_result_t left;
    uint16_t left_us;
    ultrasonic_result_t right;
    uint16_t right_us;
    robot_obstacle_status_t status;
    robot_obstacle_direction_t direction;
} side_case_t;

/* Private function declarations. */
static bool packed_obstacle_fields_valid (void);
static bool median_valid (void);
static bool position_valid (void);
static bool centre_valid (void);
static bool sides_valid (void);

/* Public function definitions. */

/**
 * @brief Run on-target task/PWM checks and the pure packed-data regression.
 * Called by the initial task after subsystem start, only with APP_TEST.
 */
void
obstacle_test (void)
{
    TEST_INIT();

    /*
     * OBS-U01: the scan task started (so obstacle_scan_init() succeeded)
     * and sleeps between scans instead of busy-looping.
     */
    INT waiting = 0;
    for (INT i = 0; i < SAMPLES; i++)
    {
        T_RTSK rtsk;
        (void)tk_dly_tsk(SAMPLE_GAP_MS);
        if ((tk_ref_tsk(obstacle_tid_scan, &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0u))
        {
            waiting++;
        }
    }
    TEST_CHECK("OBS-U01", waiting >= (SAMPLES - 1),
               "scan task runs and sleeps between scans");

    /* OBS-U02: the servo slice runs a 1 MHz counter that wraps at 20 ms. */
    UW base = PWM_BASE + (PWM_SLICE_STRIDE * SERVO_SLICE);
    TEST_CHECK("OBS-U02",
               ((in_w(base + PWM_CHx_CSR) & PWM_CH_CSR_EN) != 0u) &&
                   (in_w(base + PWM_CHx_TOP) == SERVO_WRAP) &&
                   (in_w(base + PWM_CHx_DIV) == SERVO_DIV_1MHZ),
               "servo PWM slice runs at 50 Hz");

    TEST_CHECK("OBS-U10", packed_obstacle_fields_valid(),
               "packed obstacle fields clamp without changing neighbours");

    TEST_CHECK("OBS-U16", median_valid(),
               "ping median ignores no-echo samples and averages two");

    TEST_CHECK("OBS-U17", position_valid(),
               "position result is FAULT, NO_ECHO or VALID");

    TEST_CHECK("OBS-U18", centre_valid(),
               "centre: no echo or far is CLEAR, near scans, fault UNKNOWN");

    TEST_CHECK("OBS-U19", sides_valid(),
               "sides: farther clear wins, no echo first, ties go left");

    TEST_SUMMARY("obstacle");
}

/* Private function definitions. */

/* OBS-U16: median of the nonzero samples; zero means no echo. */
static bool
median_valid (void)
{
    const uint16_t mixed[] = {0u, 1166u, 1283u};
    const uint16_t three[] = {1458u, 1399u, 1516u};
    const uint16_t none[] = {0u, 0u, 0u};
    const uint16_t one[] = {0u, 0u, 700u};
    const uint16_t largest[] = {0u, UINT16_MAX, UINT16_MAX};
    const uint16_t single[] = {500u};
    const uint16_t extra[] = {100u, 200u, 300u, 1u};

    return (obstacle_decision_median(mixed, 3u) == 1224u) &&
           (obstacle_decision_median(three, 3u) == 1458u) &&
           (obstacle_decision_median(none, 3u) == 0u) &&
           (obstacle_decision_median(one, 3u) == 700u) &&
           /* Averaging two maximum samples must not overflow. */
           (obstacle_decision_median(largest, 3u) == UINT16_MAX) &&
           (obstacle_decision_median(single, 1u) == 500u) &&
           (obstacle_decision_median(single, 0u) == 0u) &&
           (obstacle_decision_median(NULL, 3u) == 0u) &&
           /* Only the first OBSTACLE_DECISION_MEDIAN_MAX samples count. */
           (obstacle_decision_median(extra, (uint8_t)ARRAY_COUNT(extra)) ==
            200u);
}

/* OBS-U17: exhausted retries win over the median value. */
static bool
position_valid (void)
{
    return (obstacle_decision_position(0u, false) == ULTRASONIC_NO_ECHO) &&
           (obstacle_decision_position(NEAR_US, false) == ULTRASONIC_VALID) &&
           (obstacle_decision_position(NEAR_US, true) == ULTRASONIC_FAULT) &&
           (obstacle_decision_position(0u, true) == ULTRASONIC_FAULT);
}

/* OBS-U18: threshold boundary and the no-echo-is-clear team decision. */
static bool
centre_valid (void)
{
    return (obstacle_decision_centre(ULTRASONIC_NO_ECHO, 0u) ==
            OBSTACLE_CENTRE_CLEAR) &&
           (obstacle_decision_centre(ULTRASONIC_VALID, THRESHOLD_US) ==
            OBSTACLE_CENTRE_CLEAR) &&
           (obstacle_decision_centre(ULTRASONIC_VALID, UINT16_MAX) ==
            OBSTACLE_CENTRE_CLEAR) &&
           (obstacle_decision_centre(ULTRASONIC_VALID, NEAR_US) ==
            OBSTACLE_CENTRE_SCAN_SIDES) &&
           (obstacle_decision_centre(ULTRASONIC_VALID, 1u) ==
            OBSTACLE_CENTRE_SCAN_SIDES) &&
           (obstacle_decision_centre(ULTRASONIC_FAULT, CLEAR_US) ==
            OBSTACLE_CENTRE_UNKNOWN) &&
           (obstacle_decision_centre(ULTRASONIC_PENDING, CLEAR_US) ==
            OBSTACLE_CENTRE_UNKNOWN);
}

/* OBS-U19: every combination class of left and right results. */
static bool
sides_valid (void)
{
    const side_case_t cases[] = {
        /* Both near: no path. */
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_VALID, NEAR_US,
         ROBOT_OBSTACLE_BLOCKED, ROBOT_OBSTACLE_DIR_NONE},
        /* Exactly at the threshold counts as clear. */
        {ULTRASONIC_VALID, THRESHOLD_US, ULTRASONIC_VALID, NEAR_US,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_VALID, THRESHOLD_US,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        /* Equal clear distances select left. */
        {ULTRASONIC_VALID, CLEAR_US, ULTRASONIC_VALID, CLEAR_US,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* The farther clear side wins. */
        {ULTRASONIC_VALID, CLEAR_US, ULTRASONIC_VALID, FARTHER_US,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_VALID, FARTHER_US, ULTRASONIC_VALID, CLEAR_US,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* No echo ranks beyond any finite echo. */
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_VALID, UINT16_MAX,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        {ULTRASONIC_VALID, UINT16_MAX, ULTRASONIC_NO_ECHO, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_NO_ECHO, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* A fault with no clear side is UNKNOWN, not BLOCKED. */
        {ULTRASONIC_FAULT, 0u, ULTRASONIC_VALID, NEAR_US,
         ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE},
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_FAULT, 0u,
         ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE},
        {ULTRASONIC_FAULT, 0u, ULTRASONIC_FAULT, 0u, ROBOT_OBSTACLE_UNKNOWN,
         ROBOT_OBSTACLE_DIR_NONE},
        /* A clear side is still chosen when the other side faulted. */
        {ULTRASONIC_FAULT, 0u, ULTRASONIC_VALID, CLEAR_US,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_FAULT, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
    };

    for (unsigned int index = 0u; index < ARRAY_COUNT(cases); ++index)
    {
        obstacle_side_decision_t decision =
            obstacle_decision_sides(cases[index].left, cases[index].left_us,
                                    cases[index].right, cases[index].right_us);
        if ((decision.status != cases[index].status) ||
            (decision.direction != cases[index].direction))
        {
            tm_printf((UB *)"obstacle_test: OBS-U19 case %u failed\n", index);
            return false;
        }
    }
    return true;
}

/* Exercise independent packed fields and the explicit saturation rules. */
static bool
packed_obstacle_fields_valid (void)
{
    for (unsigned int status = 0u; status <= ROBOT_OBSTACLE_UNKNOWN; ++status)
    {
        for (unsigned int direction = 0u;
             direction <= ROBOT_OBSTACLE_DIR_RIGHT_120; ++direction)
        {
            for (unsigned int edges = 0u; edges < 4u; ++edges)
            {
                uint8_t flags = robot_obstacle_pack_flags(
                    (robot_obstacle_status_t)status,
                    (robot_obstacle_direction_t)direction, (edges & 1u) != 0u,
                    (edges & 2u) != 0u);
                if ((robot_obstacle_get_status(flags) != status) ||
                    (robot_obstacle_get_direction(flags) != direction) ||
                    (((flags & ROBOT_OBSTACLE_LEFT_EDGE_FOUND) != 0u) !=
                     ((edges & 1u) != 0u)) ||
                    (((flags & ROBOT_OBSTACLE_RIGHT_EDGE_FOUND) != 0u) !=
                     ((edges & 2u) != 0u)) ||
                    ((flags & 0x80u) != 0u))
                {
                    return false;
                }
            }
        }
    }

    const unsigned int invalid_status[] = {ROBOT_OBSTACLE_UNKNOWN + 1u,
                                           UINT8_MAX};
    const unsigned int invalid_direction[] = {ROBOT_OBSTACLE_DIR_RIGHT_120 + 1u,
                                              UINT8_MAX};
    for (unsigned int index = 0u;
         index < sizeof(invalid_status) / sizeof(invalid_status[0]); ++index)
    {
        uint8_t flags = robot_obstacle_pack_flags(
            (robot_obstacle_status_t)invalid_status[index],
            ROBOT_OBSTACLE_DIR_RIGHT_120, true, true);
        if ((robot_obstacle_get_status(flags) != ROBOT_OBSTACLE_UNKNOWN) ||
            ((flags & (uint8_t)~ROBOT_OBSTACLE_STATUS_MASK) != 0x78u))
        {
            return false;
        }
        flags = robot_obstacle_pack_flags(
            ROBOT_OBSTACLE_PATH_FOUND,
            (robot_obstacle_direction_t)invalid_direction[index], true, true);
        if ((robot_obstacle_get_direction(flags) != ROBOT_OBSTACLE_DIR_NONE) ||
            ((flags & (uint8_t)~ROBOT_OBSTACLE_DIRECTION_MASK) != 0x62u))
        {
            return false;
        }
    }
    if (robot_obstacle_get_status(0xFFu) != ROBOT_OBSTACLE_UNKNOWN)
    {
        return false;
    }

    const unsigned int edge_input[] = {0u, ROBOT_OBSTACLE_EDGE_MAX_INDEX,
                                       ROBOT_OBSTACLE_EDGE_MAX_INDEX + 1u,
                                       ROBOT_OBSTACLE_EDGE_NONE, UINT16_MAX};
    for (unsigned int left = 0u;
         left < sizeof(edge_input) / sizeof(edge_input[0]); ++left)
    {
        for (unsigned int right = 0u;
             right < sizeof(edge_input) / sizeof(edge_input[0]); ++right)
        {
            uint8_t expected_left =
                (edge_input[left] <= ROBOT_OBSTACLE_EDGE_MAX_INDEX)
                    ? (uint8_t)edge_input[left]
                    : ROBOT_OBSTACLE_EDGE_NONE;
            uint8_t expected_right =
                (edge_input[right] <= ROBOT_OBSTACLE_EDGE_MAX_INDEX)
                    ? (uint8_t)edge_input[right]
                    : ROBOT_OBSTACLE_EDGE_NONE;
            uint8_t edges =
                robot_obstacle_pack_edges(edge_input[left], edge_input[right]);
            if ((robot_obstacle_get_left_edge(edges) != expected_left) ||
                (robot_obstacle_get_right_edge(edges) != expected_right))
            {
                return false;
            }
        }
    }

    if ((robot_obstacle_get_left_edge(0xEDu) != ROBOT_OBSTACLE_EDGE_NONE) ||
        (robot_obstacle_get_right_edge(0xEDu) != ROBOT_OBSTACLE_EDGE_NONE))
    {
        return false;
    }

    const uint32_t width_input[] = {0u, ROBOT_OBSTACLE_WIDTH_MAX_MM,
                                    ROBOT_OBSTACLE_WIDTH_MAX_MM + 1u,
                                    UINT32_MAX};
    const uint16_t expected_width[] = {0u, ROBOT_OBSTACLE_WIDTH_MAX_MM,
                                       ROBOT_OBSTACLE_WIDTH_MAX_MM,
                                       ROBOT_OBSTACLE_WIDTH_MAX_MM};
    robot_obstacle_result_t sample = {
        .flags = robot_obstacle_pack_flags(
            ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60, true, true),
        .edges = robot_obstacle_pack_edges(2u, 10u),
        .width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE,
        .sequence = UINT8_MAX};
    for (unsigned int index = 0u;
         index < sizeof(width_input) / sizeof(width_input[0]); ++index)
    {
        sample.width_mm = robot_obstacle_clamp_width(width_input[index]);
        if ((sample.width_mm != expected_width[index]) ||
            (sample.flags != 0x6Au) || (sample.edges != 0xA2u) ||
            (sample.sequence != UINT8_MAX))
        {
            return false;
        }
    }
    sample.sequence = (uint8_t)(sample.sequence + 1u);
    if ((sample.sequence != 0u) || (sample.flags != 0x6Au) ||
        (sample.edges != 0xA2u) ||
        (sample.width_mm != ROBOT_OBSTACLE_WIDTH_MAX_MM))
    {
        return false;
    }
    return true;
}

#endif /* APP_TEST */
