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
#include "../test_report.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS 97 /* Not a multiple of the scan period. */
#define PWM_SLICE_STRIDE 0x14u
#define SERVO_SLICE (((UW)OBSTACLE_SERVO_PIN / 2u) % 8u)
#define SERVO_WRAP 19999u
#define SERVO_DIV_1MHZ ((UW)SYSCLK << 4) /* clk_sys MHz in 8.4 fixed point. */

/* Private function declarations. */
static bool packed_obstacle_fields_valid (void);

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

    TEST_SUMMARY("obstacle");
}

/* Private function definitions. */

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
