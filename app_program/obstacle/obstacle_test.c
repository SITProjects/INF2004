/**
 * @file obstacle_test.c
 * @brief On-target tests for the obstacle subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/obstacle/logs/ and record the result in
 * testing/obstacle/TEST_LOG.md. Test IDs: OBS-U01, OBS-U02, ...
 */

#include "../test_report.h"
#include "obstacle.h"
#include "obstacle_decision.h"
#include "obstacle_scan.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS (10u * OBSTACLE_UPDATE_PERIOD_MS)
#define TEST_POLL_MS OBSTACLE_TICK_MS
#define TEST_MARGIN_MS (8u * (OBSTACLE_TICK_MS + OBSTACLE_UPDATE_PERIOD_MS))
#define TEST_ACK_MS (OBSTACLE_TRANSACTION_MS + TEST_MARGIN_MS)
#define TEST_RECOVERY_MS (OBSTACLE_SCAN_TIMEOUT_MS + 15000u)
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
static bool sleeping_valid (void);
static void sleep_sampler (INT stacd, void * p_exinf);
static bool critical_valid (void);
static bool critical_release_valid (void);
static bool elapsed_valid (void);
static bool staleness_valid (void);
static bool scan_timeout_valid (void);
static bool wait_snapshot (obstacle_scan_test_snapshot_t * p_snapshot,
                           bool hold, bool clear, bool sides,
                           uint32_t timeout_ms);
static bool release_hold (void);

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
    TEST_CHECK("OBS-U01", sleeping_valid(),
               "scan task runs and sleeps between scans");

    /* OBS-U02: the servo slice runs a 1 MHz counter that wraps at 20 ms. */
    UW base = PWM_BASE + (PWM_SLICE_STRIDE * SERVO_SLICE);
    TEST_CHECK("OBS-U02",
               (OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF) &&
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

    tm_printf((UB *)"[TEST] OBS-U20 critical boundary %lu mm (%lu us)\n",
              (UW)OBSTACLE_CRITICAL_DISTANCE_MM, (UW)OBSTACLE_CRITICAL_ECHO_US);
    TEST_CHECK("OBS-U20", critical_valid(),
               "critical: VALID below configured threshold only");
    tm_printf((UB *)"[TEST] OBS-U21 release boundary %lu mm (%lu us)\n",
              (UW)OBSTACLE_CRITICAL_RELEASE_DISTANCE_MM,
              (UW)OBSTACLE_CRITICAL_RELEASE_ECHO_US);
    TEST_CHECK("OBS-U21", critical_release_valid(),
               "critical release: configured boundary is exclusive");
    TEST_CHECK("OBS-U22", elapsed_valid(), "elapsed limit: strict, wrap-safe");
    TEST_CHECK("OBS-U23", staleness_valid(),
               "stale watchdog event, UNKNOWN, and clear-path recovery");
    TEST_CHECK("OBS-U24", scan_timeout_valid(),
               "side timeout event, UNKNOWN, and clear-path recovery");

    TEST_SUMMARY("obstacle");
}

/* Private function definitions. */

/* Same-core lower-priority sampling cannot pre-empt a READY scanner.
 * The initial task's bounded flag wait also detects a scanner busy loop.
 * No sampler result is shared outside the kernel event-flag protocol. */
static void
sleep_sampler (INT stacd, void * p_exinf)
{
    (void)p_exinf;
    obstacle_scan_test_snapshot_t before;
    obstacle_scan_test_snapshot_t after;
    INT waiting = 0;
    bool ok = obstacle_scan_test_snapshot(&before) == 0;
    for (INT i = 0; i < SAMPLES; ++i)
    {
        T_RTSK state;
        if ((tk_dly_tsk(SAMPLE_GAP_MS) != E_OK) ||
            (tk_ref_tsk(obstacle_tid_scan, &state) != E_OK))
        {
            ok = false;
            break;
        }
        if ((state.tskstat & TTS_WAI) != 0u)
        {
            ++waiting;
        }
    }
    ok = ok && obstacle_scan_test_snapshot(&after) == 0 && after.initialized &&
         after.heartbeat != before.heartbeat;
    ER error =
        tk_set_flg((ID)stacd, (ok && (waiting >= SAMPLES - 1)) ? 1u : 2u);
    app_check(error, "obstacle sampler completion");
    tk_ext_tsk();
}

static bool
sleeping_valid (void)
{
#if OBSTACLE_CALIBRATION_MODE != OBSTACLE_CAL_MODE_OFF
    tm_printf((UB *)"[TEST] OBS-U01 normal scanner disabled by calibration\n");
    return false;
#endif
    T_CFLG flag = {0};
    ID flag_id = tk_cre_flg(&flag);
    if (flag_id < E_OK)
    {
        app_check(flag_id, "obstacle sampler flag");
        return false;
    }
    T_CTSK task = {0};
    task.task = (FP)sleep_sampler;
    task.tskatr = TA_HLNG | TA_RNG3;
    task.itskpri = OBSTACLE_PRI_SCAN + 1;
    task.stksz = TASK_STACK_SIZE;
#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc = TP_PRC1;
#endif
    ID task_id = tk_cre_tsk(&task);
    bool ok = false;
    if (task_id >= E_OK)
    {
        ER started = tk_sta_tsk(task_id, flag_id);
        if (started == E_OK)
        {
            UINT result = 0u;
            ER waited =
                tk_wai_flg(flag_id, 3u, TWF_ORW, &result,
                           (SAMPLES * SAMPLE_GAP_MS) + TEST_MARGIN_MS + 1000u);
            ok = (waited == E_OK) && (result == 1u);
            app_check(waited, "obstacle sampler bounded wait");
            /* E_OBJ means the worker has already become dormant. */
            ER stopped = tk_ter_tsk(task_id);
            if ((stopped != E_OK) && (stopped != E_OBJ))
            {
                app_check(stopped, "obstacle sampler terminate");
                ok = false;
            }
        }
        else
        {
            app_check(started, "obstacle sampler start");
        }
        ER deleted = tk_del_tsk(task_id);
        app_check(deleted, "obstacle sampler delete");
        ok = (deleted == E_OK) && ok;
    }
    else
    {
        app_check(task_id, "obstacle sampler create");
    }
    ER deleted = tk_del_flg(flag_id);
    app_check(deleted, "obstacle sampler flag delete");
    return (deleted == E_OK) && ok;
}

static bool
critical_valid (void)
{
    return obstacle_decision_is_critical(ULTRASONIC_VALID,
                                         OBSTACLE_CRITICAL_ECHO_US - 1u) &&
           !obstacle_decision_is_critical(ULTRASONIC_VALID,
                                          OBSTACLE_CRITICAL_ECHO_US) &&
           !obstacle_decision_is_critical(ULTRASONIC_VALID, UINT16_MAX) &&
           !obstacle_decision_is_critical(ULTRASONIC_NO_ECHO, 0u) &&
           !obstacle_decision_is_critical(ULTRASONIC_FAULT, 0u) &&
           !obstacle_decision_is_critical(ULTRASONIC_PENDING, 1u);
}

static bool
critical_release_valid (void)
{
    return obstacle_decision_blocks_critical_release(
               ULTRASONIC_VALID, OBSTACLE_CRITICAL_RELEASE_ECHO_US - 1u) &&
           !obstacle_decision_blocks_critical_release(
               ULTRASONIC_VALID, OBSTACLE_CRITICAL_RELEASE_ECHO_US) &&
           !obstacle_decision_blocks_critical_release(ULTRASONIC_NO_ECHO, 0u) &&
           !obstacle_decision_blocks_critical_release(ULTRASONIC_FAULT, 0u) &&
           !obstacle_decision_blocks_critical_release(ULTRASONIC_PENDING, 1u);
}

static bool
elapsed_valid (void)
{
    return !obstacle_decision_elapsed_exceeds(1200u, 1000u, 200u) &&
           obstacle_decision_elapsed_exceeds(1201u, 1000u, 200u) &&
           obstacle_decision_elapsed_exceeds(5u, 0xFFFFFFF0u, 20u) &&
           !obstacle_decision_elapsed_exceeds(5u, 0xFFFFFFF0u, 21u);
}

/* All fields come from one synchronized owner snapshot. No driver getters.
 * A poll budget bounds both setup and recovery, even if the owner stops. */
static bool
wait_snapshot (obstacle_scan_test_snapshot_t * p_snapshot, bool hold,
               bool clear, bool sides, uint32_t timeout_ms)
{
#if OBSTACLE_CALIBRATION_MODE != OBSTACLE_CAL_MODE_OFF
    tm_printf((UB *)"[TEST] normal scanner disabled by calibration\n");
    return false;
#endif
    SYSTIM start;
    if (tk_get_otm(&start) != E_OK)
    {
        return false;
    }
    for (;;)
    {
        SYSTIM now;
        if ((tk_get_otm(&now) != E_OK) ||
            ((uint32_t)(now.lo - start.lo) > timeout_ms))
        {
            return false;
        }
        if (obstacle_scan_test_snapshot(p_snapshot) != 0)
        {
            return false;
        }
        if (p_snapshot->initialized && (p_snapshot->hold_requested == hold) &&
            (p_snapshot->hold_acknowledged == hold) &&
            (!clear || (!p_snapshot->stop_requested &&
                        p_snapshot->status == ROBOT_OBSTACLE_CLEAR)) &&
            (!sides || p_snapshot->scanning_sides))
        {
            return true;
        }
        if (tk_dly_tsk(TEST_POLL_MS) != E_OK)
        {
            return false;
        }
    }
}

static bool
release_hold (void)
{
    obstacle_scan_test_snapshot_t snapshot;
    bool released = obstacle_scan_test_request(false) == 0;
    if (!released)
    {
        tm_printf((UB *)"[TEST] obstacle hold release request failed\n");
    }
    bool recovered = released && wait_snapshot(&snapshot, false, true, false,
                                               TEST_RECOVERY_MS);
    if (!recovered)
    {
        tm_printf((UB *)"[TEST] obstacle clear-path recovery timed out\n");
    }
    return recovered;
}

static bool
staleness_valid (void)
{
    obstacle_scan_test_snapshot_t before;
    obstacle_scan_test_snapshot_t after;
    bool ok = false;
    tm_printf((UB *)"[TEST] OBS-U23 keep path clear beyond %lu mm\n",
              (UW)(OBSTACLE_DETECT_DISTANCE_MM + 100u));
    if (wait_snapshot(&before, false, true, false, 5000u) &&
        obstacle_scan_test_request(true) == 0 &&
        wait_snapshot(&before, true, false, false, TEST_ACK_MS) &&
        !before.stop_requested &&
        tk_dly_tsk(OBSTACLE_FORWARD_MAX_AGE_MS + TEST_MARGIN_MS) == E_OK &&
        obstacle_scan_test_snapshot(&after) == 0)
    {
        uint32_t age = after.stale_ms - before.last_usable_centre_ms;
        ok = after.hold_acknowledged && after.stop_requested &&
             after.status == ROBOT_OBSTACLE_UNKNOWN &&
             after.stale_count - before.stale_count == 1u &&
             after.last_usable_centre_ms == before.last_usable_centre_ms &&
             age > OBSTACLE_FORWARD_MAX_AGE_MS &&
             age <= OBSTACLE_FORWARD_MAX_AGE_MS + TEST_MARGIN_MS;
        tm_printf((UB *)"[TEST] OBS-U23 stale event age %lu ms delta %lu\n",
                  (UW)age, (UW)(after.stale_count - before.stale_count));
    }
    else
    {
        tm_printf((UB *)"[TEST] OBS-U23 setup/hold/snapshot failed\n");
    }
    bool recovered = release_hold();
    return recovered && ok;
}

static bool
scan_timeout_valid (void)
{
    obstacle_scan_test_snapshot_t before;
    obstacle_scan_test_snapshot_t after;
    bool ok = false;
    tm_printf((UB *)"[TEST] OBS-U24 place board between %lu and %lu mm "
                    "within 15 s\n",
              (UW)OBSTACLE_CRITICAL_RELEASE_DISTANCE_MM,
              (UW)OBSTACLE_DETECT_DISTANCE_MM);
    if (wait_snapshot(&before, false, false, true, 15000u) &&
        obstacle_scan_test_request(true) == 0 &&
        wait_snapshot(&before, true, false, true, TEST_ACK_MS) &&
        tk_dly_tsk(OBSTACLE_SCAN_TIMEOUT_MS + TEST_MARGIN_MS) == E_OK &&
        obstacle_scan_test_snapshot(&after) == 0)
    {
        uint32_t age = after.timeout_ms - before.side_scan_start_ms;
        ok = after.hold_acknowledged && !after.scanning_sides &&
             after.stop_requested && after.status == ROBOT_OBSTACLE_UNKNOWN &&
             after.timeout_count - before.timeout_count == 1u &&
             age > OBSTACLE_SCAN_TIMEOUT_MS &&
             age <= OBSTACLE_SCAN_TIMEOUT_MS + TEST_MARGIN_MS;
        tm_printf((UB *)"[TEST] OBS-U24 timeout event age %lu ms delta %lu\n",
                  (UW)age, (UW)(after.timeout_count - before.timeout_count));
    }
    else
    {
        tm_printf((UB *)"[TEST] OBS-U24 setup/hold/snapshot failed\n");
    }
    tm_printf((UB *)"[TEST] OBS-U24 remove board; keep path clear\n");
    bool recovered = release_hold();
    return recovered && ok;
}

/* OBS-U16: median of the nonzero samples; zero means no echo. */
static bool
median_valid (void)
{
    const uint16_t mixed[] = {0u, 1166u, 1283u};
    const uint16_t three[] = {1458u, 1399u, 1516u};
    const uint16_t none[] = {0u, 0u, 0u};
    const uint16_t one[] = {0u, 0u, 700u};
    const uint16_t lone_near[] = {0u, 0u, 300u};
    const uint16_t largest[] = {0u, UINT16_MAX, UINT16_MAX};
    const uint16_t single[] = {500u};
    const uint16_t extra[] = {100u, 200u, 300u, 1u};

    return (obstacle_decision_median(mixed, 3u) == 1224u) &&
           (obstacle_decision_median(three, 3u) == 1458u) &&
           (obstacle_decision_median(none, 3u) == 0u) &&
           (obstacle_decision_median(one, 3u) == 700u) &&
           /* A single echo dominates two no-return samples; not a vote. */
           (obstacle_decision_median(lone_near, 3u) == 300u) &&
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
