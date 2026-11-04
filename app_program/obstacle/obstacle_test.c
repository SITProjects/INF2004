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
#include "obstacle_avoid.h"
#include "obstacle_decision.h"
#include "obstacle_scan.h"
#include "obstacle_width.h"

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

/* math.h would redefine the kernel's size_t here; use the compiler's. */
#define TEST_NAN_F __builtin_nanf("")
#define TEST_INFINITY_F __builtin_inff()

/* Width cases: the expected values follow from the fixture's fixed scale. */
#define WIDTH_FIXTURE_SCALE 1715u
#define WIDTH_TOLERANCE_MM 2u
#define CENTRE_EDGE_INDEX ((uint8_t)OBSTACLE_CENTRE_INDEX)
#define LAST_EDGE_INDEX ((uint8_t)(OBSTACLE_SWEEP_POINTS - 1u))

/** One side-selection case: both inputs and the expected decision. */
typedef struct
{
    ultrasonic_result_t left;
    uint16_t left_us;
    ultrasonic_result_t right;
    uint16_t right_us;
    uint16_t centre_us;
    robot_obstacle_status_t status;
    robot_obstacle_direction_t direction;
} side_case_t;

/** Simulated tMotion and scanner around the bypass state machine. */
typedef struct
{
    obstacle_avoid_config_t config;
    obstacle_avoid_t ctx;
    obstacle_avoid_input_t in;
    obstacle_avoid_action_t act;
    bool b_inhibit;        /* INHIBIT as tMotion sees it. */
    bool b_ignore_inhibit; /* tMotion never reports INHIBITED. */
    bool b_hold_inhibited; /* tMotion still reports INHIBITED after clear. */
    bool b_send_busy;      /* Sends return E_TMOUT. */
    bool b_auto;           /* Answer PREPARE, RUN_LEG and VERIFY. */
    uint32_t sends;        /* Commands queued. */
    uint32_t last_cmd;     /* Last command queued. */
} avoid_sim_t;

/* Static data. */

/* The initial task's stack is 1 KB: keep scripts and snapshots here. Only
 * the initial task runs these tests, one at a time. */
static obstacle_scan_test_reading_t
    script_buffer[OBSTACLE_SCAN_TEST_SCRIPT_MAX];
static obstacle_scan_test_snapshot_t snap_before;
static obstacle_scan_test_snapshot_t snap_after;
static obstacle_scan_test_snapshot_t snap_later;
static obstacle_scan_test_snapshot_t snap_cleared;
static avoid_sim_t sim;

/* Private function declarations. */
static bool packed_obstacle_fields_valid (void);
static bool median_valid (void);
static bool position_valid (void);
static bool centre_valid (void);
static bool sides_valid (void);
static bool sides_agree_valid (void);
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
static obstacle_width_config_t width_fixture (float correction_deg);
static bool width_matches (obstacle_width_result_t width, uint16_t width_mm,
                           uint8_t left, uint8_t right, bool b_left_found,
                           bool b_right_found);
static bool width_unavailable (obstacle_width_result_t width);
static bool classify_valid (void);
static bool width_wall_valid (void);
static bool width_box_valid (void);
static bool width_shifted_valid (void);
static bool width_pole_valid (void);
static bool width_limits_valid (void);
static bool width_offsets_valid (void);
static bool range_valid (void);
static bool geometry_persists_valid (void);
static bool wait_result (obstacle_scan_test_snapshot_t * p_snapshot,
                         robot_obstacle_status_t status, uint8_t after_sequence,
                         uint32_t timeout_ms);
static obstacle_scan_test_reading_t reading (ultrasonic_result_t measurement,
                                             uint16_t echo_us);
static bool rearm_valid (void);
static bool rearm_case (bool b_during_sides);
static bool start_side_scan (obstacle_scan_test_snapshot_t * p_snapshot);
static bool wait_command (obstacle_scan_test_snapshot_t * p_snapshot);
static bool corridor_valid (void);
static bool corridor_refused_valid (void);
static obstacle_avoid_geometry_t avoid_fixture (void);
static robot_obstacle_result_t path_result (robot_obstacle_direction_t dir,
                                            bool b_left_found,
                                            bool b_right_found);
static obstacle_scan_geometry_t
geometry_at (uint16_t centre_mm, int16_t left_x_mm, int16_t right_x_mm);
static bool plan_is (obstacle_avoid_plan_t plan, bool b_gap_right,
                     bool b_edge_unknown, uint16_t sidestep_mm,
                     uint16_t pass_mm);
static bool plan_left_valid (void);
static bool plan_right_valid (void);
static bool plan_limits_valid (void);
static bool plan_refusals_valid (void);
static obstacle_scan_corridor_t empty_corridor (void);
static bool corridor_check_valid (void);
static obstacle_avoid_config_t avoid_config (void);
static void sim_scan (robot_obstacle_status_t status,
                      robot_obstacle_direction_t dir, bool b_stop,
                      bool b_critical);
static void sim_corridor (bool b_hit_ahead);
static void sim_reset (void);
static obstacle_avoid_action_t sim_step (void);
static bool sim_until (obstacle_avoid_state_t state, obstacle_avoid_leg_t leg,
                       uint32_t max_steps);
static bool sim_stays (obstacle_avoid_state_t state, uint32_t steps);
static bool sim_detect_path (void);
static bool avoid_stop_valid (void);
static bool avoid_send_valid (void);
static bool avoid_rearm_valid (void);
static bool avoid_verify_valid (void);
static bool avoid_line_valid (void);
static bool sim_in_sidestep (void);
static bool avoid_fault_valid (void);
static bool avoid_obstacles_valid (void);

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

    TEST_CHECK("OBS-U05", classify_valid(),
               "sweep classify: HIT, MISS, INVALID without echo wrap");
    TEST_CHECK("OBS-U06", width_wall_valid(),
               "width: wall past both ends is a 346 mm lower bound");
    TEST_CHECK("OBS-U07", width_box_valid(),
               "width: centred 200 mm box at 250 mm reads 204 mm");
    TEST_CHECK("OBS-U08", width_shifted_valid(),
               "width: shifted box, left not found, 155 mm lower bound");
    TEST_CHECK("OBS-U09", width_pole_valid(),
               "width: 20 mm pole reads 22 mm; over-correction collapses");
    TEST_CHECK("OBS-U11", width_limits_valid(),
               "width: saturates at 511 mm; invalid input unavailable");
    TEST_CHECK("OBS-U35", plan_left_valid(),
               "plan left: sidestep and pass from the formulas");
    TEST_CHECK("OBS-U36", plan_right_valid(),
               "plan right: mirror of left using -right_x");
    TEST_CHECK("OBS-U37", plan_limits_valid(),
               "plan limits: refuse, never clamp down; unknown edge is max");
    TEST_CHECK("OBS-U38", plan_refusals_valid(),
               "plan refusals: not PATH_FOUND to a side, or no geometry");
    TEST_CHECK("OBS-U39", width_offsets_valid(),
               "width: centred box edges at +/-102 mm; unavailable is 0");
    TEST_CHECK("OBS-U40", range_valid(),
               "range: from the pivot, radial offset added once");
    TEST_CHECK("OBS-U41", corridor_check_valid(),
               "corridor: clear, shift to the far side, limits, invalid");
    TEST_CHECK("OBS-U42", avoid_stop_valid(),
               "avoid: held until clear at start-up; stop and its ack");
    TEST_CHECK("OBS-U43", avoid_send_valid(),
               "avoid: no send while inhibited; pending; kernel faults");
    TEST_CHECK("OBS-U44", avoid_rearm_valid(),
               "avoid: latched stop across turns; fresh result after rearm");
    TEST_CHECK("OBS-U45", avoid_verify_valid(),
               "avoid: extra sidesteps lengthen the return; limit holds");
    TEST_CHECK("OBS-U46", avoid_line_valid(),
               "avoid: line ends the return early; one search, then FAULT");
    TEST_CHECK("OBS-U47", avoid_fault_valid(),
               "avoid: one retry from odometry; FAULT never releases");
    TEST_CHECK("OBS-U48", avoid_obstacles_valid(),
               "avoid: blocked moves resume; turns ignore stop; full box");

    TEST_CHECK("OBS-U10", packed_obstacle_fields_valid(),
               "packed obstacle fields clamp without changing neighbours");

    TEST_CHECK("OBS-U16", median_valid(),
               "ping median ignores no-echo samples and averages two");

    TEST_CHECK("OBS-U17", position_valid(),
               "position result is FAULT, NO_ECHO or VALID");

    TEST_CHECK("OBS-U18", centre_valid(),
               "centre: no echo or far is CLEAR, near scans, fault UNKNOWN");

    TEST_CHECK("OBS-U19", sides_valid(),
               "sides: miss the obstacle, farther wins, ties go left");
    TEST_CHECK("OBS-U52", sides_agree_valid(),
               "side clearance agrees with width MISS and threshold");

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
    TEST_CHECK("OBS-U49", geometry_persists_valid(),
               "scripted sweep: geometry kept with PATH_FOUND until next");
    TEST_CHECK("OBS-U50", rearm_valid(),
               "rearm: new SCANNING, stop latched, full fresh group to clear");
    TEST_CHECK("OBS-U51", corridor_valid() && corridor_refused_valid(),
               "corridor: 13 medians and mask, refused during side scan");

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

static obstacle_width_config_t
width_fixture (float correction_deg)
{
    obstacle_width_config_t config = {.scale_mm_per_10k = WIDTH_FIXTURE_SCALE,
                                      .echo_offset_mm = 0,
                                      .radial_offset_mm = 0.0f,
                                      .edge_correction_deg = correction_deg};
    return config;
}

static bool
width_matches (obstacle_width_result_t width, uint16_t width_mm, uint8_t left,
               uint8_t right, bool b_left_found, bool b_right_found)
{
    return (width.width_mm + WIDTH_TOLERANCE_MM >= width_mm) &&
           (width.width_mm <= width_mm + WIDTH_TOLERANCE_MM) &&
           (robot_obstacle_get_left_edge(width.edges) == left) &&
           (robot_obstacle_get_right_edge(width.edges) == right) &&
           (width.b_left_found == b_left_found) &&
           (width.b_right_found == b_right_found);
}

static bool
width_unavailable (obstacle_width_result_t width)
{
    return (width.width_mm == ROBOT_OBSTACLE_WIDTH_UNAVAILABLE) &&
           (robot_obstacle_get_left_edge(width.edges) ==
            ROBOT_OBSTACLE_EDGE_NONE) &&
           (robot_obstacle_get_right_edge(width.edges) ==
            ROBOT_OBSTACLE_EDGE_NONE) &&
           !width.b_left_found && !width.b_right_found && !width.b_collapsed;
}

/* OBS-U05: centre 1458 us gives a HIT limit of 1924 us (exclusive). */
static bool
classify_valid (void)
{
    return (obstacle_width_classify(ULTRASONIC_VALID, 1923u, 1458u) ==
            OBSTACLE_WIDTH_HIT) &&
           (obstacle_width_classify(ULTRASONIC_VALID, 1924u, 1458u) ==
            OBSTACLE_WIDTH_MISS) &&
           (obstacle_width_classify(ULTRASONIC_VALID, 1u, 1458u) ==
            OBSTACLE_WIDTH_HIT) &&
           (obstacle_width_classify(ULTRASONIC_NO_ECHO, 0u, 1458u) ==
            OBSTACLE_WIDTH_MISS) &&
           (obstacle_width_classify(ULTRASONIC_FAULT, 0u, 1458u) ==
            OBSTACLE_WIDTH_INVALID) &&
           (obstacle_width_classify(ULTRASONIC_PENDING, 1u, 1458u) ==
            OBSTACLE_WIDTH_INVALID) &&
           /* 65500 + 466 exceeds 16 bits; the sum must not wrap. */
           (obstacle_width_classify(ULTRASONIC_VALID, UINT16_MAX, 65500u) ==
            OBSTACLE_WIDTH_HIT);
}

/* OBS-U06: flat wall 300 mm ahead covers both ends (346 mm at 60/120). */
static bool
width_wall_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    obstacle_width_edge_t left = {.index = 0u, .echo_us = 2020u};
    obstacle_width_edge_t right = {.index = LAST_EDGE_INDEX, .echo_us = 2020u};
    return width_matches(obstacle_width_estimate(left, right, &config), 346u,
                         0u, LAST_EDGE_INDEX, false, false);
}

/* OBS-U07: 200 mm box centred 250 mm ahead hits 70..110 degrees. */
static bool
width_box_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    obstacle_width_edge_t left = {.index = 2u, .echo_us = 1551u};
    obstacle_width_edge_t right = {.index = 10u, .echo_us = 1551u};
    obstacle_width_result_t width =
        obstacle_width_estimate(left, right, &config);
    return width_matches(width, 204u, 2u, 10u, true, true) &&
           (width.left_bound_ddeg == 675u) && (width.right_bound_ddeg == 1125u);
}

/* OBS-U08: the same box 80 mm left continues past 60 degrees. */
static bool
width_shifted_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    obstacle_width_edge_t left = {.index = 0u, .echo_us = 1683u};
    obstacle_width_edge_t right = {.index = CENTRE_EDGE_INDEX,
                                   .echo_us = 1458u};
    return width_matches(obstacle_width_estimate(left, right, &config), 155u,
                         0u, CENTRE_EDGE_INDEX, false, true);
}

/* OBS-U09: a 20 mm pole at 250 mm hits only the centre. */
static bool
width_pole_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    obstacle_width_edge_t centre = {.index = CENTRE_EDGE_INDEX,
                                    .echo_us = 1458u};
    obstacle_width_result_t width =
        obstacle_width_estimate(centre, centre, &config);
    if (!width_matches(width, 22u, CENTRE_EDGE_INDEX, CENTRE_EDGE_INDEX, true,
                       true) ||
        width.b_collapsed)
    {
        return false;
    }
    /* 5 degrees inward from 87.5/92.5 crosses: both move to 90 degrees. */
    config = width_fixture(5.0f);
    width = obstacle_width_estimate(centre, centre, &config);
    return (width.width_mm == 1u) && width.b_collapsed &&
           (width.left_bound_ddeg == 900u) && (width.right_bound_ddeg == 900u);
}

/* OBS-U11: saturation and every rejected input. */
static bool
width_limits_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    obstacle_width_edge_t far_left = {.index = 0u, .echo_us = 3499u};
    obstacle_width_edge_t far_right = {.index = LAST_EDGE_INDEX,
                                       .echo_us = 3499u};
    if (obstacle_width_estimate(far_left, far_right, &config).width_mm !=
        ROBOT_OBSTACLE_WIDTH_MAX_MM)
    {
        return false;
    }

    obstacle_width_edge_t centre = {.index = CENTRE_EDGE_INDEX,
                                    .echo_us = 1458u};
    obstacle_width_edge_t left_past_centre = {.index = CENTRE_EDGE_INDEX + 1u,
                                              .echo_us = 1458u};
    obstacle_width_edge_t right_past_centre = {.index = CENTRE_EDGE_INDEX - 1u,
                                               .echo_us = 1458u};
    obstacle_width_edge_t beyond = {.index = LAST_EDGE_INDEX + 1u,
                                    .echo_us = 1458u};
    obstacle_width_edge_t silent = {.index = CENTRE_EDGE_INDEX, .echo_us = 0u};
    if (!width_unavailable(
            obstacle_width_estimate(left_past_centre, centre, &config)) ||
        !width_unavailable(
            obstacle_width_estimate(centre, right_past_centre, &config)) ||
        !width_unavailable(obstacle_width_estimate(centre, beyond, &config)) ||
        !width_unavailable(obstacle_width_estimate(silent, centre, &config)) ||
        !width_unavailable(obstacle_width_estimate(centre, centre, NULL)) ||
        !width_unavailable(obstacle_width_unavailable()))
    {
        return false;
    }

    const float bad_correction[] = {TEST_NAN_F, TEST_INFINITY_F, -1.0f, 16.0f};
    for (unsigned int index = 0u; index < ARRAY_COUNT(bad_correction); ++index)
    {
        config = width_fixture(bad_correction[index]);
        if (!width_unavailable(
                obstacle_width_estimate(centre, centre, &config)))
        {
            return false;
        }
    }
    config = width_fixture(0.0f);
    config.scale_mm_per_10k = 0u;
    if (!width_unavailable(obstacle_width_estimate(centre, centre, &config)))
    {
        return false;
    }
    /* 1458 us is 250 mm; a -500 mm offset leaves no positive range. */
    config = width_fixture(0.0f);
    config.radial_offset_mm = -500.0f;
    if (!width_unavailable(obstacle_width_estimate(centre, centre, &config)))
    {
        return false;
    }
    config.radial_offset_mm = TEST_INFINITY_F;
    return width_unavailable(obstacle_width_estimate(centre, centre, &config));
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
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_VALID, NEAR_US, 0u,
         ROBOT_OBSTACLE_BLOCKED, ROBOT_OBSTACLE_DIR_NONE},
        /* Exactly at the threshold counts as clear. */
        {ULTRASONIC_VALID, THRESHOLD_US, ULTRASONIC_VALID, NEAR_US, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_VALID, THRESHOLD_US, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        /* Equal clear distances select left. */
        {ULTRASONIC_VALID, CLEAR_US, ULTRASONIC_VALID, CLEAR_US, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* The farther clear side wins. */
        {ULTRASONIC_VALID, CLEAR_US, ULTRASONIC_VALID, FARTHER_US, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_VALID, FARTHER_US, ULTRASONIC_VALID, CLEAR_US, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* No echo ranks beyond any finite echo. */
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_VALID, UINT16_MAX, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        {ULTRASONIC_VALID, UINT16_MAX, ULTRASONIC_NO_ECHO, 0u, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_NO_ECHO, 0u, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* A fault with no clear side is UNKNOWN, not BLOCKED. */
        {ULTRASONIC_FAULT, 0u, ULTRASONIC_VALID, NEAR_US, 0u,
         ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE},
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_FAULT, 0u, 0u,
         ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE},
        {ULTRASONIC_FAULT, 0u, ULTRASONIC_FAULT, 0u, 0u, ROBOT_OBSTACLE_UNKNOWN,
         ROBOT_OBSTACLE_DIR_NONE},
        /* A clear side is still chosen when the other side faulted. */
        {ULTRASONIC_FAULT, 0u, ULTRASONIC_VALID, CLEAR_US, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_FAULT, 0u, 0u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        /* A square-on wall near detection still occupies both sides. */
        {ULTRASONIC_VALID, (uint16_t)((uint32_t)NEAR_US * 1155u / 1000u),
         ULTRASONIC_VALID, (uint16_t)((uint32_t)NEAR_US * 1155u / 1000u),
         NEAR_US, ROBOT_OBSTACLE_BLOCKED, ROBOT_OBSTACLE_DIR_NONE},
        /* The jump boundary is inclusive, on either side. */
        {ULTRASONIC_VALID, NEAR_US + OBSTACLE_EDGE_JUMP_US, ULTRASONIC_VALID,
         NEAR_US, NEAR_US, ROBOT_OBSTACLE_PATH_FOUND,
         ROBOT_OBSTACLE_DIR_LEFT_60},
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_VALID,
         NEAR_US + OBSTACLE_EDGE_JUMP_US, NEAR_US, ROBOT_OBSTACLE_PATH_FOUND,
         ROBOT_OBSTACLE_DIR_RIGHT_120},
        {ULTRASONIC_VALID, NEAR_US + OBSTACLE_EDGE_JUMP_US - 1u,
         ULTRASONIC_VALID, NEAR_US, NEAR_US, ROBOT_OBSTACLE_BLOCKED,
         ROBOT_OBSTACLE_DIR_NONE},
        /* A tiny centre echo leaves the detection threshold in charge. */
        {ULTRASONIC_VALID, NEAR_US, ULTRASONIC_VALID, NEAR_US, 1u,
         ROBOT_OBSTACLE_BLOCKED, ROBOT_OBSTACLE_DIR_NONE},
        /* A large centre must not wrap the jump into a clear side. */
        {ULTRASONIC_VALID, UINT16_MAX, ULTRASONIC_VALID, UINT16_MAX,
         UINT16_MAX - 1u, ROBOT_OBSTACLE_BLOCKED, ROBOT_OBSTACLE_DIR_NONE},
        {ULTRASONIC_NO_ECHO, 0u, ULTRASONIC_VALID, UINT16_MAX, UINT16_MAX - 1u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60},
        {ULTRASONIC_VALID, UINT16_MAX, ULTRASONIC_NO_ECHO, 0u, UINT16_MAX - 1u,
         ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_RIGHT_120},
    };

    for (unsigned int index = 0u; index < ARRAY_COUNT(cases); ++index)
    {
        obstacle_side_decision_t decision = obstacle_decision_sides(
            cases[index].left, cases[index].left_us, cases[index].right,
            cases[index].right_us, cases[index].centre_us);
        if ((decision.status != cases[index].status) ||
            (decision.direction != cases[index].direction))
        {
            tm_printf((UB *)"obstacle_test: OBS-U19 case %u failed\n", index);
            return false;
        }
    }
    return true;
}

/* OBS-U52: both sides agree with width classification at both boundaries.
 * Include sums beyond UINT16_MAX; no wrapped sample is tested as valid. */
static bool
sides_agree_valid (void)
{
    static const uint16_t centres[] = {0u,
                                       1u,
                                       THRESHOLD_US / 2u,
                                       NEAR_US,
                                       THRESHOLD_US,
                                       UINT16_MAX - OBSTACLE_EDGE_JUMP_US,
                                       UINT16_MAX - OBSTACLE_EDGE_JUMP_US + 1u,
                                       UINT16_MAX};

    for (unsigned int c = 0u; c < ARRAY_COUNT(centres); ++c)
    {
        uint32_t const jump = (uint32_t)centres[c] + OBSTACLE_EDGE_JUMP_US;
        uint32_t const echoes[] = {
            0u,         1u,        NEAR_US, THRESHOLD_US, CLEAR_US,
            FARTHER_US, jump - 1u, jump,    jump + 1u,    UINT16_MAX};
        for (unsigned int e = 0u; e < ARRAY_COUNT(echoes); ++e)
        {
            if (echoes[e] > UINT16_MAX)
            {
                continue;
            }
            uint16_t const echo = (uint16_t)echoes[e];
            bool const b_clear =
                (echo >= OBSTACLE_DETECT_ECHO_US) &&
                (obstacle_width_classify(ULTRASONIC_VALID, echo, centres[c]) ==
                 OBSTACLE_WIDTH_MISS);
            obstacle_side_decision_t const left = obstacle_decision_sides(
                ULTRASONIC_VALID, echo, ULTRASONIC_FAULT, 0u, centres[c]);
            obstacle_side_decision_t const right = obstacle_decision_sides(
                ULTRASONIC_FAULT, 0u, ULTRASONIC_VALID, echo, centres[c]);
            robot_obstacle_status_t const expected =
                b_clear ? ROBOT_OBSTACLE_PATH_FOUND : ROBOT_OBSTACLE_UNKNOWN;
            if ((left.status != expected) || (right.status != expected) ||
                (left.direction != (b_clear ? ROBOT_OBSTACLE_DIR_LEFT_60
                                            : ROBOT_OBSTACLE_DIR_NONE)) ||
                (right.direction != (b_clear ? ROBOT_OBSTACLE_DIR_RIGHT_120
                                             : ROBOT_OBSTACLE_DIR_NONE)))
            {
                tm_printf(
                    (UB *)"obstacle_test: OBS-U52 centre %u echo %u failed\n",
                    (unsigned int)centres[c], (unsigned int)echo);
                return false;
            }
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

/* OBS-U39: the OBS-U07 box (edges 67.5 and 112.5 deg at 266 mm) lies
 * 266 * cos(67.5 deg) = 102 mm either side of the pivot. */
static bool
width_offsets_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    obstacle_width_edge_t left = {.index = 2u, .echo_us = 1551u};
    obstacle_width_edge_t right = {.index = 10u, .echo_us = 1551u};
    obstacle_width_result_t width =
        obstacle_width_estimate(left, right, &config);
    obstacle_width_result_t none = obstacle_width_unavailable();
    return (width.left_x_mm >= 99) && (width.left_x_mm <= 105) &&
           (width.right_x_mm <= -99) && (width.right_x_mm >= -105) &&
           (none.left_x_mm == 0) && (none.right_x_mm == 0);
}

/* OBS-U40: 1551 us is 266.0 mm at the fixture scale; a 20 mm radial
 * offset gives 286.0 mm from the pivot, not 306.0 mm. */
static bool
range_valid (void)
{
    obstacle_width_config_t config = width_fixture(0.0f);
    config.radial_offset_mm = 20.0f;
    float range_mm = 0.0f;
    bool b_ok = obstacle_width_range_mm(1551u, &config, &range_mm) &&
                (range_mm > 285.5f) && (range_mm < 286.5f);

    float untouched_mm = -1.0f;
    b_ok = b_ok && !obstacle_width_range_mm(0u, &config, &untouched_mm) &&
           !obstacle_width_range_mm(1551u, NULL, &untouched_mm) &&
           !obstacle_width_range_mm(1551u, &config, NULL);
    config.echo_offset_mm = -400;
    b_ok = b_ok && !obstacle_width_range_mm(100u, &config, &untouched_mm);
    return b_ok && (untouched_mm == -1.0f);
}

static obstacle_scan_test_reading_t
reading (ultrasonic_result_t measurement, uint16_t echo_us)
{
    obstacle_scan_test_reading_t entry = {.measurement = measurement,
                                          .echo_us = echo_us};
    return entry;
}

/* Wait for a published status whose sequence differs from after_sequence. */
static bool
wait_result (obstacle_scan_test_snapshot_t * p_snapshot,
             robot_obstacle_status_t status, uint8_t after_sequence,
             uint32_t timeout_ms)
{
    SYSTIM start;
    if (tk_get_otm(&start) != E_OK)
    {
        return false;
    }
    for (;;)
    {
        SYSTIM now;
        if ((tk_get_otm(&now) != E_OK) ||
            ((uint32_t)(now.lo - start.lo) > timeout_ms) ||
            (obstacle_scan_test_snapshot(p_snapshot) != 0))
        {
            return false;
        }
        if ((robot_obstacle_get_status(p_snapshot->result.flags) == status) &&
            (p_snapshot->result.sequence != after_sequence))
        {
            return true;
        }
        if (tk_dly_tsk(TEST_POLL_MS) != E_OK)
        {
            return false;
        }
    }
}

/* OBS-U49: a scripted sweep (near centre, both ends no echo, fine hits at
 * 115 and 65 degrees) publishes PATH_FOUND left with its geometry. The
 * geometry outlives reset_sweep() and is replaced by the next publish. */
static bool
geometry_persists_valid (void)
{
    obstacle_scan_test_snapshot_t * const p_before = &snap_before;
    obstacle_scan_test_snapshot_t * const p_found = &snap_after;
    obstacle_scan_test_snapshot_t * const p_later = &snap_later;
    obstacle_scan_test_snapshot_t * const p_cleared = &snap_cleared;
    obstacle_scan_test_reading_t * const script = script_buffer;
    uint8_t count = 0u;
    for (uint8_t index = 0u; index < 3u; ++index)
    {
        script[count++] = reading(ULTRASONIC_VALID, NEAR_US);
    }
    for (uint8_t index = 0u; index < 6u; ++index)
    {
        script[count++] = reading(ULTRASONIC_NO_ECHO, 0u);
    }
    script[count++] = reading(ULTRASONIC_VALID, NEAR_US);
    script[count++] = reading(ULTRASONIC_VALID, NEAR_US);
    for (uint8_t index = 0u; index < 3u; ++index)
    {
        script[count++] = reading(ULTRASONIC_VALID, CLEAR_US);
    }

    obstacle_width_config_t config = {
        .scale_mm_per_10k = OBSTACLE_SOUND_MM_PER_10K_US,
        .echo_offset_mm = OBSTACLE_ECHO_OFFSET_MM,
        .radial_offset_mm = OBSTACLE_SENSOR_RADIAL_OFFSET_MM,
        .edge_correction_deg = OBSTACLE_EDGE_CORRECTION_DEG};
    float centre_mm = 0.0f;
    tm_printf((UB *)"[TEST] OBS-U49 keep path clear beyond %lu mm\n",
              (UW)(OBSTACLE_DETECT_DISTANCE_MM + 100u));
    bool ok = obstacle_width_range_mm(NEAR_US, &config, &centre_mm) &&
              wait_snapshot(p_before, false, true, false, 5000u) &&
              (obstacle_scan_test_script(script, count) == 0) &&
              wait_result(p_found, ROBOT_OBSTACLE_PATH_FOUND,
                          p_before->result.sequence,
                          OBSTACLE_SCAN_TIMEOUT_MS + TEST_MARGIN_MS) &&
              (tk_dly_tsk(TEST_POLL_MS) == E_OK) &&
              (obstacle_scan_test_snapshot(p_later) == 0);
    if (ok)
    {
        int32_t centre_error =
            (int32_t)p_found->geometry.centre_mm - (int32_t)(centre_mm + 0.5f);
        tm_printf((UB *)"[TEST] OBS-U49 centre %u mm x %d..%d mm\n",
                  (unsigned int)p_found->geometry.centre_mm,
                  (int)p_found->geometry.left_x_mm,
                  (int)p_found->geometry.right_x_mm);
        ok = (robot_obstacle_get_direction(p_found->result.flags) ==
              ROBOT_OBSTACLE_DIR_LEFT_60) &&
             (centre_error >= -1) && (centre_error <= 1) &&
             (p_found->geometry.left_x_mm > 0) &&
             (p_found->geometry.right_x_mm < 0) &&
             ((p_later->result.sequence != p_found->result.sequence) ||
              ((p_later->geometry.centre_mm == p_found->geometry.centre_mm) &&
               (p_later->geometry.left_x_mm == p_found->geometry.left_x_mm) &&
               (p_later->geometry.right_x_mm == p_found->geometry.right_x_mm)));
    }
    ok = ok &&
         wait_result(p_cleared, ROBOT_OBSTACLE_CLEAR, p_found->result.sequence,
                     OBSTACLE_SCAN_TIMEOUT_MS + TEST_MARGIN_MS) &&
         (p_cleared->geometry.centre_mm == 0u) &&
         (p_cleared->geometry.left_x_mm == 0) &&
         (p_cleared->geometry.right_x_mm == 0) &&
         (p_cleared->script_remaining == 0u);
    if (!ok)
    {
        tm_printf((UB *)"[TEST] OBS-U49 setup, sweep or check failed\n");
    }
    return (obstacle_scan_test_script(NULL, 0u) == 0) && ok;
}

/* Wait until the scan task has run the last test command. */
static bool
wait_command (obstacle_scan_test_snapshot_t * p_snapshot)
{
    for (uint32_t waited_ms = 0u; waited_ms <= TEST_ACK_MS;
         waited_ms += TEST_POLL_MS)
    {
        if (obstacle_scan_test_snapshot(p_snapshot) != 0)
        {
            return false;
        }
        if (!p_snapshot->command_pending)
        {
            return true;
        }
        if (tk_dly_tsk(TEST_POLL_MS) != E_OK)
        {
            return false;
        }
    }
    return false;
}

/* From a clear path, script three near pings and wait for the side scan. */
static bool
start_side_scan (obstacle_scan_test_snapshot_t * p_snapshot)
{
    obstacle_scan_test_reading_t near[3] = {reading(ULTRASONIC_VALID, NEAR_US),
                                            reading(ULTRASONIC_VALID, NEAR_US),
                                            reading(ULTRASONIC_VALID, NEAR_US)};
    return wait_snapshot(p_snapshot, false, true, false, 5000u) &&
           (obstacle_scan_test_script(near, 3u) == 0) &&
           wait_snapshot(p_snapshot, false, false, true, TEST_RECOVERY_MS);
}

/* One rearm: SCANNING with the returned sequence and the stop latched at
 * once; CLEAR only after all three scripted clear pings were used. */
static bool
rearm_case (bool b_during_sides)
{
    obstacle_scan_test_snapshot_t * const p_before = &snap_before;
    obstacle_scan_test_snapshot_t * const p_after = &snap_after;
    obstacle_scan_test_snapshot_t * const p_cleared = &snap_cleared;
    obstacle_scan_test_reading_t clear[3] = {
        reading(ULTRASONIC_VALID, CLEAR_US),
        reading(ULTRASONIC_VALID, CLEAR_US),
        reading(ULTRASONIC_VALID, CLEAR_US)};
    bool ok = b_during_sides
                  ? start_side_scan(p_before)
                  : wait_snapshot(p_before, false, true, false, 5000u);
    ok = ok &&
         (obstacle_scan_test_command(OBSTACLE_SCAN_TEST_REARM, clear, 3u) ==
          0) &&
         wait_command(p_after) &&
         (robot_obstacle_get_status(p_after->result.flags) ==
          ROBOT_OBSTACLE_SCANNING) &&
         p_after->stop_requested && !p_after->scanning_sides &&
         (p_after->result.sequence == p_after->rearm_sequence) &&
         (p_after->result.sequence != p_before->result.sequence) &&
         wait_result(p_cleared, ROBOT_OBSTACLE_CLEAR, p_after->rearm_sequence,
                     TEST_RECOVERY_MS) &&
         !p_cleared->stop_requested && (p_cleared->script_remaining == 0u);
    if (!ok)
    {
        tm_printf((UB *)"[TEST] OBS-U50 rearm %s failed\n",
                  b_during_sides ? "during side scan" : "while clear");
    }
    return ok;
}

/* OBS-U50: rearm while clear and during a side scan. */
static bool
rearm_valid (void)
{
    tm_printf((UB *)"[TEST] OBS-U50 keep path clear beyond %lu mm\n",
              (UW)(OBSTACLE_DETECT_DISTANCE_MM + 100u));
    bool ok = rearm_case(false);
    ok = rearm_case(true) && ok;
    return (obstacle_scan_test_script(NULL, 0u) == 0) && ok;
}

/* OBS-U51: echoes 1000 + 20 * i us, no echo at index 3, and index 6
 * faulted past its retries (three FAULTs, then two pings). */
static bool
corridor_valid (void)
{
    obstacle_scan_test_snapshot_t * const p_before = &snap_before;
    obstacle_scan_test_snapshot_t * const p_after = &snap_after;
    obstacle_scan_test_reading_t * const script = script_buffer;
    uint8_t count = 0u;
    for (uint8_t index = 0u; index < LAST_EDGE_INDEX + 1u; ++index)
    {
        uint16_t echo_us = (uint16_t)(1000u + (20u * index));
        if (index == 3u)
        {
            for (uint8_t ping = 0u; ping < OBSTACLE_PINGS_PER_ANGLE; ++ping)
            {
                script[count++] = reading(ULTRASONIC_NO_ECHO, 0u);
            }
            continue;
        }
        if (index == 6u)
        {
            for (uint8_t fault = 0u; fault <= OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE;
                 ++fault)
            {
                script[count++] = reading(ULTRASONIC_FAULT, 0u);
            }
            for (uint8_t ping = 1u; ping < OBSTACLE_PINGS_PER_ANGLE; ++ping)
            {
                script[count++] = reading(ULTRASONIC_VALID, echo_us);
            }
            continue;
        }
        for (uint8_t ping = 0u; ping < OBSTACLE_PINGS_PER_ANGLE; ++ping)
        {
            script[count++] = reading(ULTRASONIC_VALID, echo_us);
        }
    }

    tm_printf((UB *)"[TEST] OBS-U51 keep path clear; corridor takes up to "
                    "%lu ms\n",
              (UW)OBSTACLE_CORRIDOR_WORST_MS);
    bool ok = wait_snapshot(p_before, false, true, false, 5000u) &&
              (obstacle_scan_test_command(OBSTACLE_SCAN_TEST_CORRIDOR, script,
                                          count) == 0) &&
              wait_command(p_after) && p_after->stop_requested;
    for (uint32_t waited_ms = 0u;
         ok && (p_after->corridor.sequence == p_before->corridor.sequence);
         waited_ms += TEST_POLL_MS)
    {
        ok = (waited_ms <= OBSTACLE_CORRIDOR_TIMEOUT_MS) &&
             (tk_dly_tsk(TEST_POLL_MS) == E_OK) &&
             (obstacle_scan_test_snapshot(p_after) == 0);
    }
    uint16_t expected_mask =
        (uint16_t)(((1u << OBSTACLE_SWEEP_POINTS) - 1u) & ~(1u << 6u));
    ok = ok &&
         (p_after->corridor.sequence ==
          (uint8_t)(p_before->corridor.sequence + 1u)) &&
         (p_after->corridor.valid_mask == expected_mask) &&
         (p_after->script_remaining == 0u);
    for (uint8_t index = 0u; ok && (index < LAST_EDGE_INDEX + 1u); ++index)
    {
        uint16_t expected_us = ((index == 3u) || (index == 6u))
                                   ? 0u
                                   : (uint16_t)(1000u + (20u * index));
        ok = p_after->corridor.echo_us[index] == expected_us;
    }
    if (!ok)
    {
        tm_printf((UB *)"[TEST] OBS-U51 corridor seq %u mask 0x%04X\n",
                  (unsigned int)p_after->corridor.sequence,
                  (unsigned int)p_after->corridor.valid_mask);
    }
    /* The corridor rearms afterwards; wait for the clear path to return. */
    return wait_snapshot(p_after, false, true, false, TEST_RECOVERY_MS) && ok;
}

/* OBS-U51: a corridor request during a side scan is refused. */
static bool
corridor_refused_valid (void)
{
    obstacle_scan_test_snapshot_t * const p_before = &snap_before;
    obstacle_scan_test_snapshot_t * const p_after = &snap_after;
    bool ok = start_side_scan(p_before) &&
              (obstacle_scan_test_command(OBSTACLE_SCAN_TEST_CORRIDOR, NULL,
                                          0u) == 0) &&
              wait_command(p_after) &&
              (p_after->corridor_refused == p_before->corridor_refused + 1u) &&
              (p_after->corridor.sequence == p_before->corridor.sequence);
    bool b_rearmed =
        (obstacle_scan_test_command(OBSTACLE_SCAN_TEST_REARM, NULL, 0u) == 0) &&
        wait_command(p_after) &&
        wait_snapshot(p_after, false, true, false, TEST_RECOVERY_MS);
    if (!ok)
    {
        tm_printf((UB *)"[TEST] OBS-U51 refusal check failed\n");
    }
    return b_rearmed && ok;
}

/* Fixed car for the pure bypass tests, independent of the config. */
static obstacle_avoid_geometry_t
avoid_fixture (void)
{
    obstacle_avoid_geometry_t geometry = {.half_width_mm = 75u,
                                          .margin_mm = 60u,
                                          .axle_to_pivot_mm = 80u,
                                          .rear_mm = 60u,
                                          .depth_mm = 200u,
                                          .sidestep_min_mm = 100u,
                                          .sidestep_max_mm = 400u,
                                          .lateral_max_mm = 500u,
                                          .pass_max_mm = 1500u,
                                          .return_extra_mm = 150u,
                                          .overshoot_mm = 40u};
    return geometry;
}

static robot_obstacle_result_t
path_result (robot_obstacle_direction_t dir, bool b_left_found,
             bool b_right_found)
{
    robot_obstacle_result_t result = {
        .flags = robot_obstacle_pack_flags(ROBOT_OBSTACLE_PATH_FOUND, dir,
                                           b_left_found, b_right_found),
        .edges = robot_obstacle_pack_edges(1u, 11u),
        .width_mm = 200u,
        .sequence = 1u};
    return result;
}

static obstacle_scan_geometry_t
geometry_at (uint16_t centre_mm, int16_t left_x_mm, int16_t right_x_mm)
{
    obstacle_scan_geometry_t geometry = {.centre_mm = centre_mm,
                                         .left_x_mm = left_x_mm,
                                         .right_x_mm = right_x_mm};
    return geometry;
}

static bool
plan_is (obstacle_avoid_plan_t plan, bool b_gap_right, bool b_edge_unknown,
         uint16_t sidestep_mm, uint16_t pass_mm)
{
    return (plan.status == OBSTACLE_AVOID_PLAN_OK) &&
           (plan.b_gap_right == b_gap_right) &&
           (plan.b_edge_unknown == b_edge_unknown) &&
           (plan.sidestep_mm == sidestep_mm) && (plan.pass_mm == pass_mm);
}

/* OBS-U35: left_x 100 needs 100 + 75 + 60 = 235 mm; the pass is
 * 250 + 80 + 200 + 60 + 60 = 650 mm. */
static bool
plan_left_valid (void)
{
    obstacle_avoid_geometry_t car = avoid_fixture();
    obstacle_avoid_plan_t plan =
        obstacle_avoid_plan(path_result(ROBOT_OBSTACLE_DIR_LEFT_60, true, true),
                            geometry_at(250u, 100, -100), &car);
    return plan_is(plan, false, false, 235u, 650u);
}

/* OBS-U36: edges at +40 and -120 need 175 mm on the left and 255 mm on
 * the right, so each side uses its own edge. */
static bool
plan_right_valid (void)
{
    obstacle_avoid_geometry_t car = avoid_fixture();
    obstacle_scan_geometry_t geometry = geometry_at(250u, 40, -120);
    obstacle_avoid_plan_t right = obstacle_avoid_plan(
        path_result(ROBOT_OBSTACLE_DIR_RIGHT_120, true, true), geometry, &car);
    obstacle_avoid_plan_t left = obstacle_avoid_plan(
        path_result(ROBOT_OBSTACLE_DIR_LEFT_60, true, true), geometry, &car);
    return plan_is(right, true, false, 255u, 650u) &&
           plan_is(left, false, false, 175u, 650u);
}

/* OBS-U37: refuse rather than clamp, raise to the minimum, use the
 * maximum without the gap-side edge, and split long moves evenly. */
static bool
plan_limits_valid (void)
{
    obstacle_avoid_geometry_t car = avoid_fixture();
    robot_obstacle_result_t both =
        path_result(ROBOT_OBSTACLE_DIR_LEFT_60, true, true);
    robot_obstacle_result_t no_left =
        path_result(ROBOT_OBSTACLE_DIR_LEFT_60, false, true);
    robot_obstacle_result_t no_width = both;
    no_width.width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE;

    bool ok =
        obstacle_avoid_plan(both, geometry_at(250u, 266, -100), &car).status ==
        OBSTACLE_AVOID_PLAN_TOO_WIDE;
    ok = ok &&
         plan_is(obstacle_avoid_plan(both, geometry_at(250u, 265, -100), &car),
                 false, false, 400u, 650u);
    ok = ok &&
         plan_is(obstacle_avoid_plan(both, geometry_at(250u, -50, -200), &car),
                 false, false, 100u, 650u);
    ok = ok &&
         plan_is(obstacle_avoid_plan(no_left, geometry_at(250u, 0, -100), &car),
                 false, true, 400u, 650u);
    ok = ok && plan_is(obstacle_avoid_plan(no_width,
                                           geometry_at(250u, 100, -100), &car),
                       false, true, 400u, 650u);
    ok = ok &&
         plan_is(obstacle_avoid_plan(both, geometry_at(1100u, 100, -100), &car),
                 false, false, 235u, 1500u);
    ok = ok && (obstacle_avoid_plan(both, geometry_at(1101u, 100, -100), &car)
                    .status == OBSTACLE_AVOID_PLAN_TOO_FAR);
    ok = ok && (obstacle_avoid_split_count(0u) == 0u) &&
         (obstacle_avoid_split_count(1u) == 1u) &&
         (obstacle_avoid_split_count(1023u) == 1u) &&
         (obstacle_avoid_split_count(1024u) == 2u) &&
         (obstacle_avoid_split_count(2046u) == 2u) &&
         (obstacle_avoid_split_count(2047u) == 3u);
    ok = ok && (obstacle_avoid_next_part(0u) == 0u) &&
         (obstacle_avoid_next_part(1023u) == 1023u) &&
         (obstacle_avoid_next_part(1500u) == 750u) &&
         (obstacle_avoid_next_part(2047u) == 683u);
    return ok;
}

/* OBS-U38: only PATH_FOUND toward 60 or 120 degrees with a centre range. */
static bool
plan_refusals_valid (void)
{
    obstacle_avoid_geometry_t car = avoid_fixture();
    obstacle_scan_geometry_t geometry = geometry_at(250u, 100, -100);
    robot_obstacle_status_t const statuses[] = {
        ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_BLOCKED,
        ROBOT_OBSTACLE_UNKNOWN};
    bool ok = true;
    for (size_t index = 0u; index < ARRAY_COUNT(statuses); ++index)
    {
        robot_obstacle_result_t result =
            path_result(ROBOT_OBSTACLE_DIR_LEFT_60, true, true);
        result.flags = robot_obstacle_pack_flags(
            statuses[index], ROBOT_OBSTACLE_DIR_LEFT_60, true, true);
        ok = ok && (obstacle_avoid_plan(result, geometry, &car).status ==
                    OBSTACLE_AVOID_PLAN_NOT_PATH);
    }
    ok = ok &&
         (obstacle_avoid_plan(path_result(ROBOT_OBSTACLE_DIR_NONE, true, true),
                              geometry, &car)
              .status == OBSTACLE_AVOID_PLAN_NOT_PATH) &&
         (obstacle_avoid_plan(
              path_result(ROBOT_OBSTACLE_DIR_CENTRE_90, true, true), geometry,
              &car)
              .status == OBSTACLE_AVOID_PLAN_NOT_PATH) &&
         (obstacle_avoid_plan(
              path_result(ROBOT_OBSTACLE_DIR_LEFT_60, true, true),
              geometry_at(0u, 100, -100), &car)
              .status == OBSTACLE_AVOID_PLAN_NO_GEOMETRY) &&
         (obstacle_avoid_plan(
              path_result(ROBOT_OBSTACLE_DIR_LEFT_60, true, true), geometry,
              NULL)
              .status == OBSTACLE_AVOID_PLAN_NOT_PATH);
    return ok;
}

static obstacle_scan_corridor_t
empty_corridor (void)
{
    obstacle_scan_corridor_t corridor = {
        .sequence = 1u,
        .valid_mask = (uint16_t)((1u << OBSTACLE_SWEEP_POINTS) - 1u),
        .echo_us = {0u}};
    return corridor;
}

/* OBS-U41: fixture scale, pass 650 mm, so points count below 730 mm from
 * the axle; the path half width is 75 + 60 = 135 mm. Echoes: 1458 us is
 * 250 mm, 1691 us 290 mm, 2332 us 400 mm, 4665 us 800 mm. */
static bool
corridor_check_valid (void)
{
    obstacle_avoid_geometry_t car = avoid_fixture();
    obstacle_width_config_t width = width_fixture(0.0f);
    uint16_t shift_mm = 1u;
    obstacle_scan_corridor_t corridor = empty_corridor();
    bool ok = (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u,
                                             false, &shift_mm) ==
               OBSTACLE_AVOID_CORRIDOR_CLEAR) &&
              (shift_mm == 0u);

    /* Straight ahead at 250 mm: shift the full 135 mm. */
    corridor.echo_us[CENTRE_EDGE_INDEX] = 1458u;
    ok = ok &&
         (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u, false,
                                        &shift_mm) ==
          OBSTACLE_AVOID_CORRIDOR_SHIFT) &&
         (shift_mm == 135u);

    /* 100 degrees at 290 mm is 50.4 mm right: 85 mm more to the left. */
    corridor = empty_corridor();
    corridor.echo_us[8] = 1691u;
    ok = ok &&
         (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u, false,
                                        &shift_mm) ==
          OBSTACLE_AVOID_CORRIDOR_SHIFT) &&
         (shift_mm == 85u);

    /* The mirror: 80 degrees at 290 mm with the gap on the right. */
    corridor = empty_corridor();
    corridor.echo_us[4] = 1691u;
    ok = ok &&
         (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u, true,
                                        &shift_mm) ==
          OBSTACLE_AVOID_CORRIDOR_SHIFT) &&
         (shift_mm == 85u);

    /* Beyond the pass, or a wall on the gap side outside the path: clear. */
    corridor = empty_corridor();
    corridor.echo_us[CENTRE_EDGE_INDEX] = 4665u;
    corridor.echo_us[0] = 2332u;
    ok = ok && (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u,
                                              false, &shift_mm) ==
                OBSTACLE_AVOID_CORRIDOR_CLEAR);

    /* That wall leaves 200 - 135 = 65 mm, less than the 135 mm needed. */
    corridor.echo_us[CENTRE_EDGE_INDEX] = 1458u;
    ok = ok &&
         (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u, false,
                                        &shift_mm) ==
          OBSTACLE_AVOID_CORRIDOR_BLOCKED) &&
         (shift_mm == 0u);

    /* One invalid position blocks, as do missing arguments. */
    corridor = empty_corridor();
    corridor.valid_mask &= (uint16_t) ~(1u << 2u);
    ok = ok &&
         (obstacle_avoid_check_corridor(&corridor, &width, &car, 650u, false,
                                        &shift_mm) ==
          OBSTACLE_AVOID_CORRIDOR_BLOCKED) &&
         (obstacle_avoid_check_corridor(NULL, &width, &car, 650u, false,
                                        &shift_mm) ==
          OBSTACLE_AVOID_CORRIDOR_BLOCKED);
    return ok;
}

static obstacle_avoid_config_t
avoid_config (void)
{
    obstacle_avoid_config_t config = {.geometry = avoid_fixture(),
                                      .timing = {.stop_ack_ms = 500u,
                                                 .result_ms = 2000u,
                                                 .prepare_ms = 1500u,
                                                 .send_ms = 200u,
                                                 .verify_ms = 3000u,
                                                 .block_wait_ms = 5000u,
                                                 .retry_delay_ms = 1000u,
                                                 .move_speed_mm_s = 200u,
                                                 .turn_speed_deg_s = 90u,
                                                 .resume_min_mm = 10u,
                                                 .resume_min_deg = 3u,
                                                 .max_sidesteps = 3u},
                                      .width = width_fixture(0.0f)};
    return config;
}

/* Publish a new scanner result. */
static void
sim_scan (robot_obstacle_status_t status, robot_obstacle_direction_t dir,
          bool b_stop, bool b_critical)
{
    sim.in.scan.flags = robot_obstacle_set_critical(
        robot_obstacle_pack_flags(status, dir, true, true), b_critical);
    sim.in.scan.edges = robot_obstacle_pack_edges(1u, 11u);
    sim.in.scan.width_mm = 200u;
    sim.in.scan.sequence = (uint8_t)(sim.in.scan.sequence + 1u);
    sim.in.b_stop_requested = b_stop;
    sim.in.geometry = (status == ROBOT_OBSTACLE_PATH_FOUND)
                          ? geometry_at(250u, 100, -100)
                          : geometry_at(0u, 0, 0);
}

/* Publish a corridor sweep: empty, or one point 250 mm straight ahead. */
static void
sim_corridor (bool b_hit_ahead)
{
    sim.in.corridor = empty_corridor();
    sim.in.corridor.sequence = (uint8_t)(sim.ctx.corridor_sequence + 1u);
    if (b_hit_ahead)
    {
        sim.in.corridor.echo_us[CENTRE_EDGE_INDEX] = 1458u;
    }
}

/* Start held on the line, as after obstacle_start(). */
static void
sim_reset (void)
{
    sim = (avoid_sim_t){0};
    sim.config = avoid_config();
    sim.b_inhibit = true;
    sim.in.now_ms = 1000u;
    sim.in.motion_mode = ROBOT_MOTION_MODE_INHIBITED;
    sim.in.b_odometry_valid = true;
    sim_scan(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, true, false);
    obstacle_avoid_init(&sim.ctx, &sim.config, sim.in.now_ms);
}

/* One scan-task loop: step, then play the actions back as the next input. */
static obstacle_avoid_action_t
sim_step (void)
{
    if (sim.b_auto)
    {
        if ((sim.ctx.state == OBSTACLE_AVOID_PREPARE) && sim.ctx.b_rearm_known)
        {
            sim_scan(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false,
                     false);
        }
        else if (sim.ctx.state == OBSTACLE_AVOID_RUN_LEG)
        {
            /* The return leg finds the line; every other leg its target. */
            sim.in.b_move_done = true;
            sim.in.motion_done = (sim.ctx.leg == OBSTACLE_AVOID_LEG_RETURN)
                                     ? ROBOT_MOTION_DONE_LINE
                                     : ROBOT_MOTION_DONE_DIST;
        }
        else if ((sim.ctx.state == OBSTACLE_AVOID_VERIFY) &&
                 sim.ctx.b_corridor_started)
        {
            sim_corridor(false);
        }
        else
        {
            /* Nothing to answer. */
        }
    }
    sim.act = obstacle_avoid_step(&sim.ctx, &sim.in);

    sim.in.now_ms += 10u;
    sim.in.b_move_done = false;
    sim.in.b_stall = false;
    sim.in.b_enc_fault = false;
    sim.in.b_rejected = false;
    sim.in.b_inhibit_failed = false;
    if (sim.act.b_set_inhibit)
    {
        sim.b_inhibit = true;
    }
    if (sim.act.b_clear_inhibit)
    {
        sim.b_inhibit = false;
    }
    if (sim.b_inhibit && !sim.b_ignore_inhibit)
    {
        sim.in.motion_mode = ROBOT_MOTION_MODE_INHIBITED;
    }
    else if (!sim.b_hold_inhibited)
    {
        sim.in.motion_mode = ROBOT_MOTION_MODE_STOPPED;
    }
    else
    {
        /* tMotion has not reported the clear yet. */
    }
    sim.in.b_rearmed = sim.act.b_rearm;
    if (sim.act.b_rearm)
    {
        /* obstacle_scan_rearm(): SCANNING, new sequence, stop latched. */
        sim_scan(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, true, false);
        sim.in.rearm_sequence = sim.in.scan.sequence;
    }
    sim.in.corridor_request = sim.act.b_corridor
                                  ? OBSTACLE_AVOID_CORRIDOR_REQ_STARTED
                                  : OBSTACLE_AVOID_CORRIDOR_REQ_NONE;
    sim.in.send = OBSTACLE_AVOID_SEND_NONE;
    if (sim.act.b_send)
    {
        if (sim.b_send_busy)
        {
            sim.in.send = OBSTACLE_AVOID_SEND_BUSY;
        }
        else
        {
            sim.in.send = OBSTACLE_AVOID_SEND_OK;
            ++sim.sends;
            sim.last_cmd = sim.act.cmd;
        }
    }
    return sim.act;
}

/* Step with automatic answers until the state and leg match. */
static bool
sim_until (obstacle_avoid_state_t state, obstacle_avoid_leg_t leg,
           uint32_t max_steps)
{
    sim.b_auto = true;
    for (uint32_t index = 0u; index < max_steps; ++index)
    {
        (void)sim_step();
        if ((sim.ctx.state == state) && (sim.ctx.leg == leg))
        {
            sim.b_auto = false;
            return true;
        }
    }
    sim.b_auto = false;
    return false;
}

/* Step without answers for a while; true if the state never changed. */
static bool
sim_stays (obstacle_avoid_state_t state, uint32_t steps)
{
    for (uint32_t index = 0u; index < steps; ++index)
    {
        (void)sim_step();
        if (sim.ctx.state != state)
        {
            return false;
        }
    }
    return true;
}

/* From reset: clear path, follow, then a detection with PATH_FOUND left. */
static bool
sim_detect_path (void)
{
    sim_reset();
    sim_scan(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false, false);
    bool ok = sim_until(OBSTACLE_AVOID_IDLE, OBSTACLE_AVOID_LEG_FOLLOW, 10u);
    sim_scan(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, true, false);
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_STOPPING);
    (void)sim_step();
    ok = ok && (sim.ctx.state == OBSTACLE_AVOID_WAIT_RESULT);
    sim_scan(ROBOT_OBSTACLE_PATH_FOUND, ROBOT_OBSTACLE_DIR_LEFT_60, true,
             false);
    return ok;
}

/* OBS-U42: held from start-up until a clear path; stop and its ack. */
static bool
avoid_stop_valid (void)
{
    sim_reset();
    bool ok = (sim.ctx.state == OBSTACLE_AVOID_WAIT_CLEAR);
    for (uint32_t index = 0u; ok && (index < 20u); ++index)
    {
        ok = !sim_step().b_clear_inhibit;
    }
    sim_scan(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false, false);
    ok = ok && sim_step().b_clear_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_RELEASING);
    ok = ok && sim_step().b_send && (sim.act.cmd == robot_motion_cmd_follow());
    (void)sim_step();
    ok = ok && (sim.ctx.state == OBSTACLE_AVOID_IDLE);

    sim_scan(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, true, false);
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_STOPPING);
    (void)sim_step();
    ok = ok && (sim.ctx.state == OBSTACLE_AVOID_WAIT_RESULT);

    /* tMotion never reports INHIBITED: FAULT after 500 ms, not before. */
    sim_reset();
    sim_scan(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false, false);
    ok = ok && sim_until(OBSTACLE_AVOID_IDLE, OBSTACLE_AVOID_LEG_FOLLOW, 10u);
    sim.b_ignore_inhibit = true;
    sim_scan(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, true, false);
    (void)sim_step();
    ok = ok && sim_stays(OBSTACLE_AVOID_STOPPING, 49u);
    for (uint32_t index = 0u;
         ok && (sim.ctx.state != OBSTACLE_AVOID_FAULT) && (index < 5u); ++index)
    {
        (void)sim_step();
    }
    return ok && (sim.ctx.state == OBSTACLE_AVOID_FAULT) &&
           (sim.ctx.fault == OBSTACLE_AVOID_FAULT_STOP_ACK);
}

/* OBS-U43: release before sending; pending commands; kernel failures. */
static bool
avoid_send_valid (void)
{
    bool ok = sim_detect_path();
    uint32_t const sends = sim.sends;
    sim.b_hold_inhibited = true;
    ok = ok && sim_step().b_clear_inhibit && !sim.act.b_send &&
         (sim.ctx.state == OBSTACLE_AVOID_RELEASING);
    for (uint32_t index = 0u; ok && (index < 3u); ++index)
    {
        ok = !sim_step().b_send;
    }
    sim.b_hold_inhibited = false;
    sim.in.motion_mode = ROBOT_MOTION_MODE_STOPPED;
    sim.b_send_busy = true;
    ok = ok && sim_step().b_send &&
         (sim.act.cmd == robot_motion_cmd_turn(90u, false, false));
    for (uint32_t index = 0u; ok && (index < 5u); ++index)
    {
        ok = sim_step().b_send &&
             (sim.act.cmd == robot_motion_cmd_turn(90u, false, false)) &&
             (sim.ctx.state == OBSTACLE_AVOID_SEND);
    }
    sim.b_send_busy = false;
    (void)sim_step();
    (void)sim_step();
    ok = ok && (sim.ctx.state == OBSTACLE_AVOID_RUN_LEG) &&
         (sim.sends == sends + 1u);

    /* FOLLOW stays pending too, and busy past 200 ms is a FAULT. */
    sim_reset();
    sim_scan(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false, false);
    sim.b_send_busy = true;
    (void)sim_step();
    ok = ok && sim_step().b_send && (sim.act.cmd == robot_motion_cmd_follow());
    ok = ok && sim_stays(OBSTACLE_AVOID_SEND, 19u) &&
         (sim.act.cmd == robot_motion_cmd_follow());
    for (uint32_t index = 0u;
         ok && (sim.ctx.state != OBSTACLE_AVOID_FAULT) && (index < 5u); ++index)
    {
        (void)sim_step();
    }
    ok = ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_KERNEL);

    /* A send error, or a failed INHIBIT call, latches FAULT at once. */
    ok = ok && sim_detect_path();
    sim.in.send = OBSTACLE_AVOID_SEND_ERROR;
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.fault == OBSTACLE_AVOID_FAULT_KERNEL);
    ok = ok && sim_detect_path();
    sim.in.b_inhibit_failed = true;
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.fault == OBSTACLE_AVOID_FAULT_KERNEL);
    return ok;
}

/* OBS-U44: the stop latched at detection does not stop leg 0; leg 1
 * waits for a result newer than the rearm, across the 255 -> 0 wrap. */
static bool
avoid_rearm_valid (void)
{
    bool ok = sim_detect_path();
    ok = ok &&
         sim_until(OBSTACLE_AVOID_RUN_LEG, OBSTACLE_AVOID_LEG_TURN_OUT, 10u) &&
         sim.in.b_stop_requested;
    uint32_t const sends = sim.sends;
    sim.in.scan.sequence = 254u; /* The rearm publishes 255; then 0. */
    sim.in.b_move_done = true;
    sim.in.motion_done = ROBOT_MOTION_DONE_DIST;
    ok = ok && sim_step().b_rearm && (sim.ctx.state == OBSTACLE_AVOID_PREPARE);
    ok = ok && !sim_step().b_send && (sim.in.scan.sequence == 255u);
    /* CLEAR, but with the rearm's own sequence: not fresh. */
    sim.in.scan.flags = robot_obstacle_pack_flags(
        ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false, false);
    sim.in.b_stop_requested = false;
    ok = ok && sim_stays(OBSTACLE_AVOID_PREPARE, 5u) && (sim.sends == sends);
    sim_scan(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90, false, false);
    ok = ok && (sim.in.scan.sequence == 0u) && sim_step().b_send &&
         (sim.act.cmd == robot_motion_cmd_move(235u, false, false));

    /* A fresh UNKNOWN in PREPARE holds the car instead. */
    ok = ok && sim_detect_path();
    ok = ok &&
         sim_until(OBSTACLE_AVOID_PREPARE, OBSTACLE_AVOID_LEG_SIDESTEP, 20u);
    (void)sim_step();
    sim_scan(ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE, true, false);
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_BLOCKED_WAIT);
    return ok;
}

/* OBS-U45: two 135 mm shifts after the 235 mm sidestep give a 505 mm
 * offset and a 655 mm return; a third shift is refused. */
static bool
avoid_verify_valid (void)
{
    bool ok = sim_detect_path();
    sim.config.geometry.lateral_max_mm = 800u;
    for (uint32_t shift = 0u; ok && (shift < 2u); ++shift)
    {
        ok = sim_until(OBSTACLE_AVOID_VERIFY, OBSTACLE_AVOID_LEG_VERIFY, 60u);
        (void)sim_step(); /* Corridor request answered. */
        ok = ok && sim_stays(OBSTACLE_AVOID_VERIFY, 3u); /* Stale sweep. */
        sim_corridor(true);
        ok = ok && sim_step().b_send &&
             (sim.act.cmd == robot_motion_cmd_turn(90u, false, false)) &&
             (sim.ctx.sidestep_mm == 135u);
    }
    ok = ok &&
         sim_until(OBSTACLE_AVOID_VERIFY, OBSTACLE_AVOID_LEG_VERIFY, 60u) &&
         (sim.ctx.lateral_mm == 505u) && (sim.ctx.sidesteps == 3u);
    (void)sim_step();
    sim_corridor(false);
    ok =
        ok && sim_until(OBSTACLE_AVOID_PREPARE, OBSTACLE_AVOID_LEG_RETURN, 60u);
    ok = ok && sim_until(OBSTACLE_AVOID_SEND, OBSTACLE_AVOID_LEG_RETURN, 10u) &&
         (sim.act.cmd == robot_motion_cmd_move(655u, false, true));

    /* Third shift: sidesteps used up, held, FAULT after 5 s. */
    ok = ok && sim_detect_path();
    sim.config.geometry.lateral_max_mm = 800u;
    sim.config.timing.max_sidesteps = 1u;
    ok = ok && sim_until(OBSTACLE_AVOID_VERIFY, OBSTACLE_AVOID_LEG_VERIFY, 60u);
    (void)sim_step();
    sim_corridor(true);
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_BLOCKED_WAIT);
    sim.in.b_stop_requested = true;
    ok = ok && sim_stays(OBSTACLE_AVOID_BLOCKED_WAIT, 499u);
    (void)sim_step();
    (void)sim_step();
    return ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_BLOCKED);
}

/* OBS-U46: the done reason ends the return leg early or retries once. */
static bool
avoid_line_valid (void)
{
    bool ok = sim_detect_path();
    sim.config.geometry.lateral_max_mm = 1000u;
    sim.config.geometry.sidestep_max_mm = 1000u;
    sim.in.geometry = geometry_at(250u, 765, -100); /* 900 mm sidestep. */
    ok = ok && sim_until(OBSTACLE_AVOID_SEND, OBSTACLE_AVOID_LEG_RETURN, 80u) &&
         (sim.act.cmd == robot_motion_cmd_move(525u, false, true));
    (void)sim_step();
    sim.in.b_move_done = true;
    sim.in.motion_done = ROBOT_MOTION_DONE_LINE;
    ok = ok && sim_step().b_send &&
         (sim.act.cmd == robot_motion_cmd_move(40u, false, false)) &&
         (sim.act.p_reason != NULL) &&
         (sim.ctx.leg == OBSTACLE_AVOID_LEG_OVERSHOOT);

    /* Missed line: one search move of 150 mm, then FAULT. */
    ok = ok && sim_detect_path();
    ok =
        ok && sim_until(OBSTACLE_AVOID_RUN_LEG, OBSTACLE_AVOID_LEG_RETURN, 80u);
    sim.in.b_move_done = true;
    sim.in.motion_done = ROBOT_MOTION_DONE_DIST;
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_RETRY_WAIT);
    ok = ok && sim_stays(OBSTACLE_AVOID_RETRY_WAIT, 99u);
    ok = ok && sim_until(OBSTACLE_AVOID_SEND, OBSTACLE_AVOID_LEG_RETURN, 20u) &&
         (sim.act.cmd == robot_motion_cmd_move(150u, false, true));
    (void)sim_step();
    sim.in.b_move_done = true;
    sim.in.motion_done = ROBOT_MOTION_DONE_DIST;
    (void)sim_step();
    return ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_RETRY_USED);
}

/* Run to the sidestep with its move just queued at odometry 0. */
static bool
sim_in_sidestep (void)
{
    bool ok = sim_detect_path();
    sim.in.distance_mm = 0;
    return ok &&
           sim_until(OBSTACLE_AVOID_RUN_LEG, OBSTACLE_AVOID_LEG_SIDESTEP, 30u);
}

/* OBS-U47: one retry from odometry, then FAULT; FAULT never releases. */
static bool
avoid_fault_valid (void)
{
    bool ok = sim_in_sidestep();
    sim.in.distance_mm = 100;
    sim.in.b_stall = true;
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_RETRY_WAIT);
    ok = ok && sim_stays(OBSTACLE_AVOID_RETRY_WAIT, 99u);
    ok = ok &&
         sim_until(OBSTACLE_AVOID_SEND, OBSTACLE_AVOID_LEG_SIDESTEP, 20u) &&
         (sim.act.cmd == robot_motion_cmd_move(135u, false, false));
    (void)sim_step();
    sim.in.b_stall = true;
    (void)sim_step();
    ok = ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_RETRY_USED);

    /* FAULT ignores every later input and keeps asking for INHIBIT. */
    for (uint32_t index = 0u; ok && (index < 1000u); ++index)
    {
        if ((index % 100u) == 0u)
        {
            sim_scan((index % 200u) == 0u ? ROBOT_OBSTACLE_CLEAR
                                          : ROBOT_OBSTACLE_PATH_FOUND,
                     ROBOT_OBSTACLE_DIR_LEFT_60, false, false);
        }
        sim.in.motion_mode = ROBOT_MOTION_MODE_STOPPED;
        obstacle_avoid_action_t act = sim_step();
        ok = act.b_set_inhibit && !act.b_clear_inhibit && !act.b_send &&
             !act.b_rearm && (sim.ctx.state == OBSTACLE_AVOID_FAULT);
    }

    /* A remainder under 10 mm counts as done: the next leg is the turn. */
    ok = ok && sim_in_sidestep();
    sim.in.distance_mm = 230;
    sim.in.b_stall = true;
    (void)sim_step();
    ok = ok &&
         sim_until(OBSTACLE_AVOID_SEND, OBSTACLE_AVOID_LEG_TURN_BACK, 120u) &&
         (sim.act.cmd == robot_motion_cmd_turn(90u, true, false)) &&
         (sim.ctx.lateral_mm == 235u);

    ok = ok && sim_in_sidestep();
    sim.in.b_enc_fault = true;
    (void)sim_step();
    ok = ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_ENCODER);
    ok = ok && sim_in_sidestep();
    sim.in.b_rejected = true;
    (void)sim_step();
    ok = ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_REJECTED);
    ok = ok && sim_in_sidestep();
    sim.in.b_odometry_valid = false;
    sim.in.b_stall = true;
    (void)sim_step();
    return ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_ODOMETRY);
}

/* OBS-U48: obstacles during the bypass, and one complete ideal box. */
static bool
avoid_obstacles_valid (void)
{
    bool ok = sim_in_sidestep();
    sim.in.distance_mm = 50;
    sim.in.b_stop_requested = true;
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_BLOCKED_WAIT);
    ok = ok && sim_stays(OBSTACLE_AVOID_BLOCKED_WAIT, 10u);
    sim.in.b_stop_requested = false;
    ok = ok && sim_step().b_clear_inhibit;
    ok = ok &&
         sim_until(OBSTACLE_AVOID_SEND, OBSTACLE_AVOID_LEG_SIDESTEP, 20u) &&
         (sim.act.cmd == robot_motion_cmd_move(185u, false, false));

    ok = ok && sim_in_sidestep();
    sim.in.b_stop_requested = true;
    (void)sim_step();
    ok = ok && sim_stays(OBSTACLE_AVOID_BLOCKED_WAIT, 499u);
    (void)sim_step();
    (void)sim_step();
    ok = ok && (sim.ctx.fault == OBSTACLE_AVOID_FAULT_BLOCKED);

    /* During a turn a stop request is ignored, but CRITICAL is not. */
    ok = ok && sim_detect_path();
    ok = ok &&
         sim_until(OBSTACLE_AVOID_RUN_LEG, OBSTACLE_AVOID_LEG_TURN_OUT, 10u) &&
         sim.in.b_stop_requested;
    ok = ok && sim_stays(OBSTACLE_AVOID_RUN_LEG, 5u);
    sim.in.scan.flags = robot_obstacle_set_critical(sim.in.scan.flags, true);
    ok = ok && sim_step().b_set_inhibit &&
         (sim.ctx.state == OBSTACLE_AVOID_BLOCKED_WAIT);

    /* A whole box ends with FOLLOW queued and the machine IDLE. */
    ok = ok && sim_detect_path();
    ok = ok &&
         sim_until(OBSTACLE_AVOID_IDLE, OBSTACLE_AVOID_LEG_FOLLOW, 200u) &&
         (sim.last_cmd == robot_motion_cmd_follow()) &&
         (sim.ctx.fault == OBSTACLE_AVOID_FAULT_NONE);
    return ok;
}

#endif /* APP_TEST */
