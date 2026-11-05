/**
 * @file motion_test.c
 * @brief On-target tests for the motion subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/motion/logs/ and record the result in
 * testing/motion/TEST_LOG.md. Test IDs: MOT-U01, MOT-U02, ...
 */

#include "motion.h"
#include "../test_report.h"
#include "motion_config.h"
#include "motion_drive.h"
#include "motion_move.h"
#include "motion_pid.h"
#include "drv_motor.h"
#include "config.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define PWM_SLICE_STRIDE 0x14u
#define PWM_SLICE_COUNT 8u
#define MOTOR_WRAP_20KHZ (((UW)SYSCLK * 50u) - 1u) /* clk_sys / 20 kHz - 1 */

/* PIO0 CTRL; the encoders run on state machines 2 and 3 (drv_encoder.c). */
#define PIO0_CTRL 0x50200000u
#define ENCODER_SM_ENABLE ((1u << 2) | (1u << 3))

/* SMx_ADDR (program counter) of state machines 2 and 3. */
#define PIO0_SM2_ADDR 0x50200104u
#define PIO0_SM3_ADDR 0x5020011cu

#define REST_CHECK_MS 200

/* MOT-U18: PID outputs must match hand-worked values this closely. */
#define PID_TOLERANCE 0.0001f

/*
 * MOT-U22: time this many test-loop updates; speeds assume each takes
 * MOTION_UPDATE_PERIOD_MS, so the mean must be within this many percent.
 */
#define PERIOD_CHECK_UPDATES 50u
#define PERIOD_TOLERANCE_PERCENT 5u

/*
 * tk_dly_tsk() waits one kernel tick longer than asked, so the test loops
 * ask for one tick less. Each update ends well within a tick, so every
 * wait then ends on the tick MOTION_UPDATE_PERIOD_MS after the last.
 */
#if (MOTION_UPDATE_PERIOD_MS <= CNF_TIMER_PERIOD) ||                           \
    ((MOTION_UPDATE_PERIOD_MS % CNF_TIMER_PERIOD) != 0)
#error "MOTION_UPDATE_PERIOD_MS must be a multiple of the tick, above it"
#endif
#define UPDATE_DELAY_MS (MOTION_UPDATE_PERIOD_MS - CNF_TIMER_PERIOD)

/*
 * MOT-U34: run tMotion this long, then check its wake-up periods are
 * within PERIOD_JITTER_US of MOTION_UPDATE_PERIOD_MS (NFR1, A1).
 */
#define TASK_RUN_MS 1000
#define PERIOD_JITTER_US 1000u
#define US_PER_MS 1000u

/* MOT-U35, MOT-U36: tMotion acts on a command within this many ms. */
#define TASK_REACT_MS (3 * (INT)MOTION_UPDATE_PERIOD_MS)

/* MOT-U38: read the shared odometry across this many updates. */
#define SHARED_CHECK_MS 200
#define SHARED_UPDATE_SLACK 2u

/* NFR6: INHIBIT brakes within this long. */
#define INHIBIT_LIMIT_US 20000u

#if MOTION_TASK_TESTS
/* MOT-U36, MOT-U37: tMotion commands on the lifted car. */
#define TASK_START_DELAY_MS 10000
#define TASK_SHORT_MM 300u
#define TASK_LONG_MM 1000u
/* 300 mm at 200 mm/s takes 1.5 s; allow for start and braking. */
#define TASK_SHORT_DONE_MS 4000
#define TASK_INTERRUPT_MS 500
#define TASK_QUIET_MS 2500
#define TASK_FOLLOW_MS 1500
#define TASK_FOLLOW_END_MS 300
#define TASK_STALL_WAIT_MS 5000
#endif /* MOTION_TASK_TESTS */

#if MOTION_MANUAL_TESTS
#define TURN_WINDOW_MS 20000
#define TURN_PRINT_MS 500

/* MOT-U06: each wheel alone, forward then backward, at a gentle duty. */
#define SPIN_DUTY 0.4f
#define SPIN_START_DELAY_MS 3000
#define SPIN_MS 1500
#define SPIN_PAUSE_MS 1000

typedef struct
{
    motor_id_t id;
    float duty;
    char const * p_name;
} spin_step_t;
#endif /* MOTION_MANUAL_TESTS */

#if MOTION_SPEED_TESTS
/* MOT-U19: duty sweep on the lifted car, in tenths of a percent. */
#define SPEED_START_DELAY_MS 5000
#define SWEEP_START_PERMILLE 100
#define SWEEP_STEP_PERMILLE 50
#define SWEEP_SETTLE_MS 1000
#define SWEEP_MEASURE_MS 500

/* MOT-U20: lifted speed steps; MOT-U21: floor runs, unplugged. */
#define STEP_HIGH_MM_S 200
#define STEP_LOW_MM_S 100
#define STEP_MS 1500
#define FLOOR_RUN_MM 800u
#define FLOOR_START_DELAY_MS 15000
#define FLOOR_PAUSE_MS 10000
#define PLUG_WAIT_MS 30000
/*
 * MOT-U21 prints its records once a terminal has had the USB console open
 * this long, so a link dropped by the motors does not lose them; it gives
 * up after TERMINAL_WAIT_MAX_MS. 3 is tm_usb_state()'s "host asserted DTR".
 */
#define TERMINAL_POLL_MS 100
#define TERMINAL_STEADY_MS 2000
#define TERMINAL_WAIT_MAX_MS 300000
#define USB_STATE_TERMINAL_OPEN 3u
/* 8 s: an 800 mm run with room for a slow open-loop wheel. */
#define RECORD_MAX_SAMPLES 400

/*
 * Long traces wait every PRINT_BURST_LINES lines until the USB console task
 * has sent them: it runs below this task, and its 4 KB buffer drops what
 * does not fit (about 90 trace lines). The wait is bounded, so a host that
 * has stopped reading cannot hang the test run.
 */
#define PRINT_BURST_LINES 20
#define PRINT_PAUSE_MS 20
#define PRINT_DRAIN_MAX_MS 500

/* One speed-loop update: what was asked, and what the wheels did. */
typedef struct
{
    motion_command_t command;
    int16_t left_mm_s;
    int16_t right_mm_s;
} speed_sample_t;

/* A run of samples, one per MOTION_UPDATE_PERIOD_MS. */
typedef struct
{
    speed_sample_t samples[RECORD_MAX_SAMPLES];
    INT count;
} speed_record_t;
#endif /* MOTION_SPEED_TESTS */

#if MOTION_SYNC_TESTS
/* MOT-U24: lifted sign check; MOT-U25: floor drift; MOT-U26: nudge. */
#define SYNC_START_DELAY_MS 10000
#define SYNC_FORWARD_MM 1000u
#define SYNC_BACKWARD_MM 500u
#define SYNC_RUNS 3
#define SYNC_MEASURE_MS 20000
/* Above the module's own timeout of a 1 m move, which ends a run first. */
#define SYNC_RUN_TIMEOUT_MS 30000
/* 8 s: a 1 m move at 200 mm/s with room to spare. */
#define SYNC_MAX_SAMPLES 400

/* One update of a move: the straight-line loop and the setpoints. */
typedef struct
{
    motion_sync_t sync;
    motion_command_t command;
} sync_sample_t;

/* A move's samples, one per MOTION_UPDATE_PERIOD_MS. */
typedef struct
{
    sync_sample_t samples[SYNC_MAX_SAMPLES];
    INT count;
} sync_record_t;
#endif /* MOTION_SYNC_TESTS */

#if MOTION_FOLLOW_TESTS
/* MOT-U28: lifted follow steering; MOT-U29: lifted stop-at-line. */
#define FOLLOW_START_DELAY_MS 10000
#define FOLLOW_STEP_MS 1500
/* With MOTION_FOLLOW_KP 0.1 this asks for 30 deg/s. */
#define FOLLOW_TEST_ERROR 300
/* Longest wait for following to stop, or a move to end, in the checks. */
#define FOLLOW_END_WAIT_MS 3000
#define LINE_GONE_MS 500
#define LINE_ON_MS 500
#define LINE_MOVE_MM 200u
#define LINE_TURN_DEG 360u
#endif /* MOTION_FOLLOW_TESTS */

#if MOTION_MOVE_TESTS || MOTION_TURN_TESTS || MOTION_SPEED_TESTS ||            \
    MOTION_SYNC_TESTS || MOTION_FOLLOW_TESTS
#define MOVE_SETTLE_MS 500
#endif

#if MOTION_MOVE_TESTS || MOTION_TURN_TESTS
/* MOT-U08..U16: floor moves and turns, then a held wheel, lifted. */
#define MOVE_START_DELAY_MS 5000
/* Above every module timeout, so the module's own timeout ends a run. */
#define MOVE_TIMEOUT_MS 30000
#define FORWARD_TEST_MM 1000u
#define STALL_START_DELAY_MS 10000
#endif /* MOTION_MOVE_TESTS || MOTION_TURN_TESTS */

#if MOTION_MOVE_TESTS
#define BACKWARD_TEST_MM 100u
#endif /* MOTION_MOVE_TESTS */

#if MOTION_TURN_TESTS
/* MOT-U11..U16: calibration and accuracy turns, preemption, stall. */
#define CAL_TURN_DEG 360u
#define CAL_TURN_COUNT 3
#define TEST_TURN_DEG 90u
#define U_TURN_DEG 180u
#define TURN_TRIALS 5
#define TURN_MEASURE_MS 8000
#define PREEMPT_AFTER_MS 1000
#define PREEMPT_STOP_AFTER_MS 500
#define PREEMPT_BACK_MM 100u

typedef enum
{
    TURN_LEFT = 0,
    TURN_RIGHT,
    TURN_U
} turn_kind_t;
#endif /* MOTION_TURN_TESTS */

/* Private function declarations. */
static BOOL slice_braked (UW pin);
static BOOL encoders_steady (void);
static BOOL move_idle_at_rest (void);
static BOOL pid_math_ok (void);
static BOOL near (float actual, float expected);
static BOOL update_period_ok (void);
static ER wait_update_period (void);
static BOOL sync_idle_at_rest (void);
static BOOL follow_idle_at_rest (void);
static BOOL command_packing_ok (void);
static BOOL task_timing_ok (void);
static BOOL task_refusals_ok (void);
static BOOL shared_odometry_ok (void);
static BOOL send_command (uint32_t cmd);
static UINT wait_events (UINT bits, INT timeout_ms);
static BOOL wait_mode (robot_motion_mode_t mode, INT timeout_ms);
static BOOL inhibit_latency_ok (char const * p_id, uint32_t set_us);
#if MOTION_TASK_TESTS
static BOOL task_commands_lifted (void);
static BOOL task_stall_lifted (void);
#endif
#if MOTION_MANUAL_TESTS
static void print_counts_while_turning (void);
static void spin_each_wheel (void);
static UW pin_level (UW levels, UW pin);
#endif
#if MOTION_MOVE_TESTS
static void run_move (char const * p_id, uint16_t distance_mm, bool b_forward);
#endif
#if MOTION_TURN_TESTS
static void run_turns (char const * p_id, turn_kind_t kind, uint16_t angle_deg,
                       INT count, INT pause_ms);
static motion_status_t start_turn (turn_kind_t kind, uint16_t angle_deg);
static BOOL preempt_and_stop (void);
static void run_until_timeout (void);
#endif
#if MOTION_SPEED_TESTS
static void sweep_feed_forward (void);
static void step_speed_lifted (void);
static void run_floor_unplugged (void);
static void wait_for_terminal (void);
#if TM_CONSOLE_USB_CDC
/* USB console link state (lib/libtm usb_console.c; no public header). */
extern UW tm_usb_state (void);
#endif
static void record_for (speed_record_t * p_record, INT duration_ms);
static void record_while_moving (speed_record_t * p_record);
static void print_record (char const * p_id, char const * p_label,
                          speed_record_t const * p_record);
#endif
#if MOTION_SYNC_TESTS
static void check_sync_sign_lifted (void);
static void measure_drift_floor (void);
static void nudge_and_reverse (void);
static void run_sync_move (char const * p_id, char const * p_label,
                           uint16_t distance_mm, bool b_forward,
                           sync_record_t * p_record);
static void print_sync_trace (char const * p_id,
                              sync_record_t const * p_record);
#endif
#if MOTION_SPEED_TESTS || MOTION_SYNC_TESTS
static void wait_console_drained (void);
#if TM_CONSOLE_USB_CDC
/* Bytes queued for the host (lib/libtm usb_console.c; no public header). */
extern UW tm_usb_pending (void);
#endif
#endif
#if MOTION_FOLLOW_TESTS
static BOOL check_follow_lifted (void);
static BOOL check_stop_at_line_lifted (void);
static void follow_arcs_floor (void);
static void feed_line_for (int16_t error, bool b_seen, INT duration_ms);
static INT feed_line_until_idle (bool b_seen);
#endif
#if MOTION_MOVE_TESTS || MOTION_TURN_TESTS
static BOOL hold_wheel_until_stall (char const * p_id, bool b_turn);
static INT update_while_moving (void);
#endif
#if MOTION_MOVE_TESTS || MOTION_TURN_TESTS || MOTION_SPEED_TESTS ||            \
    MOTION_SYNC_TESTS || MOTION_FOLLOW_TESTS
static void update_for (INT duration_ms);
#endif

/* Public function definitions. */

void
motion_test (void)
{
    TEST_INIT();

    /*
     * MOT-U01..U32 call the motion module directly, so tMotion hands it
     * over first and takes it back for MOT-U34 onwards.
     */
    if (motion_task_hold(true) != E_OK)
    {
        tm_printf((UB *)"[TEST] motion: tMotion did not stop; results of "
                        "MOT-U01..U32 are unreliable\n");
    }

    /* MOT-U01: both motors' slices run at 20 kHz with both inputs low. */
    TEST_CHECK("MOT-U01",
               slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                   slice_braked(MOTION_RIGHT_MOTOR_PIN_A),
               "both motors run 20 kHz PWM and are braked");

    /* MOT-U03: both encoder state machines run; counts hold at rest. */
    TEST_CHECK("MOT-U03",
               ((in_w(PIO0_CTRL) & ENCODER_SM_ENABLE) == ENCODER_SM_ENABLE) &&
                   encoders_steady(),
               "both encoders count on PIO0 and hold still at rest");

    /* MOT-U07: idle at rest; 0 mm, 0 degrees and too large are refused. */
    TEST_CHECK("MOT-U07", move_idle_at_rest(),
               "moves start stopped, odometry holds at rest, 0 refused");

    /* MOT-U18: PID arithmetic against hand-worked values; no motors. */
    TEST_CHECK("MOT-U18", pid_math_ok(),
               "PID P, I, D terms, limits, anti-windup and reset");

    /* MOT-U22: the test loops update every period; no motors. */
    TEST_CHECK("MOT-U22", update_period_ok(),
               "test-loop updates run at MOTION_UPDATE_PERIOD_MS");

    /* MOT-U23: straight-line loop idle at rest, switchable; no motors. */
    TEST_CHECK("MOT-U23", sync_idle_at_rest(),
               "straight-line loop idle at rest, switches on and off");

    /* MOT-U27: follow and line input refusals, debouncing; no motors. */
    TEST_CHECK("MOT-U27", follow_idle_at_rest(),
               "follow and line input refuse bad values, line debounced");

#if MOTION_MANUAL_TESTS
    /* MOT-U06: manual; see testing/motion/TEST_LOG.md. */
    spin_each_wheel();

    /* MOT-U04, MOT-U05: manual; see testing/motion/TEST_LOG.md. */
    print_counts_while_turning();
#endif

#if MOTION_MOVE_TESTS
    /* MOT-U08..U10: manual; see testing/motion/TEST_LOG.md. */
    run_move("MOT-U08", FORWARD_TEST_MM, true);
    run_move("MOT-U09", BACKWARD_TEST_MM, false);
    TEST_CHECK("MOT-U10", hold_wheel_until_stall("MOT-U10", false),
               "held wheel stalls, brakes, refuses moves until stopped");
#endif

#if MOTION_TURN_TESTS
    /* MOT-U11..U16: manual; see testing/motion/TEST_LOG.md. */
    run_turns("MOT-U11", TURN_LEFT, CAL_TURN_DEG, CAL_TURN_COUNT, 0);
    run_turns("MOT-U12", TURN_RIGHT, CAL_TURN_DEG, CAL_TURN_COUNT, 0);
    run_turns("MOT-U13", TURN_LEFT, TEST_TURN_DEG, TURN_TRIALS,
              TURN_MEASURE_MS);
    run_turns("MOT-U13", TURN_RIGHT, TEST_TURN_DEG, TURN_TRIALS,
              TURN_MEASURE_MS);
    run_turns("MOT-U14", TURN_U, U_TURN_DEG, TURN_TRIALS, TURN_MEASURE_MS);
    TEST_CHECK("MOT-U15", preempt_and_stop(),
               "a turn replaces a move, and stop ends a turn at once");
    TEST_CHECK("MOT-U16", hold_wheel_until_stall("MOT-U16", true),
               "held wheel stalls a turn, brakes, refuses until stopped");
    run_until_timeout();
#endif

#if MOTION_SPEED_TESTS
    /* MOT-U19..U21: manual; see testing/motion/TEST_LOG.md. */
    sweep_feed_forward();
    step_speed_lifted();
    run_floor_unplugged();
#endif

#if MOTION_SYNC_TESTS
    /* MOT-U24..U26: manual; see testing/motion/TEST_LOG.md. */
    check_sync_sign_lifted();
    measure_drift_floor();
    nudge_and_reverse();
#endif

#if MOTION_FOLLOW_TESTS
    /* MOT-U28..U30: manual; see testing/motion/TEST_LOG.md. */
    TEST_CHECK("MOT-U28", check_follow_lifted(),
               "follow steers towards the error, stops on stale input");
    TEST_CHECK("MOT-U29", check_stop_at_line_lifted(),
               "moves and turns end when the line comes into view");
    follow_arcs_floor();
#endif

    /* MOT-U33: command word packing, limits and decoding; no motors. */
    TEST_CHECK("MOT-U33", command_packing_ok(),
               "motion commands pack, saturate and decode");

    /* MOT-U34: tMotion wakes every period while the system runs. */
    TEST_CHECK("MOT-U34", task_timing_ok(),
               "tMotion updates every period within 1 ms (NFR1)");

    /* MOT-U35: invalid commands refused, INHIBIT discards; no motors. */
    TEST_CHECK("MOT-U35", task_refusals_ok(),
               "tMotion refuses bad commands, INHIBIT brakes and flushes");

    /* MOT-U38: shared odometry updates every period; no motors. */
    TEST_CHECK("MOT-U38", shared_odometry_ok(),
               "shared odometry rises every update and holds at rest");

#if MOTION_TASK_TESTS
    /* MOT-U36, MOT-U37: manual; see testing/motion/TEST_LOG.md. */
    TEST_CHECK("MOT-U36", task_commands_lifted(),
               "MOVE_DONE, preemption, STOP, INHIBIT and FOLLOW via tMotion");
    TEST_CHECK("MOT-U37", task_stall_lifted(),
               "held wheel raises STALL via tMotion until STOP");
#endif

    /* Leave tMotion running the car for the rest of the system. */
    (void)motion_task_hold(false);

    TEST_SUMMARY("motion");
}

/* Private function definitions. */

/* The slice of @p pin is enabled, wraps for 20 kHz, and both levels are 0. */
static BOOL
slice_braked (UW pin)
{
    UW base = PWM_BASE + (PWM_SLICE_STRIDE * ((pin / 2u) % PWM_SLICE_COUNT));

    return ((in_w(base + PWM_CHx_CSR) & PWM_CH_CSR_EN) != 0u) &&
           (in_w(base + PWM_CHx_TOP) == MOTOR_WRAP_20KHZ) &&
           (in_w(base + PWM_CHx_CC) == 0u);
}

/* Count reads succeed and neither count changes over REST_CHECK_MS. */
static BOOL
encoders_steady (void)
{
    motion_counts_t before;
    motion_counts_t after;

    if ((motion_drive_get_counts(&before) != MOTION_OK) ||
        (tk_dly_tsk(REST_CHECK_MS) != E_OK) ||
        (motion_drive_get_counts(&after) != MOTION_OK))
    {
        return FALSE;
    }
    return (before.left_count == after.left_count) &&
           (before.right_count == after.right_count);
}

/* STOPPED and not busy; distance and speed hold over a rest period. */
static BOOL
move_idle_at_rest (void)
{
    motion_odometry_t before;
    motion_odometry_t after;

    if ((motion_move_stop() != MOTION_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_get_odometry(&before) != MOTION_OK) ||
        (tk_dly_tsk(REST_CHECK_MS) != E_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_get_odometry(&after) != MOTION_OK))
    {
        return FALSE;
    }
    return (MOTION_STATE_STOPPED == motion_move_get_state()) &&
           !motion_move_is_busy() &&
           (before.distance_mm == after.distance_mm) &&
           (before.heading_deg == after.heading_deg) &&
           (0 == after.speed_mm_s) &&
           (motion_move_forward(0u) == MOTION_ERR_PARAM) &&
           (motion_move_turn_left(0u) == MOTION_ERR_PARAM) &&
           (motion_move_turn_right(0u) == MOTION_ERR_PARAM) &&
           (motion_move_turn_left(MOTION_MAX_TURN_DEG + 1u) ==
            MOTION_ERR_PARAM) &&
           !motion_move_is_busy() && slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
           slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
}

/* Each PID term, the limits, anti-windup and reset give worked values. */
static BOOL
pid_math_ok (void)
{
    motion_pid_t pid;
    BOOL b_ok = TRUE;

    /* P: 0.5 * 4 = 2. */
    motion_pid_init(&pid, 0.5f, 0.0f, 0.0f, -10.0f, 10.0f);
    b_ok = b_ok && near(motion_pid_update(&pid, 4.0f, 0.02f), 2.0f);

    /* I: 1 * 2 * 0.5 per update, accumulating. */
    motion_pid_init(&pid, 0.0f, 1.0f, 0.0f, -10.0f, 10.0f);
    b_ok = b_ok && near(motion_pid_update(&pid, 2.0f, 0.5f), 1.0f);
    b_ok = b_ok && near(motion_pid_update(&pid, 2.0f, 0.5f), 2.0f);

    /* D: none on the first update, then 1 * (1 - 0) / 0.5. */
    motion_pid_init(&pid, 0.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    b_ok = b_ok && near(motion_pid_update(&pid, 0.0f, 0.5f), 0.0f);
    b_ok = b_ok && near(motion_pid_update(&pid, 1.0f, 0.5f), 2.0f);

    /*
     * Limits and anti-windup: a large error holds the output at the limit
     * without winding the integral up, so a small error of the other sign
     * swings the output at once: -0.5 P + -0.5 I. A wound-up integral (1)
     * would give 0 instead.
     */
    motion_pid_init(&pid, 1.0f, 10.0f, 0.0f, -1.0f, 1.0f);
    for (INT i = 0; i < 20; i++)
    {
        b_ok = b_ok && near(motion_pid_update(&pid, 5.0f, 0.1f), 1.0f);
    }
    b_ok = b_ok && near(motion_pid_update(&pid, -0.5f, 0.1f), -1.0f);

    /* Narrower limits pull the integral in; reset clears it. */
    motion_pid_set_limits(&pid, -0.25f, 0.25f);
    b_ok = b_ok && near(motion_pid_update(&pid, 0.0f, 0.1f), -0.25f);
    motion_pid_reset(&pid);
    b_ok = b_ok && near(motion_pid_update(&pid, 0.0f, 0.1f), 0.0f);

    /* Bad arguments give 0. */
    b_ok = b_ok && near(motion_pid_update(NULL, 1.0f, 0.1f), 0.0f);
    b_ok = b_ok && near(motion_pid_update(&pid, 1.0f, 0.0f), 0.0f);
    return b_ok;
}

/* @p actual is within PID_TOLERANCE of @p expected. */
static BOOL
near (float actual, float expected)
{
    float difference = actual - expected;

    return (difference < PID_TOLERANCE) && (difference > -PID_TOLERANCE);
}

/*
 * Time PERIOD_CHECK_UPDATES updates of the loop the speed tests use (a
 * delay, then motion_move_update()) and print the total. The system
 * timer's 10 ms resolution is 1 % of the roughly 1 s this takes.
 */
static BOOL
update_period_ok (void)
{
    SYSTIM start;
    SYSTIM end;
    UW expected_ms = PERIOD_CHECK_UPDATES * MOTION_UPDATE_PERIOD_MS;
    UW margin_ms = (expected_ms * PERIOD_TOLERANCE_PERCENT) / 100u;

    if (tk_get_otm(&start) != E_OK)
    {
        return FALSE;
    }
    for (UW i = 0u; i < PERIOD_CHECK_UPDATES; i++)
    {
        if ((wait_update_period() != E_OK) ||
            (motion_move_update() != MOTION_OK))
        {
            return FALSE;
        }
    }
    if (tk_get_otm(&end) != E_OK)
    {
        return FALSE;
    }

    /* Unsigned subtraction stays right across a wrap of the low word. */
    UW elapsed_ms = end.lo - start.lo;
    tm_printf((UB *)"[TEST] MOT-U22 %d updates took %d ms (%d expected)\n",
              (INT)PERIOD_CHECK_UPDATES, (INT)elapsed_ms, (INT)expected_ms);
    return (elapsed_ms + margin_ms >= expected_ms) &&
           (elapsed_ms <= expected_ms + margin_ms);
}

/* Wait until the next test-loop update is due (see UPDATE_DELAY_MS). */
static ER
wait_update_period (void)
{
    return tk_dly_tsk((TMO)UPDATE_DELAY_MS);
}

/*
 * Stopped, the straight-line loop has no error or correction; switching
 * it off and on works and leaves the motors braked; a NULL read fails.
 */
static BOOL
sync_idle_at_rest (void)
{
    motion_sync_t sync;

    if ((motion_move_stop() != MOTION_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_get_sync(&sync) != MOTION_OK))
    {
        return FALSE;
    }
    BOOL b_idle = (0 == sync.error_ticks) && (0 == sync.correction_deg_s);
    BOOL b_switched = (motion_move_set_sync(false) == MOTION_OK) &&
                      (motion_move_set_sync(true) == MOTION_OK);
    return b_idle && b_switched &&
           (motion_move_get_sync(NULL) == MOTION_ERR_PARAM) &&
           (MOTION_STATE_STOPPED == motion_move_get_state()) &&
           slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
           slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
}

/*
 * Stopped: follow and the line input refuse out-of-range values,
 * stop-at-line needs a move, the line counts as seen (and as gone) only
 * after MOTION_LINE_SEEN_SAMPLES readings agree, follow refuses a stale
 * line reading, and nothing drives the motors.
 */
static BOOL
follow_idle_at_rest (void)
{
    motion_follow_t follow;
    BOOL b_debounced = TRUE;

    if ((motion_move_stop() != MOTION_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_get_follow(&follow) != MOTION_OK))
    {
        return FALSE;
    }
    BOOL b_idle = (0 == follow.turn_deg_s) &&
                  (MOTION_DONE_NONE == motion_move_get_done_reason());
    BOOL b_refused = (motion_move_follow(0u) == MOTION_ERR_PARAM) &&
                     (motion_move_follow((uint16_t)MOTION_MAX_SPEED_MM_S +
                                         1u) == MOTION_ERR_PARAM) &&
                     (motion_move_set_line(MOTION_LINE_ERROR_MAX + 1, false) ==
                      MOTION_ERR_PARAM) &&
                     (motion_move_set_line(-MOTION_LINE_ERROR_MAX - 1, false) ==
                      MOTION_ERR_PARAM) &&
                     (motion_move_stop_at_line() == MOTION_ERR_PARAM) &&
                     (motion_move_get_follow(NULL) == MOTION_ERR_PARAM);

    /* Seen only on the last of MOTION_LINE_SEEN_SAMPLES readings; then gone. */
    for (UW pass = 0u; pass < 2u; pass++)
    {
        bool b_seen = (0u == pass);
        for (UW i = 1u; i <= MOTION_LINE_SEEN_SAMPLES; i++)
        {
            (void)motion_move_set_line(0, b_seen);
            (void)motion_move_get_follow(&follow);
            bool b_expected =
                (i == MOTION_LINE_SEEN_SAMPLES) ? b_seen : !b_seen;
            b_debounced = b_debounced && (follow.b_line_seen == b_expected);
        }
    }

    /* Let the last reading go stale: following must refuse to start. */
    for (UW i = 0u; i <= (MOTION_LINE_TIMEOUT_MS / MOTION_UPDATE_PERIOD_MS);
         i++)
    {
        (void)motion_move_update();
    }
    b_refused = b_refused && (motion_move_follow(MOTION_FOLLOW_SPEED_MM_S) ==
                              MOTION_ERR_PARAM);

    return b_idle && b_refused && b_debounced &&
           ((UW)MOTION_STATE_FOLLOW <= 7u) &&
           (MOTION_STATE_STOPPED == motion_move_get_state()) &&
           !motion_move_is_busy() && slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
           slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
}

/*
 * Each command packs into its fields, saturates at the field limits
 * without touching its neighbours, and leaves the reserved bits clear.
 */
static BOOL
command_packing_ok (void)
{
    uint32_t move = robot_motion_cmd_move(500u, true, true);
    uint32_t move_max = robot_motion_cmd_move(1023u, false, false);
    uint32_t move_over = robot_motion_cmd_move(5000u, false, false);
    uint32_t turn = robot_motion_cmd_turn(90u, true, false);
    uint32_t turn_max = robot_motion_cmd_turn(180u, false, true);
    uint32_t turn_over = robot_motion_cmd_turn(181u, false, false);
    uint32_t follow = robot_motion_cmd_follow();
    uint32_t stop = robot_motion_cmd_stop();

    BOOL b_move = (ROBOT_MOTION_CMD_MOVE == robot_motion_cmd_get_type(move)) &&
                  (500u == robot_motion_cmd_get_distance(move)) &&
                  robot_motion_cmd_get_reverse(move) &&
                  robot_motion_cmd_get_stop_at_line(move) &&
                  (0u == robot_motion_cmd_get_angle(move)) &&
                  (1023u == robot_motion_cmd_get_distance(move_max)) &&
                  !robot_motion_cmd_get_reverse(move_max) &&
                  !robot_motion_cmd_get_stop_at_line(move_max) &&
                  (0u == robot_motion_cmd_get_angle(move_max)) &&
                  (1023u == robot_motion_cmd_get_distance(move_over));
    BOOL b_turn = (ROBOT_MOTION_CMD_TURN == robot_motion_cmd_get_type(turn)) &&
                  (90u == robot_motion_cmd_get_angle(turn)) &&
                  robot_motion_cmd_get_reverse(turn) &&
                  !robot_motion_cmd_get_stop_at_line(turn) &&
                  (180u == robot_motion_cmd_get_angle(turn_max)) &&
                  robot_motion_cmd_get_stop_at_line(turn_max) &&
                  (0u == robot_motion_cmd_get_distance(turn_max)) &&
                  (180u == robot_motion_cmd_get_angle(turn_over));
    BOOL b_other =
        (ROBOT_MOTION_CMD_FOLLOW == robot_motion_cmd_get_type(follow)) &&
        (ROBOT_MOTION_CMD_STOP == robot_motion_cmd_get_type(stop)) &&
        (0u == stop);
    uint32_t all =
        move | move_max | move_over | turn | turn_max | turn_over | follow;
    return b_move && b_turn && b_other &&
           (0u == (all & ROBOT_MOTION_CMD_RESERVED_MASK));
}

/*
 * Let tMotion run for TASK_RUN_MS while every other task runs, then check
 * each period between its updates is MOTION_UPDATE_PERIOD_MS within
 * PERIOD_JITTER_US. Ends with tMotion held.
 */
static BOOL
task_timing_ok (void)
{
    motion_task_timing_t timing;
    uint32_t period_us = MOTION_UPDATE_PERIOD_MS * US_PER_MS;
    /* Hand-over at each end may cost up to a period each. */
    uint32_t min_updates = (TASK_RUN_MS / MOTION_UPDATE_PERIOD_MS) - 2u;

    if ((motion_task_hold(false) != E_OK) ||
        (tk_dly_tsk(TASK_RUN_MS) != E_OK) || (motion_task_hold(true) != E_OK))
    {
        return FALSE;
    }
    motion_task_get_timing(&timing);
    tm_printf((UB *)"[TEST] MOT-U34 %d updates, period %d..%d us "
                    "(%d expected), longest update %d us\n",
              (INT)timing.updates, (INT)timing.min_period_us,
              (INT)timing.max_period_us, (INT)period_us,
              (INT)timing.max_busy_us);
    return (timing.updates >= min_updates) &&
           (timing.min_period_us + PERIOD_JITTER_US >= period_us) &&
           (timing.max_period_us <= period_us + PERIOD_JITTER_US);
}

/*
 * Through MBF_MOTION_CMD, with tMotion running: a word with reserved bits,
 * a 0 mm move and a turn above 180 degrees are refused, a stop is not.
 * Then INHIBIT: mode INHIBITED, a command sent meanwhile is discarded
 * (an invalid one, so the wheels never move even if it ran), and clearing
 * it returns to STOPPED. Nothing here drives the motors. Ends held.
 */
static BOOL
task_refusals_ok (void)
{
    T_RMBF rmbf;
    uint32_t bad_turn = (uint32_t)ROBOT_MOTION_CMD_TURN |
                        (200u << ROBOT_MOTION_CMD_ANGLE_SHIFT);

    if (motion_task_hold(false) != E_OK)
    {
        return FALSE;
    }
    (void)tk_clr_flg(motion_flg_event, 0u);

    BOOL b_refused = send_command(ROBOT_MOTION_CMD_RESERVED_MASK) &&
                     (MOTION_EVT_REJECTED ==
                      wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS)) &&
                     send_command(robot_motion_cmd_move(0u, false, false)) &&
                     (MOTION_EVT_REJECTED ==
                      wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS)) &&
                     send_command(bad_turn) &&
                     (MOTION_EVT_REJECTED ==
                      wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS));
    BOOL b_stop = send_command(robot_motion_cmd_stop()) &&
                  (0u == wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS)) &&
                  wait_mode(ROBOT_MOTION_MODE_STOPPED, TASK_REACT_MS);

    uint32_t set_us = (uint32_t)in_w(TIMER_TIMERAWL);
    (void)tk_set_flg(motion_flg_ctrl, MOTION_FLG_INHIBIT);
    BOOL b_inhibited =
        wait_mode(ROBOT_MOTION_MODE_INHIBITED, TASK_REACT_MS) &&
        send_command(ROBOT_MOTION_CMD_RESERVED_MASK) &&
        (0u == wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS)) &&
        (tk_ref_mbf(motion_mbf_cmd, &rmbf) == E_OK) && (0 == rmbf.msgsz) &&
        (ROBOT_MOTION_MODE_INHIBITED == motion_get_status().mode);
    (void)tk_clr_flg(motion_flg_ctrl, ~(UINT)MOTION_FLG_INHIBIT);
    BOOL b_cleared = wait_mode(ROBOT_MOTION_MODE_STOPPED, TASK_REACT_MS) &&
                     (0u == wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS));

    BOOL b_held = (motion_task_hold(true) == E_OK);
    BOOL b_latency = b_held && inhibit_latency_ok("MOT-U35", set_us);
    tm_printf((UB *)"[TEST] MOT-U35 refused %d stop %d inhibited %d "
                    "cleared %d\n",
              (INT)b_refused, (INT)b_stop, (INT)b_inhibited, (INT)b_cleared);
    return b_refused && b_stop && b_inhibited && b_cleared && b_latency &&
           slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
           slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
}

/*
 * With tMotion running and the car at rest, motion_get_odometry() refuses
 * NULL, its update count rises by one per period over SHARED_CHECK_MS,
 * distance and heading hold and every speed is 0. Ends held.
 */
static BOOL
shared_odometry_ok (void)
{
    motion_shared_odometry_t first = {0};
    motion_shared_odometry_t last = {0};
    uint32_t expected = SHARED_CHECK_MS / MOTION_UPDATE_PERIOD_MS;

    if ((motion_task_hold(false) != E_OK) ||
        (tk_dly_tsk(MOTION_UPDATE_PERIOD_MS) != E_OK))
    {
        return FALSE;
    }
    BOOL b_read = !motion_get_odometry(NULL) && motion_get_odometry(&first) &&
                  (tk_dly_tsk(SHARED_CHECK_MS) == E_OK) &&
                  motion_get_odometry(&last);
    BOOL b_held = (motion_task_hold(true) == E_OK);
    uint32_t updates = last.updates - first.updates;

    tm_printf((UB *)"[TEST] MOT-U38 %d updates in %d ms (%d expected), "
                    "distance %d mm, heading %d deg\n",
              (INT)updates, SHARED_CHECK_MS, (INT)expected,
              (INT)last.distance_mm, (INT)last.heading_deg);
    return b_read && b_held && (updates + SHARED_UPDATE_SLACK >= expected) &&
           (updates <= expected + SHARED_UPDATE_SLACK) &&
           (first.distance_mm == last.distance_mm) &&
           (first.heading_deg == last.heading_deg) && (0 == last.speed_mm_s) &&
           (0 == last.left_speed_mm_s) && (0 == last.right_speed_mm_s);
}

/* Queue one command for tMotion, as tMission does. */
static BOOL
send_command (uint32_t cmd)
{
    return tk_snd_mbf(motion_mbf_cmd, &cmd, (INT)sizeof(cmd),
                      (TMO)TASK_REACT_MS) == E_OK;
}

/* Wait for any of @p bits; clear and return those set, or 0 on timeout. */
static UINT
wait_events (UINT bits, INT timeout_ms)
{
    UINT flags = 0u;

    if (tk_wai_flg(motion_flg_event, bits, TWF_ORW, &flags, (TMO)timeout_ms) !=
        E_OK)
    {
        return 0u;
    }
    (void)tk_clr_flg(motion_flg_event, ~(flags & bits));
    return flags & bits;
}

/* Poll motion_get_status() each period until it reports @p mode. */
static BOOL
wait_mode (robot_motion_mode_t mode, INT timeout_ms)
{
    for (INT elapsed = 0; elapsed < timeout_ms;
         elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
    {
        if (motion_get_status().mode == mode)
        {
            return TRUE;
        }
        (void)tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
    }
    return motion_get_status().mode == mode;
}

/*
 * Print and check the time from @p set_us (TIMER read just before setting
 * INHIBIT) to tMotion braking (NFR6). Call while tMotion is held.
 */
static BOOL
inhibit_latency_ok (char const * p_id, uint32_t set_us)
{
    motion_task_timing_t timing;

    motion_task_get_timing(&timing);
    uint32_t latency_us = timing.inhibit_brake_us - set_us;
    tm_printf((UB *)"[TEST] %s INHIBIT to brake %d us (limit %d)\n", p_id,
              (INT)latency_us, (INT)INHIBIT_LIMIT_US);
    return latency_us <= INHIBIT_LIMIT_US;
}

#if MOTION_TASK_TESTS
/*
 * On the lifted car, through MBF_MOTION_CMD: (a) a move ends with
 * MOVE_DONE (DIST); (b) a move replaced by another raises MOVE_DONE only
 * for the second; (c) STOP ends a move without MOVE_DONE; (d) INHIBIT
 * brakes a move within NFR6 and discards a move sent meanwhile; (e) FOLLOW
 * runs on the interim line input and ends with FOLLOW_ENDED once readings
 * stop. Ends with tMotion running.
 */
static BOOL
task_commands_lifted (void)
{
    UINT const any_end = MOTION_EVT_MOVE_DONE | MOTION_EVT_STALL |
                         MOTION_EVT_ENC_FAULT | MOTION_EVT_REJECTED;

    tm_printf((UB *)"[TEST] MOT-U36 keep the car lifted: the wheels spin "
                    "in %d s, for about 20 s\n",
              TASK_START_DELAY_MS / 1000);
    if ((motion_task_hold(false) != E_OK) ||
        (tk_dly_tsk(TASK_START_DELAY_MS) != E_OK))
    {
        return FALSE;
    }
    (void)tk_clr_flg(motion_flg_event, 0u);

    /* (a) */
    BOOL b_done =
        send_command(robot_motion_cmd_move(TASK_SHORT_MM, false, false)) &&
        (MOTION_EVT_MOVE_DONE == wait_events(any_end, TASK_SHORT_DONE_MS)) &&
        (ROBOT_MOTION_DONE_DIST == motion_get_status().done) &&
        (ROBOT_MOTION_MODE_STOPPED == motion_get_status().mode);

    /* (b) */
    BOOL b_replaced =
        send_command(robot_motion_cmd_move(TASK_LONG_MM, false, false)) &&
        (0u == wait_events(any_end, TASK_INTERRUPT_MS)) &&
        send_command(robot_motion_cmd_move(TASK_SHORT_MM, true, false)) &&
        (MOTION_EVT_MOVE_DONE == wait_events(any_end, TASK_SHORT_DONE_MS));

    /* (c) */
    BOOL b_stopped =
        send_command(robot_motion_cmd_move(TASK_LONG_MM, false, false)) &&
        (tk_dly_tsk(TASK_INTERRUPT_MS) == E_OK) &&
        send_command(robot_motion_cmd_stop()) &&
        wait_mode(ROBOT_MOTION_MODE_STOPPED, TASK_REACT_MS) &&
        (0u == wait_events(any_end, TASK_QUIET_MS));

    /* (d) */
    BOOL b_inhibited =
        send_command(robot_motion_cmd_move(TASK_LONG_MM, true, false)) &&
        (tk_dly_tsk(TASK_INTERRUPT_MS) == E_OK);
    uint32_t set_us = (uint32_t)in_w(TIMER_TIMERAWL);
    (void)tk_set_flg(motion_flg_ctrl, MOTION_FLG_INHIBIT);
    b_inhibited =
        b_inhibited && wait_mode(ROBOT_MOTION_MODE_INHIBITED, TASK_REACT_MS) &&
        send_command(robot_motion_cmd_move(TASK_SHORT_MM, false, false)) &&
        (tk_dly_tsk(TASK_REACT_MS) == E_OK) &&
        (ROBOT_MOTION_MODE_INHIBITED == motion_get_status().mode);
    (void)tk_clr_flg(motion_flg_ctrl, ~(UINT)MOTION_FLG_INHIBIT);
    b_inhibited = b_inhibited &&
                  wait_mode(ROBOT_MOTION_MODE_STOPPED, TASK_REACT_MS) &&
                  (0u == wait_events(any_end, TASK_QUIET_MS)) &&
                  (motion_task_hold(true) == E_OK) &&
                  inhibit_latency_ok("MOT-U36", set_us) &&
                  (motion_task_hold(false) == E_OK);

    /* (e) Readings first, so FOLLOW sees a fresh, debounced line. */
    BOOL b_follow = TRUE;
    for (INT elapsed = 0; elapsed < TASK_FOLLOW_MS;
         elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
    {
        motion_set_line_input(0, true);
        if ((elapsed == TASK_REACT_MS) &&
            !send_command(robot_motion_cmd_follow()))
        {
            b_follow = FALSE;
        }
        (void)tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
    }
    b_follow =
        b_follow && (ROBOT_MOTION_MODE_FOLLOW == motion_get_status().mode) &&
        (MOTION_EVT_FOLLOW_ENDED ==
         wait_events(MOTION_EVT_FOLLOW_ENDED | any_end,
                     (INT)MOTION_LINE_TIMEOUT_MS + TASK_FOLLOW_END_MS)) &&
        (ROBOT_MOTION_MODE_STOPPED == motion_get_status().mode);

    tm_printf((UB *)"[TEST] MOT-U36 (a) done %d (b) replaced %d (c) stopped "
                    "%d (d) inhibited %d (e) follow %d\n",
              (INT)b_done, (INT)b_replaced, (INT)b_stopped, (INT)b_inhibited,
              (INT)b_follow);
    return b_done && b_replaced && b_stopped && b_inhibited && b_follow;
}

/*
 * On the lifted car with the left wheel held: a move through tMotion
 * raises STALL, mode STALLED; a new move is refused until STOP.
 */
static BOOL
task_stall_lifted (void)
{
    tm_printf((UB *)"[TEST] MOT-U37 keep the car lifted and hold the LEFT "
                    "wheel still; the right wheel spins in %d s\n",
              TASK_START_DELAY_MS / 1000);
    (void)tk_dly_tsk(TASK_START_DELAY_MS);
    (void)tk_clr_flg(motion_flg_event, 0u);

    BOOL b_stalled =
        send_command(robot_motion_cmd_move(TASK_LONG_MM, false, false)) &&
        (MOTION_EVT_STALL ==
         wait_events(MOTION_EVT_STALL | MOTION_EVT_MOVE_DONE |
                         MOTION_EVT_ENC_FAULT,
                     TASK_STALL_WAIT_MS)) &&
        (ROBOT_MOTION_MODE_STALLED == motion_get_status().mode);
    BOOL b_refused =
        send_command(robot_motion_cmd_move(TASK_SHORT_MM, false, false)) &&
        (MOTION_EVT_REJECTED ==
         wait_events(MOTION_EVT_REJECTED, TASK_REACT_MS)) &&
        (ROBOT_MOTION_MODE_STALLED == motion_get_status().mode);
    BOOL b_cleared = send_command(robot_motion_cmd_stop()) &&
                     wait_mode(ROBOT_MOTION_MODE_STOPPED, TASK_REACT_MS);

    tm_printf((UB *)"[TEST] MOT-U37 stalled %d refused %d cleared %d; "
                    "let go\n",
              (INT)b_stalled, (INT)b_refused, (INT)b_cleared);
    return b_stalled && b_refused && b_cleared;
}
#endif /* MOTION_TASK_TESTS */

#if MOTION_MANUAL_TESTS
/* Print both counts while a tester turns the wheels by hand. */
static void
print_counts_while_turning (void)
{
    motion_counts_t counts;

    tm_printf((UB *)"[TEST] MOT-U04/U05 turn each wheel one revolution "
                    "forward, then back, within %d s (expect %d per rev)\n",
              TURN_WINDOW_MS / 1000, (INT)MOTION_ENCODER_TICKS_PER_REV);
    for (INT elapsed = 0; elapsed < TURN_WINDOW_MS; elapsed += TURN_PRINT_MS)
    {
        if ((tk_dly_tsk(TURN_PRINT_MS) != E_OK) ||
            (motion_drive_get_counts(&counts) != MOTION_OK))
        {
            tm_printf((UB *)"[TEST] MOT-U04/U05 encoder read failed\n");
            return;
        }
        /* Raw phase levels and PCs show wiring faults apart from code. */
        UW levels = in_w(GPIO_IN);
        tm_printf((UB *)"[TEST] MOT-U04/U05 left %d right %d  "
                        "pins L%d%d R%d%d  pc %d %d\n",
                  (INT)counts.left_count, (INT)counts.right_count,
                  (INT)pin_level(levels, MOTION_LEFT_ENCODER_PIN_A),
                  (INT)pin_level(levels, MOTION_LEFT_ENCODER_PIN_B),
                  (INT)pin_level(levels, MOTION_RIGHT_ENCODER_PIN_A),
                  (INT)pin_level(levels, MOTION_RIGHT_ENCODER_PIN_B),
                  (INT)in_w(PIO0_SM2_ADDR), (INT)in_w(PIO0_SM3_ADDR));
    }
}

/* Spin each wheel alone, forward then backward, and print its counts. */
static void
spin_each_wheel (void)
{
    static spin_step_t const steps[] = {
        {MOTOR_LEFT, SPIN_DUTY, "LEFT wheel FORWARD"},
        {MOTOR_LEFT, -SPIN_DUTY, "LEFT wheel BACKWARD"},
        {MOTOR_RIGHT, SPIN_DUTY, "RIGHT wheel FORWARD"},
        {MOTOR_RIGHT, -SPIN_DUTY, "RIGHT wheel BACKWARD"},
    };
    motion_counts_t before;
    motion_counts_t after;

    tm_printf((UB *)"[TEST] MOT-U06 keep the car lifted: each wheel spins "
                    "alone in %d s\n",
              SPIN_START_DELAY_MS / 1000);
    if (tk_dly_tsk(SPIN_START_DELAY_MS) != E_OK)
    {
        return;
    }

    for (UW i = 0u; i < (sizeof(steps) / sizeof(steps[0])); i++)
    {
        tm_printf((UB *)"[TEST] MOT-U06 %s\n", steps[i].p_name);
        (void)motion_drive_get_counts(&before);
        motor_set_duty(steps[i].id, steps[i].duty);
        ER er = tk_dly_tsk(SPIN_MS);
        motor_brake(steps[i].id);
        if ((er != E_OK) || (motion_drive_get_counts(&after) != MOTION_OK))
        {
            tm_printf((UB *)"[TEST] MOT-U06 stopped early\n");
            return;
        }
        tm_printf((UB *)"[TEST] MOT-U06   encoder change left %d right %d\n",
                  (INT)(int16_t)(after.left_count - before.left_count),
                  (INT)(int16_t)(after.right_count - before.right_count));
        (void)tk_dly_tsk(SPIN_PAUSE_MS);
    }
}

/* Level (0 or 1) of @p pin in a GPIO_IN snapshot. */
static UW
pin_level (UW levels, UW pin)
{
    return (levels >> pin) & 1u;
}
#endif /* MOTION_MANUAL_TESTS */

#if MOTION_MOVE_TESTS
/* Run one move on the floor and print the odometry to compare with a
 * ruler. */
static void
run_move (char const * p_id, uint16_t distance_mm, bool b_forward)
{
    motion_odometry_t start;
    motion_odometry_t end;

    tm_printf((UB *)"[TEST] %s car on the floor: mark its position; it "
                    "drives %s %d mm in %d s\n",
              p_id, b_forward ? "FORWARD" : "BACKWARD", (INT)distance_mm,
              MOVE_START_DELAY_MS / 1000);
    if (tk_dly_tsk(MOVE_START_DELAY_MS) != E_OK)
    {
        return;
    }

    (void)motion_move_update();
    (void)motion_move_get_odometry(&start);
    motion_status_t status = b_forward ? motion_move_forward(distance_mm)
                                       : motion_move_backward(distance_mm);
    if (status != MOTION_OK)
    {
        tm_printf((UB *)"[TEST] %s move refused (%d)\n", p_id, (INT)status);
        return;
    }

    INT moving_ms = update_while_moving();
    motion_state_t end_state = motion_move_get_state();
    (void)motion_move_stop();
    /* Keep counting while the car coasts to rest. */
    update_for(MOVE_SETTLE_MS);
    (void)motion_move_get_odometry(&end);
    tm_printf((UB *)"[TEST] %s state %d after %d ms; odometry %d mm "
                    "(target %d): ruler must agree within 3 percent\n",
              p_id, (INT)end_state, moving_ms,
              (INT)(end.distance_mm - start.distance_mm),
              b_forward ? (INT)distance_mm : -(INT)distance_mm);
}
#endif /* MOTION_MOVE_TESTS */

#if MOTION_TURN_TESTS
/*
 * Run @p count turns on the floor, @p pause_ms apart, and print each one's
 * encoder heading at the brake and after the coast, to compare with the
 * measured angle.
 */
static void
run_turns (char const * p_id, turn_kind_t kind, uint16_t angle_deg, INT count,
           INT pause_ms)
{
    static char const * const p_names[] = {"LEFT", "RIGHT", "U-TURN"};
    INT sign = (TURN_RIGHT == kind) ? -1 : 1;
    motion_odometry_t first;
    motion_odometry_t start;
    motion_odometry_t braked;
    motion_odometry_t end;

    tm_printf((UB *)"[TEST] %s car on the floor: mark its heading; it turns "
                    "%s %d deg, %d times, in %d s\n",
              p_id, p_names[kind], (INT)angle_deg, count,
              MOVE_START_DELAY_MS / 1000);
    if (tk_dly_tsk(MOVE_START_DELAY_MS) != E_OK)
    {
        return;
    }

    (void)motion_move_update();
    (void)motion_move_get_odometry(&first);
    for (INT run = 1; run <= count; run++)
    {
        (void)motion_move_get_odometry(&start);
        motion_status_t status = start_turn(kind, angle_deg);
        if (status != MOTION_OK)
        {
            tm_printf((UB *)"[TEST] %s turn refused (%d)\n", p_id, (INT)status);
            return;
        }

        INT turning_ms = update_while_moving();
        motion_state_t end_state = motion_move_get_state();
        (void)motion_move_get_odometry(&braked);
        (void)motion_move_stop();
        /* Keep counting while the car coasts to rest. */
        update_for(MOVE_SETTLE_MS);
        (void)motion_move_get_odometry(&end);
        tm_printf((UB *)"[TEST] %s run %d state %d after %d ms; heading %d "
                        "deg (target %d), coast %d\n",
                  p_id, run, (INT)end_state, turning_ms,
                  (INT)(end.heading_deg - start.heading_deg),
                  sign * (INT)angle_deg,
                  (INT)(end.heading_deg - braked.heading_deg));

        if ((pause_ms > 0) && (run < count))
        {
            tm_printf((UB *)"[TEST] %s measure, then re-mark; next turn in "
                            "%d s\n",
                      p_id, pause_ms / 1000);
            update_for(pause_ms);
        }
    }

    if (count > 1)
    {
        tm_printf((UB *)"[TEST] %s total heading %d deg (target %d): "
                        "measure it\n",
                  p_id, (INT)(end.heading_deg - first.heading_deg),
                  sign * count * (INT)angle_deg);
    }
}

/* Start a turn of @p kind; @p angle_deg is ignored for a U-turn. */
static motion_status_t
start_turn (turn_kind_t kind, uint16_t angle_deg)
{
    if (TURN_RIGHT == kind)
    {
        return motion_move_turn_right(angle_deg);
    }
    if (TURN_U == kind)
    {
        return motion_move_u_turn();
    }
    return motion_move_turn_left(angle_deg);
}

/*
 * On the floor: a turn replaces a move in progress and is measured from
 * where it started, a move replaces a turn, and a stop during a turn
 * brakes at once.
 */
static BOOL
preempt_and_stop (void)
{
    motion_odometry_t start;
    motion_odometry_t end;

    tm_printf((UB *)"[TEST] MOT-U15 car on the floor, 1 m clear ahead: mark "
                    "its heading; it drives forward, turns LEFT %d deg "
                    "partway, starts a turn that a short reverse replaces, "
                    "then a turn that is stopped, in %d s\n",
              (INT)TEST_TURN_DEG, MOVE_START_DELAY_MS / 1000);
    if ((tk_dly_tsk(MOVE_START_DELAY_MS) != E_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_forward(FORWARD_TEST_MM) != MOTION_OK))
    {
        return FALSE;
    }

    update_for(PREEMPT_AFTER_MS);
    BOOL b_was_moving = (MOTION_STATE_MOVE == motion_move_get_state());
    (void)motion_move_get_odometry(&start);
    if (motion_move_turn_left(TEST_TURN_DEG) != MOTION_OK)
    {
        return FALSE;
    }
    BOOL b_replaced = (MOTION_STATE_TURN == motion_move_get_state());

    (void)update_while_moving();
    BOOL b_turn_done = (MOTION_STATE_STOPPED == motion_move_get_state());
    update_for(MOVE_SETTLE_MS);
    (void)motion_move_get_odometry(&end);

    /* A move part way through a turn replaces it and runs to the end. */
    BOOL b_move_replaced = FALSE;
    if (motion_move_turn_left(CAL_TURN_DEG) == MOTION_OK)
    {
        update_for(PREEMPT_STOP_AFTER_MS);
        b_move_replaced =
            (motion_move_backward(PREEMPT_BACK_MM) == MOTION_OK) &&
            (MOTION_STATE_MOVE == motion_move_get_state());
        (void)update_while_moving();
        b_move_replaced = b_move_replaced &&
                          (MOTION_STATE_STOPPED == motion_move_get_state());
        update_for(MOVE_SETTLE_MS);
    }

    /* A stop part way through a turn brakes before the next update. */
    BOOL b_stopped = FALSE;
    if (motion_move_turn_left(CAL_TURN_DEG) == MOTION_OK)
    {
        update_for(PREEMPT_STOP_AFTER_MS);
        b_stopped = (motion_move_stop() == MOTION_OK) &&
                    (MOTION_STATE_STOPPED == motion_move_get_state()) &&
                    !motion_move_is_busy() &&
                    slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                    slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
    }
    update_for(MOVE_SETTLE_MS);

    tm_printf((UB *)"[TEST] MOT-U15 moving %d, replaced %d, turn done %d, "
                    "move replaced turn %d, stopped %d; turn heading %d deg "
                    "(target %d)\n",
              (INT)b_was_moving, (INT)b_replaced, (INT)b_turn_done,
              (INT)b_move_replaced, (INT)b_stopped,
              (INT)(end.heading_deg - start.heading_deg), (INT)TEST_TURN_DEG);
    return b_was_moving && b_replaced && b_turn_done && b_move_replaced &&
           b_stopped;
}

/*
 * On the lifted car, turn and print how the turn ended. With one
 * MOTION_*_ENCODER_INVERTED flag deliberately flipped, the count
 * difference barely moves and the turn must time out.
 */
static void
run_until_timeout (void)
{
    tm_printf((UB *)"[TEST] MOT-U17 keep the car lifted: it turns LEFT %d "
                    "deg in %d s\n",
              (INT)TEST_TURN_DEG, MOVE_START_DELAY_MS / 1000);
    if ((tk_dly_tsk(MOVE_START_DELAY_MS) != E_OK) ||
        (motion_move_update() != MOTION_OK) ||
        (motion_move_turn_left(TEST_TURN_DEG) != MOTION_OK))
    {
        tm_printf((UB *)"[TEST] MOT-U17 turn refused\n");
        return;
    }

    INT turning_ms = update_while_moving();
    motion_state_t end_state = motion_move_get_state();
    motion_status_t retry = motion_move_turn_left(TEST_TURN_DEG);
    (void)motion_move_stop();
    tm_printf((UB *)"[TEST] MOT-U17 state %d after %d ms, retry %d (flag "
                    "flipped: expect state %d after about %d ms, retry %d; "
                    "else state %d)\n",
              (INT)end_state, turning_ms, (INT)retry,
              (INT)MOTION_STATE_TIMED_OUT,
              (INT)(((MOTION_TIMEOUT_FACTOR * TEST_TURN_DEG * 1000u) /
                     (UW)MOTION_TURN_SPEED_DEG_S) +
                    MOTION_TIMEOUT_MARGIN_MS),
              (INT)MOTION_ERR_TIMED_OUT, (INT)MOTION_STATE_STOPPED);
}
#endif /* MOTION_TURN_TESTS */

#if MOTION_SPEED_TESTS
/*
 * On the lifted car, step both wheels through rising duties, open loop,
 * and print each wheel's settled speed, for the feed-forward line.
 */
static void
sweep_feed_forward (void)
{
    motion_odometry_t odometry;

    tm_printf((UB *)"[TEST] MOT-U19 keep the car lifted: both wheels step "
                    "from duty %d to 1000 permille in %d s\n",
              SWEEP_START_PERMILLE, SPEED_START_DELAY_MS / 1000);
    (void)motion_move_stop();
    update_for(SPEED_START_DELAY_MS);

    for (INT permille = SWEEP_START_PERMILLE; permille <= 1000;
         permille += SWEEP_STEP_PERMILLE)
    {
        float duty = (float)permille / 1000.0f;
        INT left_sum = 0;
        INT right_sum = 0;
        INT samples = 0;

        motor_set_duty(MOTOR_LEFT, duty);
        motor_set_duty(MOTOR_RIGHT, duty);
        update_for(SWEEP_SETTLE_MS);
        for (INT elapsed = 0; elapsed < SWEEP_MEASURE_MS;
             elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
        {
            update_for((INT)MOTION_UPDATE_PERIOD_MS);
            (void)motion_move_get_odometry(&odometry);
            left_sum += (INT)odometry.left_speed_mm_s;
            right_sum += (INT)odometry.right_speed_mm_s;
            samples++;
        }
        tm_printf((UB *)"[TEST] MOT-U19 duty %d left %d right %d mm/s\n",
                  permille, left_sum / samples, right_sum / samples);
    }
    (void)motion_move_stop();
    update_for(MOVE_SETTLE_MS);
}

/*
 * On the lifted car, step the speed setpoint with the speed loop on and
 * record every update, for tuning MOTION_SPEED_KP and MOTION_SPEED_KI.
 */
static void
step_speed_lifted (void)
{
    static speed_record_t record;

    tm_printf((UB *)"[TEST] MOT-U20 keep the car lifted: speed steps %d, "
                    "%d, %d mm/s in %d s\n",
              STEP_HIGH_MM_S, STEP_LOW_MM_S, STEP_HIGH_MM_S,
              SPEED_START_DELAY_MS / 1000);
    update_for(SPEED_START_DELAY_MS);

    record.count = 0;
    (void)motion_drive_set_velocity(STEP_HIGH_MM_S, 0);
    record_for(&record, STEP_MS);
    (void)motion_drive_set_velocity(STEP_LOW_MM_S, 0);
    record_for(&record, STEP_MS);
    (void)motion_drive_set_velocity(STEP_HIGH_MM_S, 0);
    record_for(&record, STEP_MS);
    (void)motion_move_stop();
    update_for(MOVE_SETTLE_MS);
    print_record("MOT-U20", "speed loop", &record);
}

/*
 * On the floor, on battery alone, drive the same distance open loop, then
 * with the speed loop; print both runs once USB is plugged back in.
 */
static void
run_floor_unplugged (void)
{
    static speed_record_t open_loop;
    static speed_record_t closed_loop;

    tm_printf((UB *)"[TEST] MOT-U21 unplug USB, put the car on the floor "
                    "with 2 m clear ahead: it drives %d mm open loop in "
                    "%d s, then %d mm with the speed loop\n",
              (INT)FLOOR_RUN_MM, FLOOR_START_DELAY_MS / 1000,
              (INT)FLOOR_RUN_MM);
    update_for(FLOOR_START_DELAY_MS);

    (void)motion_drive_set_speed_pid(false);
    open_loop.count = 0;
    if (motion_move_forward(FLOOR_RUN_MM) == MOTION_OK)
    {
        record_while_moving(&open_loop);
    }
    (void)motion_move_stop();
    update_for(FLOOR_PAUSE_MS);

    (void)motion_drive_set_speed_pid(true);
    closed_loop.count = 0;
    if (motion_move_forward(FLOOR_RUN_MM) == MOTION_OK)
    {
        record_while_moving(&closed_loop);
    }
    (void)motion_move_stop();

    tm_printf((UB *)"[TEST] MOT-U21 done; results print once a terminal is "
                    "open (plug USB back in or restart the terminal)\n");
    wait_for_terminal();
    print_record("MOT-U21", "open loop", &open_loop);
    print_record("MOT-U21", "speed loop", &closed_loop);
}

/*
 * Wait until a terminal has had the USB console open for
 * TERMINAL_STEADY_MS, or TERMINAL_WAIT_MAX_MS at most; with another
 * console, wait PLUG_WAIT_MS.
 */
static void
wait_for_terminal (void)
{
#if TM_CONSOLE_USB_CDC
    INT open_ms = 0;

    for (INT waited_ms = 0;
         (open_ms < TERMINAL_STEADY_MS) && (waited_ms < TERMINAL_WAIT_MAX_MS);
         waited_ms += TERMINAL_POLL_MS)
    {
        update_for(TERMINAL_POLL_MS);
        if (tm_usb_state() == USB_STATE_TERMINAL_OPEN)
        {
            open_ms += TERMINAL_POLL_MS;
        }
        else
        {
            open_ms = 0;
        }
    }
#else
    update_for(PLUG_WAIT_MS);
#endif
}

/* Update every period for @p duration_ms, recording each update. */
static void
record_for (speed_record_t * p_record, INT duration_ms)
{
    motion_odometry_t odometry;

    for (INT elapsed = 0; elapsed < duration_ms;
         elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
    {
        if ((wait_update_period() != E_OK) ||
            (motion_move_update() != MOTION_OK))
        {
            return;
        }
        if (p_record->count < RECORD_MAX_SAMPLES)
        {
            speed_sample_t * p_sample = &p_record->samples[p_record->count];
            (void)motion_drive_get_command(&p_sample->command);
            (void)motion_move_get_odometry(&odometry);
            p_sample->left_mm_s = odometry.left_speed_mm_s;
            p_sample->right_mm_s = odometry.right_speed_mm_s;
            p_record->count++;
        }
    }
}

/* Record every update until the move ends or the record is full. */
static void
record_while_moving (speed_record_t * p_record)
{
    while (motion_move_is_busy() && (p_record->count < RECORD_MAX_SAMPLES))
    {
        INT before = p_record->count;
        record_for(p_record, (INT)MOTION_UPDATE_PERIOD_MS);
        if (p_record->count == before)
        {
            return;
        }
    }
}

/*
 * Print one line per recorded update, then each wheel's mean speed over
 * the second half of the run, once it has settled.
 */
static void
print_record (char const * p_id, char const * p_label,
              speed_record_t const * p_record)
{
    INT left_sum = 0;
    INT right_sum = 0;
    INT half = p_record->count / 2;

    tm_printf((UB *)"[TEST] %s %s: ms setL setR speedL speedR dutyL dutyR\n",
              p_id, p_label);
    for (INT i = 0; i < p_record->count; i++)
    {
        speed_sample_t const * p_sample = &p_record->samples[i];
        tm_printf((UB *)"[TEST] %s %d %d %d %d %d %d %d\n", p_id,
                  (i + 1) * (INT)MOTION_UPDATE_PERIOD_MS,
                  (INT)p_sample->command.left_setpoint_mm_s,
                  (INT)p_sample->command.right_setpoint_mm_s,
                  (INT)p_sample->left_mm_s, (INT)p_sample->right_mm_s,
                  (INT)p_sample->command.left_duty_permille,
                  (INT)p_sample->command.right_duty_permille);
        if (((i + 1) % PRINT_BURST_LINES) == 0)
        {
            wait_console_drained();
        }
        if (i >= half)
        {
            left_sum += (INT)p_sample->left_mm_s;
            right_sum += (INT)p_sample->right_mm_s;
        }
    }
    if (p_record->count > half)
    {
        tm_printf((UB *)"[TEST] %s %s settled: left %d right %d mm/s "
                        "(target %d)\n",
                  p_id, p_label, left_sum / (p_record->count - half),
                  right_sum / (p_record->count - half),
                  (INT)MOTION_MOVE_SPEED_MM_S);
    }
}
#endif /* MOTION_SPEED_TESTS */

#if MOTION_SYNC_TESTS
/*
 * On the lifted car, drive forward then backward with the straight-line
 * loop on while a tester slows the left wheel; print every update, to
 * check the loop steers against the error rather than with it.
 */
static void
check_sync_sign_lifted (void)
{
    static sync_record_t record;

    tm_printf((UB *)"[TEST] MOT-U24 keep the car lifted; while the wheels "
                    "turn, lightly slow the LEFT wheel with a finger, do "
                    "not stop it. FORWARD %d mm, then BACKWARD %d mm, in "
                    "%d s\n",
              (INT)SYNC_FORWARD_MM, (INT)SYNC_FORWARD_MM,
              SYNC_START_DELAY_MS / 1000);
    (void)motion_move_set_sync(true);
    update_for(SYNC_START_DELAY_MS);

    run_sync_move("MOT-U24", "forward", SYNC_FORWARD_MM, true, &record);
    print_sync_trace("MOT-U24", &record);
    run_sync_move("MOT-U24", "backward", SYNC_FORWARD_MM, false, &record);
    print_sync_trace("MOT-U24", &record);
}

/*
 * On the floor, drive 1 m SYNC_RUNS times with the straight-line loop off,
 * then SYNC_RUNS times with it on; the tester measures each run's
 * sideways drift.
 */
static void
measure_drift_floor (void)
{
    static sync_record_t record;
    static char const * const p_labels[] = {"sync OFF", "sync ON"};

    for (UW mode = 0u; mode < 2u; mode++)
    {
        (void)motion_move_set_sync(mode != 0u);
        for (INT run = 1; run <= SYNC_RUNS; run++)
        {
            tm_printf((UB *)"[TEST] MOT-U25 %s run %d: car on the floor on "
                            "the start line, tape along its side; it drives "
                            "FORWARD %d mm in %d s\n",
                      p_labels[mode], run, (INT)SYNC_FORWARD_MM,
                      SYNC_MEASURE_MS / 1000);
            update_for(SYNC_MEASURE_MS);
            run_sync_move("MOT-U25", p_labels[mode], SYNC_FORWARD_MM, true,
                          &record);
        }
    }
    (void)motion_move_set_sync(true);
    tm_printf((UB *)"[TEST] MOT-U25 measure the last run within %d s\n",
              SYNC_MEASURE_MS / 1000);
    update_for(SYNC_MEASURE_MS);
}

/*
 * On the floor with the loop on: drive 1 m while the tester nudges the
 * car, printing every update, then reverse SYNC_BACKWARD_MM.
 */
static void
nudge_and_reverse (void)
{
    static sync_record_t record;

    tm_printf((UB *)"[TEST] MOT-U26 car on the floor, 1 m clear ahead: "
                    "about halfway, push its front sideways about 1 cm. "
                    "FORWARD %d mm in %d s\n",
              (INT)SYNC_FORWARD_MM, SYNC_START_DELAY_MS / 1000);
    update_for(SYNC_START_DELAY_MS);
    run_sync_move("MOT-U26", "nudged", SYNC_FORWARD_MM, true, &record);
    print_sync_trace("MOT-U26", &record);

    tm_printf((UB *)"[TEST] MOT-U26 measure, re-mark; it reverses %d mm in "
                    "%d s\n",
              (INT)SYNC_BACKWARD_MM, SYNC_MEASURE_MS / 1000);
    update_for(SYNC_MEASURE_MS);
    run_sync_move("MOT-U26", "reverse", SYNC_BACKWARD_MM, false, &record);
}

/*
 * Run one move, record every update, and print how it ended: state,
 * distance and heading by odometry after the coast, and the largest
 * straight-line error during the move.
 */
static void
run_sync_move (char const * p_id, char const * p_label, uint16_t distance_mm,
               bool b_forward, sync_record_t * p_record)
{
    motion_odometry_t start;
    motion_odometry_t end;
    INT max_error = 0;
    INT elapsed = 0;

    p_record->count = 0;
    (void)motion_move_update();
    (void)motion_move_get_odometry(&start);
    motion_status_t status = b_forward ? motion_move_forward(distance_mm)
                                       : motion_move_backward(distance_mm);
    if (status != MOTION_OK)
    {
        tm_printf((UB *)"[TEST] %s %s move refused (%d)\n", p_id, p_label,
                  (INT)status);
        return;
    }

    while (motion_move_is_busy() && (elapsed < SYNC_RUN_TIMEOUT_MS))
    {
        if ((wait_update_period() != E_OK) ||
            (motion_move_update() != MOTION_OK))
        {
            break;
        }
        elapsed += (INT)MOTION_UPDATE_PERIOD_MS;

        /* The move's last update has reset the loop; keep its error. */
        if (!motion_move_is_busy())
        {
            continue;
        }
        if (p_record->count < SYNC_MAX_SAMPLES)
        {
            sync_sample_t * p_sample = &p_record->samples[p_record->count];
            (void)motion_move_get_sync(&p_sample->sync);
            (void)motion_drive_get_command(&p_sample->command);
            p_record->count++;
            INT error = (INT)p_sample->sync.error_ticks;
            error = (error < 0) ? -error : error;
            max_error = (error > max_error) ? error : max_error;
        }
    }

    motion_state_t end_state = motion_move_get_state();
    (void)motion_move_stop();
    /* Keep counting while the car coasts to rest. */
    update_for(MOVE_SETTLE_MS);
    (void)motion_move_get_odometry(&end);
    tm_printf((UB *)"[TEST] %s %s state %d after %d ms; odometry %d mm "
                    "(target %d), heading %d deg, max error %d ticks\n",
              p_id, p_label, (INT)end_state, elapsed,
              (INT)(end.distance_mm - start.distance_mm),
              b_forward ? (INT)distance_mm : -(INT)distance_mm,
              (INT)(end.heading_deg - start.heading_deg), max_error);
}

/* Print one line per recorded update of a move. */
static void
print_sync_trace (char const * p_id, sync_record_t const * p_record)
{
    tm_printf((UB *)"[TEST] %s trace: ms errTicks corrDegS setL setR\n", p_id);
    for (INT i = 0; i < p_record->count; i++)
    {
        sync_sample_t const * p_sample = &p_record->samples[i];
        tm_printf((UB *)"[TEST] %s %d %d %d %d %d\n", p_id,
                  (i + 1) * (INT)MOTION_UPDATE_PERIOD_MS,
                  (INT)p_sample->sync.error_ticks,
                  (INT)p_sample->sync.correction_deg_s,
                  (INT)p_sample->command.left_setpoint_mm_s,
                  (INT)p_sample->command.right_setpoint_mm_s);
        if (((i + 1) % PRINT_BURST_LINES) == 0)
        {
            wait_console_drained();
        }
    }
}
#endif /* MOTION_SYNC_TESTS */

#if MOTION_SPEED_TESTS || MOTION_SYNC_TESTS
/*
 * Wait until the USB console has sent everything queued, for at most
 * PRINT_DRAIN_MAX_MS; with another console, just pause.
 */
static void
wait_console_drained (void)
{
#if TM_CONSOLE_USB_CDC
    for (INT waited_ms = 0;
         (tm_usb_pending() > 0u) && (waited_ms < PRINT_DRAIN_MAX_MS);
         waited_ms += PRINT_PAUSE_MS)
    {
        (void)tk_dly_tsk((TMO)PRINT_PAUSE_MS);
    }
#else
    (void)tk_dly_tsk((TMO)PRINT_PAUSE_MS);
#endif
}
#endif

#if MOTION_FOLLOW_TESTS
/*
 * On the lifted car, follow with hand-fed errors 0, +, 0, -: a + error
 * (line to the left) must speed the right wheel up over the left. Then
 * stop feeding errors: following must brake after MOTION_LINE_TIMEOUT_MS.
 */
static BOOL
check_follow_lifted (void)
{
    static int16_t const errors[] = {0, FOLLOW_TEST_ERROR, 0,
                                     -FOLLOW_TEST_ERROR};
    motion_command_t command;
    motion_odometry_t odometry;
    motion_follow_t follow;
    BOOL b_steered = TRUE;

    tm_printf((UB *)"[TEST] MOT-U28 keep the car lifted: it follows at %d "
                    "mm/s with line errors 0, %d, 0, %d, %d s each, in %d s\n",
              (INT)MOTION_FOLLOW_SPEED_MM_S, FOLLOW_TEST_ERROR,
              -FOLLOW_TEST_ERROR, FOLLOW_STEP_MS / 1000,
              FOLLOW_START_DELAY_MS / 1000);
    (void)motion_move_stop();
    update_for(FOLLOW_START_DELAY_MS);

    (void)motion_move_set_line(0, true);
    if (motion_move_follow(MOTION_FOLLOW_SPEED_MM_S) != MOTION_OK)
    {
        return FALSE;
    }
    for (UW i = 0u; i < (sizeof(errors) / sizeof(errors[0])); i++)
    {
        feed_line_for(errors[i], true, FOLLOW_STEP_MS);
        (void)motion_drive_get_command(&command);
        (void)motion_move_get_odometry(&odometry);
        (void)motion_move_get_follow(&follow);
        tm_printf((UB *)"[TEST] MOT-U28 error %d turn %d deg/s set L %d R %d "
                        "speed L %d R %d mm/s state %d\n",
                  (INT)errors[i], (INT)follow.turn_deg_s,
                  (INT)command.left_setpoint_mm_s,
                  (INT)command.right_setpoint_mm_s,
                  (INT)odometry.left_speed_mm_s, (INT)odometry.right_speed_mm_s,
                  (INT)motion_move_get_state());

        INT difference =
            (INT)command.right_setpoint_mm_s - (INT)command.left_setpoint_mm_s;
        if (errors[i] > 0)
        {
            b_steered = b_steered && (difference > 0);
        }
        else if (errors[i] < 0)
        {
            b_steered = b_steered && (difference < 0);
        }
        else
        {
            b_steered = b_steered && (0 == difference);
        }
        b_steered =
            b_steered && (MOTION_STATE_FOLLOW == motion_move_get_state());
    }

    /* No more line errors: following must stop on its own. */
    INT elapsed = 0;
    while ((MOTION_STATE_FOLLOW == motion_move_get_state()) &&
           (elapsed < FOLLOW_END_WAIT_MS))
    {
        update_for((INT)MOTION_UPDATE_PERIOD_MS);
        elapsed += (INT)MOTION_UPDATE_PERIOD_MS;
    }
    BOOL b_timed_out = (MOTION_STATE_STOPPED == motion_move_get_state()) &&
                       (elapsed <= (INT)MOTION_LINE_TIMEOUT_MS +
                                       (2 * (INT)MOTION_UPDATE_PERIOD_MS)) &&
                       slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                       slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
    tm_printf((UB *)"[TEST] MOT-U28 steered %d; stopped %d ms after the "
                    "last error (limit %d)\n",
              (INT)b_steered, elapsed, (INT)MOTION_LINE_TIMEOUT_MS);
    (void)motion_move_stop();
    update_for(MOVE_SETTLE_MS);
    return b_steered && b_timed_out;
}

/*
 * On the lifted car, with hand-fed line readings: (a) a move asked to stop
 * at the line ends as it comes into view; (b) a turn that starts on the
 * line ignores it until it has gone and come back; (c) a move not asked to
 * stop drives past it; (d) one asked to stop that never sees it ends at
 * its distance.
 */
static BOOL
check_stop_at_line_lifted (void)
{
    tm_printf((UB *)"[TEST] MOT-U29 keep the car lifted: four short moves "
                    "and turns with a pretend line, in %d s\n",
              FOLLOW_START_DELAY_MS / 1000);
    (void)motion_move_stop();
    update_for(FOLLOW_START_DELAY_MS);

    /* (a) Gone, then seen: ends within the debounce. */
    feed_line_for(0, false, LINE_GONE_MS);
    BOOL b_a = (motion_move_forward(1000u) == MOTION_OK) &&
               (motion_move_stop_at_line() == MOTION_OK);
    feed_line_for(0, false, LINE_GONE_MS);
    INT updates_a = feed_line_until_idle(true);
    b_a = b_a && (MOTION_DONE_LINE == motion_move_get_done_reason()) &&
          (updates_a <= (INT)MOTION_LINE_SEEN_SAMPLES);
    update_for(MOVE_SETTLE_MS);

    /* (b) Starts on the line: keeps turning until it is gone and back. */
    feed_line_for(0, true, LINE_ON_MS);
    BOOL b_b = (motion_move_turn_left(LINE_TURN_DEG) == MOTION_OK) &&
               (motion_move_stop_at_line() == MOTION_OK);
    feed_line_for(0, true, LINE_ON_MS);
    b_b = b_b && (MOTION_STATE_TURN == motion_move_get_state());
    feed_line_for(0, false, LINE_GONE_MS);
    INT updates_b = feed_line_until_idle(true);
    b_b = b_b && (MOTION_DONE_LINE == motion_move_get_done_reason()) &&
          (updates_b <= (INT)MOTION_LINE_SEEN_SAMPLES);
    update_for(MOVE_SETTLE_MS);

    /* (c) Not asked to stop: crosses the line, ends at its distance. */
    feed_line_for(0, false, LINE_GONE_MS);
    BOOL b_c = (motion_move_forward(LINE_MOVE_MM) == MOTION_OK);
    feed_line_for(0, false, LINE_GONE_MS / 2);
    (void)feed_line_until_idle(true);
    b_c = b_c && (MOTION_DONE_TARGET == motion_move_get_done_reason());
    update_for(MOVE_SETTLE_MS);

    /* (d) Asked to stop, never sees it: ends at its distance. */
    feed_line_for(0, false, LINE_GONE_MS);
    BOOL b_d = (motion_move_forward(LINE_MOVE_MM) == MOTION_OK) &&
               (motion_move_stop_at_line() == MOTION_OK);
    (void)feed_line_until_idle(false);
    b_d = b_d && (MOTION_DONE_TARGET == motion_move_get_done_reason());

    tm_printf((UB *)"[TEST] MOT-U29 (a) %d after %d updates, (b) %d after "
                    "%d, (c) %d, (d) %d; state %d\n",
              (INT)b_a, updates_a, (INT)b_b, updates_b, (INT)b_c, (INT)b_d,
              (INT)motion_move_get_state());
    (void)motion_move_stop();
    update_for(MOVE_SETTLE_MS);
    return b_a && b_b && b_c && b_d;
}

/*
 * On the floor, follow with hand-fed errors +, 0, -: the car must arc
 * left, run straight, then arc right. Print each phase's heading change by
 * odometry against the turn rate asked for.
 */
static void
follow_arcs_floor (void)
{
    static int16_t const errors[] = {FOLLOW_TEST_ERROR, 0, -FOLLOW_TEST_ERROR};
    motion_odometry_t start;
    motion_odometry_t end;
    motion_follow_t follow;

    tm_printf((UB *)"[TEST] MOT-U30 car on the floor, 1 m clear all round: "
                    "it arcs LEFT, runs straight, then arcs RIGHT, %d s "
                    "each, in %d s\n",
              FOLLOW_STEP_MS / 1000, FOLLOW_START_DELAY_MS / 1000);
    (void)motion_move_stop();
    update_for(FOLLOW_START_DELAY_MS);

    (void)motion_move_set_line(errors[0], true);
    if (motion_move_follow(MOTION_FOLLOW_SPEED_MM_S) != MOTION_OK)
    {
        tm_printf((UB *)"[TEST] MOT-U30 follow refused\n");
        return;
    }
    for (UW i = 0u; i < (sizeof(errors) / sizeof(errors[0])); i++)
    {
        (void)motion_move_get_odometry(&start);
        feed_line_for(errors[i], true, FOLLOW_STEP_MS);
        (void)motion_move_get_odometry(&end);
        (void)motion_move_get_follow(&follow);
        tm_printf((UB *)"[TEST] MOT-U30 error %d: heading %d deg (asked %d), "
                        "distance %d mm, state %d\n",
                  (INT)errors[i], (INT)(end.heading_deg - start.heading_deg),
                  ((INT)follow.turn_deg_s * FOLLOW_STEP_MS) / 1000,
                  (INT)(end.distance_mm - start.distance_mm),
                  (INT)motion_move_get_state());
    }
    (void)motion_move_stop();
    update_for(MOVE_SETTLE_MS);
}

/* Pass in one line reading, then update, every period for @p duration_ms. */
static void
feed_line_for (int16_t error, bool b_seen, INT duration_ms)
{
    for (INT elapsed = 0; elapsed < duration_ms;
         elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
    {
        (void)motion_move_set_line(error, b_seen);
        (void)wait_update_period();
        (void)motion_move_update();
    }
}

/*
 * Feed line readings until the move or turn ends, for at most
 * FOLLOW_END_WAIT_MS; return the updates it took.
 */
static INT
feed_line_until_idle (bool b_seen)
{
    INT updates = 0;

    while (motion_move_is_busy() &&
           ((updates * (INT)MOTION_UPDATE_PERIOD_MS) < FOLLOW_END_WAIT_MS))
    {
        feed_line_for(0, b_seen, (INT)MOTION_UPDATE_PERIOD_MS);
        updates++;
    }
    return updates;
}
#endif /* MOTION_FOLLOW_TESTS */

#if MOTION_MOVE_TESTS || MOTION_TURN_TESTS
/*
 * Drive the lifted car with the left wheel held, forward or (@p b_turn)
 * turning left; check the stall.
 */
static BOOL
hold_wheel_until_stall (char const * p_id, bool b_turn)
{
    tm_printf((UB *)"[TEST] %s lift the car and hold the LEFT wheel still: "
                    "the car is driven %s in %d s\n",
              p_id, b_turn ? "turning left" : "forward",
              STALL_START_DELAY_MS / 1000);
    if ((tk_dly_tsk(STALL_START_DELAY_MS) != E_OK) ||
        (motion_move_update() != MOTION_OK))
    {
        return FALSE;
    }
    motion_status_t status = b_turn ? motion_move_turn_left(90u)
                                    : motion_move_forward(FORWARD_TEST_MM);
    if (status != MOTION_OK)
    {
        return FALSE;
    }

    INT moving_ms = update_while_moving();
    BOOL b_stalled = (MOTION_STATE_STALLED == motion_move_get_state());
    BOOL b_braked = slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                    slice_braked(MOTION_RIGHT_MOTOR_PIN_A);
    status = b_turn ? motion_move_turn_left(90u)
                    : motion_move_forward(FORWARD_TEST_MM);
    BOOL b_refused = (MOTION_ERR_STALLED == status);
    BOOL b_cleared = (motion_move_stop() == MOTION_OK) &&
                     (MOTION_STATE_STOPPED == motion_move_get_state());
    tm_printf((UB *)"[TEST] %s stalled %d after %d ms (expect about %d), "
                    "braked %d, refused %d, cleared %d\n",
              p_id, (INT)b_stalled, moving_ms, (INT)MOTION_STALL_TIMEOUT_MS,
              (INT)b_braked, (INT)b_refused, (INT)b_cleared);
    return b_stalled && b_braked && b_refused && b_cleared;
}

/* Update every period until the move ends or times out; return its ms. */
static INT
update_while_moving (void)
{
    INT elapsed = 0;

    while (motion_move_is_busy() && (elapsed < MOVE_TIMEOUT_MS))
    {
        if ((wait_update_period() != E_OK) ||
            (motion_move_update() != MOTION_OK))
        {
            break;
        }
        elapsed += (INT)MOTION_UPDATE_PERIOD_MS;
    }
    return elapsed;
}
#endif /* MOTION_MOVE_TESTS || MOTION_TURN_TESTS */

#if MOTION_MOVE_TESTS || MOTION_TURN_TESTS || MOTION_SPEED_TESTS ||            \
    MOTION_SYNC_TESTS || MOTION_FOLLOW_TESTS
/* Update every period for @p duration_ms, running any move or turn. */
static void
update_for (INT duration_ms)
{
    for (INT elapsed = 0; elapsed < duration_ms;
         elapsed += (INT)MOTION_UPDATE_PERIOD_MS)
    {
        (void)wait_update_period();
        (void)motion_move_update();
    }
}
#endif /* MOTION_MOVE_TESTS ... MOTION_FOLLOW_TESTS */

#endif /* APP_TEST */
