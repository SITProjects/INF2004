/**
 * @file obstacle.c
 * @brief Obstacle subsystem: starts and runs the scan task.
 *
 * The scan task owns the ultrasonic sensor, the servo's PWM slice and the
 * GPIO bank interrupt (TEAM.md section 3). It runs on core 0 because the
 * echo interrupt is enabled in core 0's interrupt controller.
 */

#include "obstacle.h"
#include "obstacle_calibration.h"
#include "obstacle_scan.h"
#include <stdbool.h>
#if TM_MQTT
#include "../communication/telemetry_queue.h"
#endif

/* Private types, constants, and macros. */
#define TELEMETRY_SAMPLE_PERIOD_MS 1000u

/* obstacle_get_state() gives up after this many torn reads. */
#define STATE_READ_TRIES 4u

/* Compiler and memory barrier: keeps the shared_state copy between the
 * sequence counter accesses, on both cores. */
#define STATE_BARRIER() __asm__ volatile("dmb" ::: "memory")

/* Level bits of obstacle_flg_event. */
#define EVT_LEVELS                                                             \
    (OBSTACLE_EVT_STOP | OBSTACLE_EVT_CRITICAL | OBSTACLE_EVT_FAULT)

/* Static data. */

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
/* Use configured travel and settling values; calibration remains pending. */
static const obstacle_servo_timing_t servo_timing = {
    .travel_ms_per_60_degrees = OBSTACLE_SERVO_TRAVEL_MS_PER_60_DEG,
    .settle_margin_ms = OBSTACLE_SERVO_SETTLE_MARGIN_MS};
#endif

ID obstacle_tid_scan;
ID obstacle_flg_event;

/*
 * State for obstacle_get_state(), written by the scan task only. The
 * sequence is odd while it writes; a reader that sees it odd or changed
 * reads again. shared_state is not volatile, so STATE_BARRIER() stops the
 * compiler moving its copy across the counter accesses.
 */
static volatile uint32_t state_sequence;
static obstacle_shared_state_t shared_state;

/* Private function declarations. */
static void scan_task (INT stacd, void * p_exinf);
#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
static void publish_state (obstacle_shared_state_t const * p_state,
                           bool b_new_result);
#endif

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
static int direction_degrees (robot_obstacle_direction_t direction);
static int edge_degrees (uint8_t index);
static uint32_t log_time_ms (void);

#endif

/* Public function definitions. */

ER
obstacle_init (void)
{
    T_CFLG cflg = {0};
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;
    obstacle_flg_event = tk_cre_flg(&cflg);
    if (obstacle_flg_event < E_OK)
    {
        return obstacle_flg_event;
    }
#if APP_TEST
    if (obstacle_scan_test_init() != 0)
    {
        return E_SYS;
    }
#endif
    /* The scan task sets up its own hardware. */
    return E_OK;
}

ER
obstacle_start (void)
{
    obstacle_tid_scan = app_start_task(scan_task, OBSTACLE_PRI_SCAN, TP_PRC1,
                                       "obstacle_scan_task");
    return (obstacle_tid_scan < E_OK) ? obstacle_tid_scan : E_OK;
}

bool
obstacle_get_state (obstacle_shared_state_t * p_state)
{
    if (p_state == NULL)
    {
        return false;
    }
    for (uint32_t attempt = 0u; attempt < STATE_READ_TRIES; ++attempt)
    {
        uint32_t before = state_sequence;
        if ((before & 1u) == 0u)
        {
            STATE_BARRIER(); /* Copy only after reading the counter. */
            *p_state = shared_state;
            STATE_BARRIER(); /* Finish the copy before checking again. */
            if (state_sequence == before)
            {
                return true;
            }
        }
    }
    return false;
}

/* Private function definitions. */

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
static uint32_t
log_time_ms (void)
{
    SYSTIM tim;
    (void)tk_get_otm(&tim);
    return (uint32_t)tim.lo;
}
#endif

static void
scan_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

#if OBSTACLE_CALIBRATION_MODE != OBSTACLE_CAL_MODE_OFF
    obstacle_calibration_run(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN,
                             OBSTACLE_SERVO_PIN);
#else
    if (obstacle_scan_init(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN,
                           OBSTACLE_SERVO_PIN, &servo_timing) != 0)
    {
        tm_printf(
            (UB *)"[%lu ms]obstacle: failed to initialize obstacle scanning\n",
            log_time_ms());
        tk_ext_tsk();
    }
    tm_printf((UB *)"[%lu ms]obstacle: scanning started\n", log_time_ms());

#if TM_MQTT
    uint32_t last_sample_ms = log_time_ms();
    uint32_t sample_ms = 0u;
    bool b_sample_pending = false;
    UB sample_text[TELEMETRY_TEXT_SIZE];
#endif
    uint8_t last_sequence = 0u;
    bool b_last_stop_requested = obstacle_scan_stop_requested();
    bool b_last_critical = false;
    tm_printf(
        (UB *)"[%lu ms]obstacle: stop request ACTIVE (sensing output only)\n",
        log_time_ms());
    for (;;)
    {
        robot_obstacle_result_t scan = obstacle_scan_update();
        bool b_stop = obstacle_scan_stop_requested();
        bool b_critical = robot_obstacle_get_critical(scan.flags);
        if (b_critical != b_last_critical)
        {
            tm_printf((UB *)"[%lu ms]obstacle: CRITICAL %s\n", log_time_ms(),
                      b_critical ? "set" : "cleared");
            b_last_critical = b_critical;
        }
        if (b_stop != b_last_stop_requested)
        {
            tm_printf((UB *)"[%lu ms]obstacle: stop request %s\n",
                      log_time_ms(), b_stop ? "ACTIVE" : "RELEASED");
            b_last_stop_requested = b_stop;
        }
        robot_obstacle_status_t status = robot_obstacle_get_status(scan.flags);
        robot_obstacle_direction_t direction =
            robot_obstacle_get_direction(scan.flags);
        bool new_result = scan.sequence != last_sequence;
        last_sequence = scan.sequence;
        obstacle_shared_state_t state = {.result = scan,
                                         .geometry = obstacle_scan_geometry(),
                                         .b_stop_requested = b_stop,
                                         .avoid_state = 0u,
                                         .fault_reason = 0u};
        publish_state(&state, new_result);
        if (new_result && (scan.width_mm != ROBOT_OBSTACLE_WIDTH_UNAVAILABLE))
        {
            tm_printf((UB *)"[%lu ms]obstacle: [0x%02X] %s dir %d width %u mm "
                            "hits %d..%d (L %s, R %s)\n",
                      log_time_ms(), (unsigned int)scan.flags,
                      obstacle_scan_status_name(status),
                      direction_degrees(direction), (unsigned int)scan.width_mm,
                      edge_degrees(robot_obstacle_get_left_edge(scan.edges)),
                      edge_degrees(robot_obstacle_get_right_edge(scan.edges)),
                      ((scan.flags & ROBOT_OBSTACLE_LEFT_EDGE_FOUND) != 0u)
                          ? "found"
                          : "not found",
                      ((scan.flags & ROBOT_OBSTACLE_RIGHT_EDGE_FOUND) != 0u)
                          ? "found"
                          : "not found");
        }
        else if (new_result)
        {
            tm_printf((UB *)"[%lu ms]obstacle: [0x%02X] %s dir %d\n",
                      log_time_ms(), (unsigned int)scan.flags,
                      obstacle_scan_status_name(status),
                      direction_degrees(direction));
        }

#if TM_MQTT
        uint32_t now_ms = log_time_ms();
        if (!b_sample_pending &&
            ((uint32_t)(now_ms - last_sample_ms) >= TELEMETRY_SAMPLE_PERIOD_MS))
        {
            last_sample_ms = now_ms;
            sample_ms = now_ms;
            (void)tm_sprintf(
                sample_text,
                (const UB *)"obstacle flags=%u edges=%u width_mm=%u",
                (unsigned int)scan.flags, (unsigned int)scan.edges,
                (unsigned int)scan.width_mm);
            b_sample_pending = true;
        }
        if (b_sample_pending)
        {
            uint32_t sequence = 0u;
            telemetry_enqueue_result_t queued = telemetry_queue_push(
                sample_ms, (const char *)sample_text, &sequence);
            if (queued != TELEMETRY_BUSY)
            {
                b_sample_pending = false;
                if (queued == TELEMETRY_QUEUED)
                {
                    tm_printf(
                        (UB *)"telemetry: queued seq=%lu captured_ms=%lu\n",
                        (unsigned long)sequence, (unsigned long)sample_ms);
                }
                else
                {
                    tm_printf((UB *)"telemetry: DROP seq=%lu reason=%s\n",
                              (unsigned long)sequence,
                              queued == TELEMETRY_FULL ? "full" : "invalid");
                }
            }
        }
#endif
        (void)tk_dly_tsk(OBSTACLE_UPDATE_PERIOD_MS);
    }
#endif
}

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
/* Copy the state for obstacle_get_state(), then update the event flag:
 * level bits only when they change, RESULT on every new sequence. */
static void
publish_state (obstacle_shared_state_t const * p_state, bool b_new_result)
{
    static uint32_t levels;

    state_sequence = state_sequence + 1u; /* Odd: readers retry. */
    STATE_BARRIER();
    shared_state = *p_state;
    STATE_BARRIER();
    state_sequence = state_sequence + 1u; /* Even: whole again. */

    uint32_t now = 0u;
    if (p_state->b_stop_requested)
    {
        now |= OBSTACLE_EVT_STOP;
    }
    if (robot_obstacle_get_critical(p_state->result.flags))
    {
        now |= OBSTACLE_EVT_CRITICAL;
    }
    if (p_state->fault_reason != 0u)
    {
        now |= OBSTACLE_EVT_FAULT;
    }
    uint32_t set = (now & ~levels) & EVT_LEVELS;
    uint32_t cleared = (levels & ~now) & EVT_LEVELS;
    if (b_new_result)
    {
        set |= OBSTACLE_EVT_RESULT;
    }
    if (set != 0u)
    {
        app_check(tk_set_flg(obstacle_flg_event, set), "obstacle event set");
    }
    if (cleared != 0u)
    {
        app_check(tk_clr_flg(obstacle_flg_event, ~cleared),
                  "obstacle event clear");
    }
    levels = now;
}

/* Convert the packed direction code only for logging, without geometry. */
static int
direction_degrees (robot_obstacle_direction_t direction)
{
    switch (direction)
    {
        case ROBOT_OBSTACLE_DIR_LEFT_60:
            return (int)OBSTACLE_SWEEP_FIRST_DEG;
        case ROBOT_OBSTACLE_DIR_CENTRE_90:
            return (int)OBSTACLE_CENTRE_DEG;
        case ROBOT_OBSTACLE_DIR_RIGHT_120:
            return (int)OBSTACLE_SWEEP_LAST_DEG;
        case ROBOT_OBSTACLE_DIR_NONE:
        default:
            return -1;
    }
}

/* Convert a packed edge index to its grid angle for logging, -1 if none. */
static int
edge_degrees (uint8_t index)
{
    return (index == ROBOT_OBSTACLE_EDGE_NONE)
               ? -1
               : (int)(OBSTACLE_SWEEP_FIRST_DEG +
                       (OBSTACLE_SWEEP_STEP_DEG * index));
}

#endif
