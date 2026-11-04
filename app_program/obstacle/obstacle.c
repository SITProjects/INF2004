/**
 * @file obstacle.c
 * @brief Obstacle subsystem: starts and runs the scan task.
 *
 * The scan task owns the ultrasonic sensor, the servo's PWM slice and the
 * GPIO bank interrupt (TEAM.md section 3). It runs on core 0 because the
 * echo interrupt is enabled in core 0's interrupt controller.
 *
 * With OBSTACLE_AVOID_ENABLED (and not APP_TEST), the scan task is also the
 * interim tMission/tSafety for obstacles (IF-06): it holds the motors with
 * MOTION_FLG_INHIBIT from start-up until a clear path is confirmed and on
 * every stop, and drives the box bypass through motion_mbf_cmd. It is the
 * only task that sets or clears INHIBIT, and the only consumer of
 * MOVE_DONE.
 */

#include "obstacle.h"
#include "obstacle_avoid.h"
#include "obstacle_calibration.h"
#include "obstacle_scan.h"
#include <stdbool.h>
#if TM_MQTT
#include "../communication/telemetry_queue.h"
#endif

/* Private types, constants, and macros. */

/* The bypass runs only in normal firmware: TEST=1 lets the motion tests
 * own the motors, and calibration never reaches the scan loop. */
#define AVOID_ACTIVE                                                           \
    (OBSTACLE_AVOID_ENABLED && !APP_TEST &&                                    \
     (OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF))
#if AVOID_ACTIVE
#include "../motion/motion.h"
#include "../motion/motion_config.h"
#endif

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

#if AVOID_ACTIVE
static const obstacle_avoid_config_t avoid_config = {
    .geometry = {.half_width_mm = (uint16_t)(OBSTACLE_CAR_WIDTH_MM / 2u),
                 .margin_mm = OBSTACLE_AVOID_MARGIN_MM,
                 .axle_to_pivot_mm = OBSTACLE_CAR_AXLE_TO_PIVOT_MM,
                 .rear_mm = OBSTACLE_CAR_REAR_MM,
                 .depth_mm = OBSTACLE_AVOID_DEPTH_MM,
                 .sidestep_min_mm = OBSTACLE_AVOID_SIDESTEP_MIN_MM,
                 .sidestep_max_mm = OBSTACLE_AVOID_SIDESTEP_MAX_MM,
                 .lateral_max_mm = OBSTACLE_AVOID_LATERAL_MAX_MM,
                 .pass_max_mm = OBSTACLE_AVOID_PASS_MAX_MM,
                 .return_extra_mm = OBSTACLE_AVOID_RETURN_EXTRA_MM,
                 .overshoot_mm = OBSTACLE_AVOID_LINE_OVERSHOOT_MM},
    .timing = {.stop_ack_ms = OBSTACLE_STOP_ACK_TIMEOUT_MS,
               .result_ms = OBSTACLE_SCAN_TIMEOUT_MS,
               .prepare_ms = OBSTACLE_AVOID_PREPARE_TIMEOUT_MS,
               .send_ms = OBSTACLE_AVOID_SEND_TIMEOUT_MS,
               .verify_ms = OBSTACLE_AVOID_VERIFY_TIMEOUT_MS,
               .block_wait_ms = OBSTACLE_AVOID_BLOCK_WAIT_MS,
               .retry_delay_ms = OBSTACLE_AVOID_RETRY_DELAY_MS,
               .move_speed_mm_s = MOTION_MOVE_SPEED_MM_S,
               .turn_speed_deg_s = MOTION_TURN_SPEED_DEG_S,
               .resume_min_mm = OBSTACLE_AVOID_RESUME_MIN_MM,
               .resume_min_deg = OBSTACLE_AVOID_RESUME_MIN_DEG,
               .max_sidesteps = OBSTACLE_AVOID_MAX_SIDESTEPS},
    .width = {.scale_mm_per_10k = OBSTACLE_SOUND_MM_PER_10K_US,
              .echo_offset_mm = OBSTACLE_ECHO_OFFSET_MM,
              .radial_offset_mm = OBSTACLE_SENSOR_RADIAL_OFFSET_MM,
              .edge_correction_deg = OBSTACLE_EDGE_CORRECTION_DEG}};

/* Scan task only. avoid_in also carries last loop's action results. */
static obstacle_avoid_t avoid;
static obstacle_avoid_input_t avoid_in;
#endif

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
#if AVOID_ACTIVE
static void avoid_loop (robot_obstacle_result_t scan, bool b_stop);
static void apply_actions (obstacle_avoid_action_t const * p_act);
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
#if AVOID_ACTIVE
    /* Every init has run, so motion_flg_ctrl exists; tMotion starts held. */
    ER const held = tk_set_flg(motion_flg_ctrl, MOTION_FLG_INHIBIT);
    if (held < E_OK)
    {
        return held;
    }
    tm_printf((UB *)"obstacle: motors held until the path is confirmed "
                    "clear\n");
#endif
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
#if AVOID_ACTIVE
        tm_printf((UB *)"[%lu ms]obstacle: scanner failed, motors stay held\n",
                  log_time_ms());
#endif
        tk_ext_tsk();
    }
    tm_printf((UB *)"[%lu ms]obstacle: scanning started\n", log_time_ms());
#if AVOID_ACTIVE
    obstacle_avoid_init(&avoid, &avoid_config, log_time_ms());
#endif

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
#if AVOID_ACTIVE
        avoid_loop(scan, b_stop);
#endif
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
#if AVOID_ACTIVE
        state.avoid_state = (uint8_t)avoid.state;
        state.fault_reason = (uint8_t)avoid.fault;
#endif
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

#if AVOID_ACTIVE
/* Read motion events before the status (IF-03 ordering), step, act. */
static void
avoid_loop (robot_obstacle_result_t scan, bool b_stop)
{
    T_RFLG events = {0};
    ER const read = tk_ref_flg(motion_flg_event, &events);
    if (read < E_OK)
    {
        /* Any failed kernel call latches FAULT; report it as one. */
        app_check(read, "obstacle motion event read");
        avoid_in.b_inhibit_failed = true;
        events.flgptn = 0u;
    }
    motion_task_status_t const status = motion_get_status();
    motion_shared_odometry_t odometry = {0};
    avoid_in.b_odometry_valid = motion_get_odometry(&odometry);
    avoid_in.now_ms = log_time_ms();
    avoid_in.scan = scan;
    avoid_in.b_stop_requested = b_stop;
    avoid_in.geometry = obstacle_scan_geometry();
    avoid_in.corridor = obstacle_scan_corridor();
    avoid_in.motion_mode = status.mode;
    avoid_in.motion_done = status.done;
    avoid_in.b_move_done = (events.flgptn & MOTION_EVT_MOVE_DONE) != 0u;
    avoid_in.b_stall = (events.flgptn & MOTION_EVT_STALL) != 0u;
    avoid_in.b_enc_fault = (events.flgptn & MOTION_EVT_ENC_FAULT) != 0u;
    avoid_in.b_rejected = (events.flgptn & MOTION_EVT_REJECTED) != 0u;
    avoid_in.distance_mm = odometry.distance_mm;
    avoid_in.heading_deg = odometry.heading_deg;

    obstacle_avoid_action_t const act = obstacle_avoid_step(&avoid, &avoid_in);
    apply_actions(&act);
}

/* Carry out the actions in their documented order and keep each result
 * in avoid_in for the next step. */
static void
apply_actions (obstacle_avoid_action_t const * p_act)
{
    avoid_in.b_inhibit_failed = false;
    avoid_in.send = OBSTACLE_AVOID_SEND_NONE;
    avoid_in.b_rearmed = false;
    avoid_in.corridor_request = OBSTACLE_AVOID_CORRIDOR_REQ_NONE;
    if (p_act->b_set_inhibit)
    {
        ER const er = tk_set_flg(motion_flg_ctrl, MOTION_FLG_INHIBIT);
        app_check(er, "obstacle inhibit set");
        avoid_in.b_inhibit_failed = er < E_OK;
    }
    if (p_act->b_clear_inhibit)
    {
        ER const er = tk_clr_flg(motion_flg_ctrl, ~MOTION_FLG_INHIBIT);
        app_check(er, "obstacle inhibit clear");
        avoid_in.b_inhibit_failed = avoid_in.b_inhibit_failed || (er < E_OK);
    }
    if (p_act->b_rearm)
    {
        avoid_in.rearm_sequence = obstacle_scan_rearm();
        avoid_in.b_rearmed = true;
    }
    if (p_act->b_corridor)
    {
        avoid_in.corridor_request = obstacle_scan_request_corridor()
                                        ? OBSTACLE_AVOID_CORRIDOR_REQ_STARTED
                                        : OBSTACLE_AVOID_CORRIDOR_REQ_REFUSED;
    }
    if (p_act->b_send)
    {
        /* Only after the clear can the next MOVE_DONE be this command's. */
        ER er = tk_clr_flg(motion_flg_event, ~(UINT)MOTION_EVT_ALL);
        if (er >= E_OK)
        {
            uint32_t cmd = p_act->cmd;
            er = tk_snd_mbf(motion_mbf_cmd, &cmd, (INT)sizeof(cmd), TMO_POL);
        }
        if (er == E_TMOUT)
        {
            avoid_in.send = OBSTACLE_AVOID_SEND_BUSY;
        }
        else
        {
            app_check(er, "obstacle motion command");
            avoid_in.send = (er >= E_OK) ? OBSTACLE_AVOID_SEND_OK
                                         : OBSTACLE_AVOID_SEND_ERROR;
        }
    }
    if (p_act->p_reason != NULL)
    {
        tm_printf((UB *)"[%lu ms]obstacle: avoid %s -> %s (%s)\n",
                  (unsigned long)log_time_ms(),
                  obstacle_avoid_state_name(p_act->from),
                  obstacle_avoid_state_name(p_act->to), p_act->p_reason);
    }
}
#endif
