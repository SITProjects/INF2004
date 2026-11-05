/**
 * @file motion_command.c
 * @brief Feed calibrated IR edge errors to the existing motion controller.
 *
 * A mutex protects a latest-value slot. tMotion services this adapter once
 * per update before its single motion_move_update() call. This module creates
 * no task. Dry-run and APP_TEST builds never call motion APIs from it.
 */

#include "motion_command.h"
#include "motion.h"
#include "motion_command_config.h"
#include "motion_move.h"

/* Private types, constants, and macros. */
#define ACTUATION_ACTIVE (MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST)
#define ADC_MAX 4095u
_Static_assert(MOTION_LINE_ERROR_MAX <= 1023, "Line error must fit 11 bits");
_Static_assert(LINE_COMMAND_ALIGN_RIGHT <= 15,
               "Line command must fit four bits");
_Static_assert(MOTION_COMMAND_PROBE_ERROR > 0 &&
                   MOTION_COMMAND_PROBE_ERROR <= MOTION_COMMAND_EDGE_ERROR &&
                   MOTION_COMMAND_SEARCH_ERROR > 0 &&
                   MOTION_COMMAND_SEARCH_ERROR <= MOTION_COMMAND_STEER_ERROR,
               "Invalid probe/search correction");
_Static_assert(MOTION_COMMAND_STEER_ERROR > 1 &&
                   MOTION_COMMAND_STEER_ERROR <= MOTION_LINE_ERROR_MAX,
               "Invalid steering magnitude");
_Static_assert(MOTION_COMMAND_EDGE_ERROR > 0 &&
                   MOTION_COMMAND_EDGE_ERROR <= MOTION_COMMAND_STEER_ERROR,
               "Invalid gentle edge correction");
_Static_assert(LINE_LEFT_WHITE_MAX < LINE_LEFT_DETECT_MIN &&
                   LINE_LEFT_DETECT_MIN < LINE_LEFT_BLACK_MIN,
               "Invalid left detection/calibration range");
_Static_assert(LINE_RIGHT_WHITE_MAX < LINE_RIGHT_DETECT_MIN &&
                   LINE_RIGHT_DETECT_MIN < LINE_RIGHT_BLACK_MIN,
               "Invalid right detection/calibration range");
_Static_assert(MOTION_COMMAND_FOLLOW_SPEED_MM_S > 0u &&
                   MOTION_COMMAND_FOLLOW_SPEED_MM_S <= MOTION_MAX_SPEED_MM_S,
               "Invalid follow speed");
_Static_assert(MOTION_COMMAND_SEARCH_SPEED_MM_S > 0u &&
                   MOTION_COMMAND_SEARCH_SPEED_MM_S <=
                       MOTION_COMMAND_FOLLOW_SPEED_MM_S,
               "Invalid search speed");

typedef struct
{
    uint32_t received_ms;
    signed int error : 11;
    unsigned int command : 4;
    unsigned int b_seen : 1;
    unsigned int b_valid : 1;
    unsigned int b_pending : 1;
#if IR_MOTOR_TEST
    ir_sensor_reading_t reading;
#endif
} command_request_t;

#if ACTUATION_ACTIVE && IR_MOTOR_TEST
#define TEST_TRACE_SAMPLES 32u

typedef struct
{
    uint32_t time_ms;
    ir_sensor_reading_t reading;
    int16_t error;
    int16_t turn_deg_s;
    line_command_t command;
    motion_state_t state;
    bool b_valid;
} test_trace_t;
#endif

/* Static data. */
static ID command_mutex;
static bool b_started;
#if ACTUATION_ACTIVE
static uint32_t startup_ms;
static bool b_control_previous;
#endif
/* Shared only under command_mutex. */
static command_request_t latest_request;
/* The following state belongs exclusively to the motion task. */
static command_request_t active_request;
#if ACTUATION_ACTIVE
static bool b_fault_latched;
/* A stall or timed-out turn holds the car for FAULT_RETRY_UPDATES (500 ms),
 * then is cleared so following and searching carry on. After
 * FAULT_RETRY_MAX retries in a row it latches for good, so a wheel that
 * stays blocked is not driven into the stall forever; FAULT_RETRY_RESET
 * updates (3 s) without a fault start the count again. */
#define FAULT_RETRY_UPDATES (500u / MOTION_UPDATE_PERIOD_MS)
#define FAULT_RETRY_MAX 3u
#define FAULT_RETRY_RESET (3000u / MOTION_UPDATE_PERIOD_MS)
static uint32_t fault_updates;   /* Updates in the current fault. */
static uint32_t fault_retries;   /* Retries since the last quiet spell. */
static uint32_t healthy_updates; /* Updates without a fault since. */
static bool b_slow_active;
static bool b_search_right;
static bool b_recovery_turn;
static bool b_turn_latched;
#endif

/* Private function declarations. */
static ER take_request (bool * p_new);
static int16_t estimate_error (line_command_t command,
                               ir_sensor_reading_t reading);
static int16_t edge_error (uint16_t raw, uint16_t detect, uint16_t black);
#if ACTUATION_ACTIVE
static void update_controller (bool b_new);
static motion_status_t apply_reading (void);
static void latch_fault (motion_status_t status);
#if IR_MOTOR_TEST
static void motor_test_trace (bool b_new);
#endif
#else
static void report_request (void);
#endif

/* Public function definitions. */

ER
motion_command_init (void)
{
    T_CMTX const config = {.mtxatr = TA_INHERIT};
    if (command_mutex > 0)
    {
        return E_OBJ;
    }
    command_mutex = tk_cre_mtx(&config);
    return (command_mutex < E_OK) ? command_mutex : E_OK;
}

ER
motion_command_start (void)
{
    if ((command_mutex <= 0) || b_started)
    {
        return E_OBJ;
    }
#if ACTUATION_ACTIVE
    SYSTIM now;
    ER const result = tk_get_otm(&now);
    if (result < E_OK)
    {
        return result;
    }
    startup_ms = (uint32_t)now.lo;
    tm_printf((UB *)"motion: tMotion line adapter at %u mm/s; "
                    "%u s startup delay, no run cutoff\n",
              (UINT)MOTION_COMMAND_FOLLOW_SPEED_MM_S,
              (UINT)(MOTION_COMMAND_START_DELAY_MS / 1000u));
#else
    tm_printf((UB *)"motion: line adapter dry run; no motor API calls\n");
#endif
    b_started = true;
    return E_OK;
}

ER
motion_command_submit (line_command_t command,
                       ir_sensor_reading_t const * p_reading)
{
    bool const b_command_valid =
        ((unsigned int)command <= LINE_COMMAND_ALIGN_RIGHT);
    bool const b_counts_valid =
        (p_reading == NULL) ||
        ((p_reading->left_raw <= ADC_MAX) && (p_reading->right_raw <= ADC_MAX));
    command_request_t request = {.command = LINE_COMMAND_STOP};
    SYSTIM now;

#if IR_MOTOR_TEST
    if (p_reading != NULL)
    {
        request.reading = *p_reading;
    }
#endif
    if (b_command_valid && b_counts_valid && (p_reading != NULL))
    {
        request.command = command;
        request.error = estimate_error(command, *p_reading);
        request.b_seen = (p_reading->left_raw >= LINE_LEFT_DETECT_MIN) ||
                         (p_reading->right_raw >= LINE_RIGHT_DETECT_MIN);
        request.b_valid = 1u;
    }
    if (command_mutex <= 0)
    {
        return E_OBJ;
    }
    ER result = tk_loc_mtx(command_mutex, TMO_POL);
    if (result < E_OK)
    {
        return result;
    }
    result = tk_get_otm(&now);
    if (result == E_OK)
    {
        request.received_ms = (uint32_t)now.lo;
        request.b_pending = 1u;
        latest_request = request;
    }
    (void)tk_unl_mtx(command_mutex);
    return (result < E_OK)
               ? result
               : ((b_command_valid && b_counts_valid) ? E_OK : E_PAR);
}

/* Public owner hook: tMotion remains the sole caller of motion APIs. */
void
motion_command_service (bool b_control_enabled)
{
    bool b_new;
    SYSTIM now;
    ER const result = take_request(&b_new);
    bool const b_clock = (tk_get_otm(&now) == E_OK);
    /* Contention skips this poll; the cached timestamp still expires. */
    if (((result < E_OK) && (result != E_TMOUT)) || !b_clock ||
        ((uint32_t)((uint32_t)now.lo - active_request.received_ms) >=
         MOTION_LINE_TIMEOUT_MS))
    {
        active_request.b_valid = 0u;
    }
#if ACTUATION_ACTIVE
    bool const b_take_over = b_control_enabled && !b_control_previous;
    b_control_previous = b_control_enabled;
    if (!b_control_enabled)
    {
        b_recovery_turn = false;
        b_turn_latched = false;
        b_slow_active = false;
        /* External MOVE/TURN stop-at-line still receives current visibility,
         * but no line decision may replace the queued command or inhibit. */
        if (b_new && active_request.b_valid)
        {
            (void)motion_move_set_line((int16_t)active_request.error,
                                       active_request.b_seen != 0u);
        }
        return;
    }
    if (!b_started || (b_clock && ((uint32_t)((uint32_t)now.lo - startup_ms) <
                                   MOTION_COMMAND_START_DELAY_MS)))
    {
        return;
    }
    if (b_take_over)
    {
        /* An explicit FOLLOW is accepted only after tMotion clears a motor
         * fault with STOP/INHIBIT. No sensor sample can clear this latch. */
        b_fault_latched = false;
    }
    update_controller(b_new || b_take_over);
#else
    (void)b_control_enabled;
    report_request();
#endif
}

/* Private function definitions. */

static ER
take_request (bool * p_new)
{
    *p_new = false;
    ER const result = tk_loc_mtx(command_mutex, TMO_POL);
    if (result < E_OK)
    {
        return result;
    }
    *p_new = (latest_request.b_pending != 0u);
    if (*p_new)
    {
        active_request = latest_request;
        latest_request.b_pending = 0u;
    }
    (void)tk_unl_mtx(command_mutex);
    return E_OK;
}

/* Single-edge coverage supplies calibrated correction. During alignment,
 * ambiguous pairs keep the remembered gentle bias. D may counter-steer. */
static int16_t
estimate_error (line_command_t command, ir_sensor_reading_t reading)
{
    bool const b_left_line = (reading.left_raw >= LINE_LEFT_DETECT_MIN);
    bool const b_right_line = (reading.right_raw >= LINE_RIGHT_DETECT_MIN);
    if (!b_left_line && !b_right_line)
    {
        if (command == LINE_COMMAND_PROBE_LEFT)
        {
            return MOTION_COMMAND_PROBE_ERROR;
        }
        if (command == LINE_COMMAND_PROBE_RIGHT)
        {
            return -MOTION_COMMAND_PROBE_ERROR;
        }
    }
    if (((command == LINE_COMMAND_STEER_LEFT) ||
         (command == LINE_COMMAND_ALIGN_LEFT)) &&
        b_left_line && !b_right_line)
    {
        return edge_error(reading.left_raw, LINE_LEFT_DETECT_MIN,
                          LINE_LEFT_BLACK_MIN);
    }
    if (((command == LINE_COMMAND_STEER_RIGHT) ||
         (command == LINE_COMMAND_ALIGN_RIGHT)) &&
        !b_left_line && b_right_line)
    {
        return -edge_error(reading.right_raw, LINE_RIGHT_DETECT_MIN,
                           LINE_RIGHT_BLACK_MIN);
    }
    if (b_left_line == b_right_line)
    {
        if (command == LINE_COMMAND_ALIGN_LEFT)
        {
            return MOTION_COMMAND_PROBE_ERROR;
        }
        if (command == LINE_COMMAND_ALIGN_RIGHT)
        {
            return -MOTION_COMMAND_PROBE_ERROR;
        }
    }
    return 0;
}

/* Start gently at detection, then ramp with calibrated line coverage.
 * Both callers guarantee raw >= detect and calibration gives black > detect. */
static int16_t
edge_error (uint16_t raw, uint16_t detect, uint16_t black)
{
    uint32_t distance = (uint32_t)raw - detect;
    uint32_t const span = (uint32_t)black - detect;
    if (distance > span)
    {
        distance = span;
    }
    uint32_t const range =
        MOTION_COMMAND_STEER_ERROR - MOTION_COMMAND_EDGE_ERROR;
    return (int16_t)(MOTION_COMMAND_EDGE_ERROR +
                     (distance * range + span / 2u) / span);
}

#if ACTUATION_ACTIVE
static void
update_controller (bool b_new)
{
    motion_state_t const state = motion_move_get_state();
    if ((state == MOTION_STATE_STALLED) || (state == MOTION_STATE_TIMED_OUT))
    {
        motion_status_t const fault = (state == MOTION_STATE_STALLED)
                                          ? MOTION_ERR_STALLED
                                          : MOTION_ERR_TIMED_OUT;
        healthy_updates = 0u;
        if (fault_retries >= FAULT_RETRY_MAX)
        {
            latch_fault(fault); /* For good: STOP then FOLLOW, or restart. */
        }
        else if (fault_updates == 0u)
        {
            ++fault_updates;
            b_fault_latched = true; /* Held for the retry delay. */
            tm_printf((UB *)"motion: fault %d, retry %u of %u in 500 ms\n",
                      (INT)fault, (unsigned int)(fault_retries + 1u),
                      (unsigned int)FAULT_RETRY_MAX);
        }
        else if (++fault_updates >= FAULT_RETRY_UPDATES)
        {
            fault_updates = 0u;
            ++fault_retries;
            (void)motion_move_stop();
            b_fault_latched = false;
            tm_printf((UB *)"motion: fault cleared, searching again\n");
        }
        else
        {
            /* Waiting out the retry delay. */
        }
    }
    else
    {
        fault_updates = 0u; /* The next fault waits the full delay. */
        if ((fault_retries > 0u) && (++healthy_updates >= FAULT_RETRY_RESET))
        {
            fault_retries = 0u;
            healthy_updates = 0u;
        }
    }
    if (!b_fault_latched)
    {
        motion_status_t status = MOTION_OK;
        if (!active_request.b_valid)
        {
            b_recovery_turn = false;
            if (motion_move_is_busy())
            {
                status = motion_move_stop();
            }
        }
        else if (b_new)
        {
            status = apply_reading();
        }
        if (status != MOTION_OK)
        {
            latch_fault(status);
        }
    }
#if IR_MOTOR_TEST
    motor_test_trace(b_new);
#endif
}

static motion_status_t
apply_reading (void)
{
    line_command_t const command = (line_command_t)active_request.command;
    bool const b_turn = (command == LINE_COMMAND_TURN_LEFT) ||
                        (command == LINE_COMMAND_TURN_RIGHT);
    if (command == LINE_COMMAND_STOP)
    {
        b_recovery_turn = false;
        return motion_move_is_busy() ? motion_move_stop() : MOTION_OK;
    }
    /* A fresh sensor stream is required throughout the encoder turn. Side
     * corrections observed while pivoting must not replace its target. */
    if (b_recovery_turn)
    {
        if (motion_move_get_state() == MOTION_STATE_TURN)
        {
            return motion_move_set_line(0, active_request.b_seen != 0u);
        }
        b_recovery_turn = false;
    }
    if (b_turn && !b_turn_latched)
    {
        b_search_right = (command == LINE_COMMAND_TURN_RIGHT);
        motion_status_t const status = b_search_right
                                           ? motion_move_turn_right(90u)
                                           : motion_move_turn_left(90u);
        if (status == MOTION_OK)
        {
            b_recovery_turn = true;
            b_turn_latched = true;
            tm_printf((UB *)"motion: search reacquisition TURN_%s_90\n",
                      b_search_right ? "RIGHT" : "LEFT");
        }
        return status;
    }
    if (!b_turn)
    {
        b_turn_latched = false;
    }
    if ((command == LINE_COMMAND_STEER_LEFT) ||
        (command == LINE_COMMAND_PROBE_LEFT) ||
        (command == LINE_COMMAND_ALIGN_LEFT))
    {
        b_search_right = false;
    }
    else if ((command == LINE_COMMAND_STEER_RIGHT) ||
             (command == LINE_COMMAND_PROBE_RIGHT) ||
             (command == LINE_COMMAND_ALIGN_RIGHT))
    {
        b_search_right = true;
    }
    /* A held turn request after completion resumes search, not another pivot. */
    bool const b_search = (command == LINE_COMMAND_SEARCH) || b_turn;
    bool const b_slow = b_search || (command == LINE_COMMAND_ALIGN_LEFT) ||
                        (command == LINE_COMMAND_ALIGN_RIGHT);
    int16_t const error = b_search
                              ? (b_search_right ? -MOTION_COMMAND_SEARCH_ERROR
                                                : MOTION_COMMAND_SEARCH_ERROR)
                              : (int16_t)active_request.error;
    /* The trace records the actual owner-selected search input as well. */
    active_request.error = error;
    motion_status_t status =
        motion_move_set_line(error, active_request.b_seen != 0u);
    if (status != MOTION_OK)
    {
        return status;
    }
    /* Restart only on entry or a tracking/search speed change, never on
     * each fresh sample. The existing controller owns all wheel commands. */
    if ((motion_move_get_state() != MOTION_STATE_FOLLOW) ||
        (b_slow != b_slow_active))
    {
        uint16_t const speed = b_slow ? MOTION_COMMAND_SEARCH_SPEED_MM_S
                                      : MOTION_COMMAND_FOLLOW_SPEED_MM_S;
        status = motion_move_follow(speed);
        if (status == MOTION_OK)
        {
            b_slow_active = b_slow;
        }
    }
    return status;
}

static void
latch_fault (motion_status_t status)
{
    if (!b_fault_latched)
    {
        b_fault_latched = true;
        motion_state_t const state = motion_move_get_state();
        if ((state != MOTION_STATE_STALLED) &&
            (state != MOTION_STATE_TIMED_OUT))
        {
            (void)motion_move_stop();
        }
        tm_printf((UB *)"motion: controller fault %d; STOP then FOLLOW or "
                        "restart required\n",
                  (INT)status);
    }
}
#if IR_MOTOR_TEST
/* Retain fresh inputs without printing during following or recovery turns.
 * Dump once on the first actual stop, including faults or turn completion. */
static void
motor_test_trace (bool b_new)
{
    static test_trace_t trace[TEST_TRACE_SAMPLES];
    static uint32_t next;
    static uint32_t count;
    static bool b_started;
    static bool b_dumped;
    if (b_dumped)
    {
        return;
    }
    motion_state_t const state = motion_move_get_state();
    if (!b_started)
    {
        if (state != MOTION_STATE_FOLLOW)
        {
            return;
        }
        b_started = true;
    }
    bool const b_moving =
        (state == MOTION_STATE_FOLLOW) || (state == MOTION_STATE_TURN);
    if (b_new || !b_moving)
    {
        SYSTIM now;
        motion_follow_t follow;
        if ((tk_get_otm(&now) == E_OK) &&
            (motion_move_get_follow(&follow) == MOTION_OK))
        {
            trace[next] = (test_trace_t){
                .time_ms = (uint32_t)now.lo,
                .reading = active_request.reading,
                .error = (int16_t)active_request.error,
                .turn_deg_s = follow.turn_deg_s,
                .command = (line_command_t)active_request.command,
                .state = state,
                .b_valid = (active_request.b_valid != 0u)};
            next = (next + 1u) % TEST_TRACE_SAMPLES;
            if (count < TEST_TRACE_SAMPLES)
            {
                ++count;
            }
        }
    }
    if (b_moving)
    {
        return;
    }
    b_dumped = true;
    tm_printf((UB *)"motion: TRACE after first stop; oldest first; "
                    "turn +left/-right, mode 5=follow/3=turn/0=stopped\n");
    for (uint32_t i = 0u; i < count; ++i)
    {
        uint32_t const index =
            (next + TEST_TRACE_SAMPLES - count + i) % TEST_TRACE_SAMPLES;
        test_trace_t const * p_trace = &trace[index];
        tm_printf((UB *)"t=%u L=%u R=%u cmd=%s err=%d turn=%d "
                        "mode=%u valid=%u\n",
                  (UINT)p_trace->time_ms, (UINT)p_trace->reading.left_raw,
                  (UINT)p_trace->reading.right_raw,
                  line_command_name(p_trace->command), (INT)p_trace->error,
                  (INT)p_trace->turn_deg_s, (UINT)p_trace->state,
                  (UINT)p_trace->b_valid);
    }
    tm_printf((UB *)"motion: TRACE end\n");
}

#endif

#else
static void
report_request (void)
{
    static line_command_t previous = (line_command_t)-1;
    static bool b_previous_valid;
    line_command_t const command = active_request.b_valid
                                       ? (line_command_t)active_request.command
                                       : LINE_COMMAND_STOP;
    /* Raw changes are already reported by the line task. Log transitions
     * only, so the dry-run adapter does not flood the calibration console. */
    if ((command != previous) ||
        ((active_request.b_valid != 0u) != b_previous_valid))
    {
        tm_printf((UB *)"motion: %s edge_error=%d seen=%u valid=%u (dry run)\n",
                  line_command_name(command), (INT)active_request.error,
                  (UINT)active_request.b_seen, (UINT)active_request.b_valid);
        previous = command;
        b_previous_valid = (active_request.b_valid != 0u);
    }
}
#endif /* ACTUATION_ACTIVE */
