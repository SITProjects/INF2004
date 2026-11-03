/**
 * @file motion_command.c
 * @brief Feed calibrated IR edge errors to the existing motion controller.
 *
 * A mutex protects a latest-value slot. The owner task services the motion
 * controller every update period, rather than blocking for sensor messages.
 * Dry-run and APP_TEST builds never call motion APIs from this adapter;
 * existing motion tests retain exclusive access to the encoders and motors.
 */

#include "motion_command.h"
#include "motion.h"
#include "motion_config.h"
#include "motion_move.h"

/* Private types, constants, and macros. */
#define ACTUATION_ACTIVE (MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST)
#define ADC_MAX 4095u
#define REPORT_INTERVAL_MS 100u
/* The kernel adds one tick to a positive task delay, as in motion_test.c. */
#if (MOTION_UPDATE_PERIOD_MS <= CNF_TIMER_PERIOD) ||                           \
    ((MOTION_UPDATE_PERIOD_MS % CNF_TIMER_PERIOD) != 0)
#error "Motion update period must be a multiple of the tick, above one tick"
#endif
#define UPDATE_DELAY_MS (MOTION_UPDATE_PERIOD_MS - CNF_TIMER_PERIOD)

_Static_assert(MOTION_LINE_ERROR_MAX <= 1023, "Line error must fit 11 bits");
_Static_assert(LINE_LEFT_EDGE_MIN > 0u &&
                   LINE_LEFT_BLACK_MIN > LINE_LEFT_EDGE_MAX,
               "Invalid left calibration span");
_Static_assert(LINE_RIGHT_EDGE_MIN > 0u &&
                   LINE_RIGHT_BLACK_MIN > LINE_RIGHT_EDGE_MAX,
               "Invalid right calibration span");

typedef struct
{
    uint32_t received_ms;
    signed int error : 11;
    unsigned int command : 3;
    unsigned int b_seen : 1;
    unsigned int b_valid : 1;
    unsigned int b_pending : 1;
} command_request_t;

/* Static data. */
static ID command_mutex;
static ID drive_task_id;
/* Shared only under command_mutex. */
static command_request_t latest_request;
/* The following state belongs exclusively to the motion task. */
static command_request_t active_request;
#if ACTUATION_ACTIVE
static int16_t last_error = MOTION_LINE_ERROR_MAX;
static bool b_fault_latched;
#endif

/* Private function declarations. */
static void drive_task (INT stacd, void * p_exinf);
static void service_commands (void);
static ER take_request (bool * p_new);
static int16_t edge_offset (uint16_t raw, uint16_t lower, uint16_t upper,
                            uint16_t black);
static int16_t estimate_error (ir_sensor_reading_t reading);
#if ACTUATION_ACTIVE
static void update_controller (bool b_new);
static motion_status_t apply_reading (void);
static void latch_fault (motion_status_t status);
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
    if ((command_mutex <= 0) || (drive_task_id > 0))
    {
        return E_OBJ;
    }
    drive_task_id =
        app_start_task(drive_task, MOTION_PRI_DRIVE, 0, "motion_drive_task");
    return (drive_task_id < E_OK) ? drive_task_id : E_OK;
}

ER
motion_command_submit (line_command_t command,
                       ir_sensor_reading_t const * p_reading)
{
    bool const b_command_valid = ((unsigned int)command <= LINE_COMMAND_STOP);
    bool const b_counts_valid =
        (p_reading == NULL) ||
        ((p_reading->left_raw <= ADC_MAX) && (p_reading->right_raw <= ADC_MAX));
    command_request_t request = {.command = LINE_COMMAND_STOP};
    SYSTIM now;

    if (b_command_valid && b_counts_valid && (p_reading != NULL))
    {
        request.command = command;
        request.error = estimate_error(*p_reading);
        request.b_seen = (p_reading->left_raw >= LINE_LEFT_EDGE_MIN) ||
                         (p_reading->right_raw >= LINE_RIGHT_EDGE_MIN);
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

/* Private function definitions. */

static void
drive_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
#if ACTUATION_ACTIVE
    tm_printf((UB *)"motion: line adapter enabled\n");
#else
    tm_printf((UB *)"motion: line adapter dry run; no motor API calls\n");
#endif
    for (;;)
    {
        service_commands();
        (void)tk_dly_tsk(UPDATE_DELAY_MS);
    }
}

static void
service_commands (void)
{
    bool b_new;
    SYSTIM now;
    ER const result = take_request(&b_new);
    /* Contention skips this poll; the cached timestamp still expires. */
    if (((result < E_OK) && (result != E_TMOUT)) || (tk_get_otm(&now) < E_OK) ||
        ((uint32_t)((uint32_t)now.lo - active_request.received_ms) >=
         MOTION_LINE_TIMEOUT_MS))
    {
        active_request.b_valid = 0u;
    }
#if ACTUATION_ACTIVE
    update_controller(b_new);
#else
    report_request();
#endif
}

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

/* Provisional reflectance offset, not a measured distance. Inside the band
 * is zero. Scale toward ADC zero or the measured full-black minimum. */
static int16_t
edge_offset (uint16_t raw, uint16_t lower, uint16_t upper, uint16_t black)
{
    int32_t offset = 0;
    if (raw < lower)
    {
        offset = ((int32_t)raw - lower) * MOTION_LINE_ERROR_MAX / lower;
    }
    else if (raw > upper)
    {
        offset =
            ((int32_t)raw - upper) * MOTION_LINE_ERROR_MAX / (black - upper);
        if (offset > MOTION_LINE_ERROR_MAX)
        {
            offset = MOTION_LINE_ERROR_MAX;
        }
    }
    return (int16_t)offset;
}

static int16_t
estimate_error (ir_sensor_reading_t reading)
{
    int32_t const left = edge_offset(reading.left_raw, LINE_LEFT_EDGE_MIN,
                                     LINE_LEFT_EDGE_MAX, LINE_LEFT_BLACK_MIN);
    int32_t const right =
        edge_offset(reading.right_raw, LINE_RIGHT_EDGE_MIN, LINE_RIGHT_EDGE_MAX,
                    LINE_RIGHT_BLACK_MIN);
    /* More black on the left and/or less on the right requires a left turn. */
    return (int16_t)((left - right) / 2);
}

#if ACTUATION_ACTIVE
static void
update_controller (bool b_new)
{
    motion_state_t const state = motion_move_get_state();
    if (state == MOTION_STATE_STALLED)
    {
        latch_fault(MOTION_ERR_STALLED);
    }
    else if (state == MOTION_STATE_TIMED_OUT)
    {
        latch_fault(MOTION_ERR_TIMED_OUT);
    }
    if (!b_fault_latched)
    {
        motion_status_t status = MOTION_OK;
        if (!active_request.b_valid)
        {
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
    /* Keep odometry and the wheel-speed loop running even without new IR. */
    motion_status_t const status = motion_move_update();
    if (status != MOTION_OK)
    {
        latch_fault(status);
    }
}

static motion_status_t
apply_reading (void)
{
    int16_t error = (int16_t)active_request.error;
    if (active_request.b_seen)
    {
        if (error != 0)
        {
            last_error = error;
        }
    }
    else
    {
        /* The API requires a last-side hint for lost line, never zero.
         * With no prior direction, choose positive; SEARCH still brakes. */
        error =
            (last_error < 0) ? -MOTION_LINE_ERROR_MAX : MOTION_LINE_ERROR_MAX;
    }
    motion_status_t status =
        motion_move_set_line(error, active_request.b_seen != 0u);
    if (status != MOTION_OK)
    {
        return status;
    }
    bool const b_follow =
        active_request.b_seen &&
        ((active_request.command == LINE_COMMAND_FORWARD) ||
         (active_request.command == LINE_COMMAND_STEER_LEFT) ||
         (active_request.command == LINE_COMMAND_STEER_RIGHT));
    if (!b_follow)
    {
        return motion_move_is_busy() ? motion_move_stop() : MOTION_OK;
    }
    /* Start only on entry; restarting every sample would reset the PD loop. */
    if (motion_move_get_state() != MOTION_STATE_FOLLOW)
    {
        status = motion_move_follow(MOTION_FOLLOW_SPEED_MM_S);
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
        tm_printf((UB *)"motion: controller fault %d; restart required\n",
                  (INT)status);
    }
}
#else
static void
report_request (void)
{
    static uint32_t sample;
    if ((sample++ % (REPORT_INTERVAL_MS / MOTION_UPDATE_PERIOD_MS)) == 0u)
    {
        line_command_t const command =
            active_request.b_valid ? (line_command_t)active_request.command
                                   : LINE_COMMAND_STOP;
        tm_printf((UB *)"motion: %s edge_error=%d seen=%u valid=%u (dry run)\n",
                  line_command_name(command), (INT)active_request.error,
                  (UINT)active_request.b_seen, (UINT)active_request.b_valid);
    }
}
#endif /* ACTUATION_ACTIVE */
