/**
 * @file motion_command.c
 * @brief Map latest line commands to the existing open-loop motor API.
 *
 * The motion task owns PWM slices 4 and 5. A mutex protects the latest
 * request; an event flag wakes the task without building a command backlog.
 * Actuation is disabled by default and always disabled in APP_TEST builds.
 */

#include "motion_command.h"
#include "motion.h"
#include "motion_drive.h"

/* Private types, constants, and macros. */
#define COMMAND_PENDING 1u
/* Provisional mapping values; tune on the actual robot. */
#define FORWARD_MM_S 80
#define STEER_MM_S 60
#define TURN_DEG_S 30
#define COMMAND_TIMEOUT_MS 250
#define LOCK_TIMEOUT_MS 10

typedef struct
{
    int16_t linear_mm_s;
    int16_t angular_deg_s;
} velocity_request_t;

typedef struct
{
    uint32_t received_ms;
    unsigned int command : 3;
} command_request_t;

/* Static data. */
static ID command_mutex;
static ID command_event;
static ID drive_task_id;
static command_request_t latest_request;

/* Private function declarations. */
static void drive_task (INT stacd, void * p_exinf);
static bool receive_command (line_command_t * p_command,
                             uint32_t * p_received_ms, bool b_active);
static velocity_request_t map_command (line_command_t command);
static void apply_command (line_command_t command);

/* Public function definitions. */

ER
motion_command_init (void)
{
    T_CMTX const mutex_config = {.mtxatr = TA_INHERIT};
    T_CFLG const event_config = {0};

    if ((command_mutex > 0) || (command_event > 0))
    {
        return E_OBJ;
    }
    command_mutex = tk_cre_mtx(&mutex_config);
    if (command_mutex < E_OK)
    {
        return command_mutex;
    }
    command_event = tk_cre_flg(&event_config);
    if (command_event < E_OK)
    {
        (void)tk_del_mtx(command_mutex);
        command_mutex = 0;
        return command_event;
    }
    return E_OK;
}

ER
motion_command_start (void)
{
    if ((command_mutex <= 0) || (command_event <= 0))
    {
        return E_OBJ;
    }
    if (drive_task_id > 0)
    {
        return E_OBJ;
    }
    drive_task_id =
        app_start_task(drive_task, MOTION_PRI_DRIVE, 0, "motion_drive_task");
    return (drive_task_id < E_OK) ? drive_task_id : E_OK;
}

ER
motion_command_submit (line_command_t command)
{
    bool const b_valid = ((unsigned int)command <= LINE_COMMAND_STOP);
    SYSTIM now;

    if ((command_mutex <= 0) || (command_event <= 0))
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
        latest_request.received_ms = (uint32_t)now.lo;
        latest_request.command = b_valid ? command : LINE_COMMAND_STOP;
        result = tk_set_flg(command_event, COMMAND_PENDING);
    }
    (void)tk_unl_mtx(command_mutex);
    return (result < E_OK) ? result : (b_valid ? E_OK : E_PAR);
}

/* Private function definitions. */

static void
drive_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

#if MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST
    tm_printf((UB *)"motion: line-command actuation enabled\n");
#else
    tm_printf((UB *)"motion: dry run; motors remain braked\n");
#endif
    uint32_t received_ms = 0u;
    bool b_active = false;
    for (;;)
    {
        line_command_t command = LINE_COMMAND_STOP;
        b_active = receive_command(&command, &received_ms, b_active);
        apply_command(b_active ? command : LINE_COMMAND_STOP);
        if (!b_active)
        {
            /* Stop before yielding, including after a kernel error. */
            (void)tk_dly_tsk(1);
        }
    }
}

static bool
receive_command (line_command_t * p_command, uint32_t * p_received_ms,
                 bool b_active)
{
    UINT flags;
    SYSTIM now;
    TMO wait_ms = COMMAND_TIMEOUT_MS;

    if (tk_get_otm(&now) < E_OK)
    {
        return false;
    }
    if (b_active)
    {
        uint32_t const elapsed = (uint32_t)now.lo - *p_received_ms;
        wait_ms = (elapsed >= COMMAND_TIMEOUT_MS)
                      ? TMO_POL
                      : (TMO)(COMMAND_TIMEOUT_MS - elapsed);
    }
    if (tk_wai_flg(command_event, COMMAND_PENDING, TWF_ORW | TWF_CLR, &flags,
                   wait_ms) < E_OK)
    {
        return false;
    }
    if (tk_loc_mtx(command_mutex, LOCK_TIMEOUT_MS) < E_OK)
    {
        return false;
    }
    command_request_t const request = latest_request;
    (void)tk_unl_mtx(command_mutex);

    if ((tk_get_otm(&now) < E_OK) ||
        ((uint32_t)((uint32_t)now.lo - request.received_ms) >=
         COMMAND_TIMEOUT_MS))
    {
        return false;
    }
    *p_received_ms = request.received_ms;
    *p_command = (line_command_t)request.command;
    return true;
}

static velocity_request_t
map_command (line_command_t command)
{
    velocity_request_t request = {0};
    switch (command)
    {
        case LINE_COMMAND_FORWARD:
            request.linear_mm_s = FORWARD_MM_S;
            break;

        case LINE_COMMAND_STEER_LEFT:
        case LINE_COMMAND_STEER_RIGHT:
            request.linear_mm_s = STEER_MM_S;
            request.angular_deg_s =
                (command == LINE_COMMAND_STEER_LEFT) ? TURN_DEG_S : -TURN_DEG_S;
            break;

        default:
            /* SEARCH, JUNCTION, STOP and unknown values all brake. */
            break;
    }
    return request;
}

static void
apply_command (line_command_t command)
{
#if !MOTION_COMMAND_ACTUATION_ENABLED || APP_TEST
    static line_command_t previous = (line_command_t)-1;
#endif
    velocity_request_t const request = map_command(command);
    motion_status_t status;

#if MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST
    if ((request.linear_mm_s != 0) || (request.angular_deg_s != 0))
    {
        status = motion_drive_set_velocity(request.linear_mm_s,
                                           request.angular_deg_s);
    }
    else
#endif
    {
        status = motion_drive_stop();
    }
    if (status != MOTION_OK)
    {
        (void)motion_drive_stop();
        tm_printf((UB *)"motion: drive request failed (%d)\n", (INT)status);
        return;
    }
#if !MOTION_COMMAND_ACTUATION_ENABLED || APP_TEST
    if (command != previous)
    {
        tm_printf((UB *)"motion: request linear=%d mm/s angular=%d deg/s\n",
                  (INT)request.linear_mm_s, (INT)request.angular_deg_s);
        previous = command;
    }
#endif
}
