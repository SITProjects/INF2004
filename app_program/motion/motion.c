/**
 * @file motion.c
 * @brief Motion subsystem: creates FLG_MOTION, CYC_MOTION, MBF_MOTION_CMD
 * and the motion event flag, and runs the motion task (tMotion).
 *
 * The motion subsystem owns PWM slices 4 and 5 (GP8-GP11) and PIO0
 * (encoders: left on GP13/GP14, right on GP2/GP3; TEAM.md section 3).
 * motion_start() configures them and then hands them to tMotion, the only
 * task that calls motion_drive_ or motion_move_ functions afterwards (test
 * builds may borrow them with motion_task_hold()).
 *
 * Each CYC_MOTION tick tMotion: discards commands if inhibited, otherwise
 * runs every queued command in order; passes in a new line reading; calls
 * motion_move_update(); and reports MOVE_DONE, STALL and the other
 * MOTION_EVT_ bits when the update changes the state. MOTION_FLG_INHIBIT
 * wakes it at once to brake. The line-command adapter is serviced by this
 * same owner; it never creates another motor task. Queued STOP/MOVE/TURN
 * and INHIBIT suspend automatic following until a queued FOLLOW resumes it.
 */

#include "motion.h"
#include "motion_config.h"
#include "motion_drive.h"
#include "motion_move.h"
#include "motion_command.h"
#include "motion_command_config.h"

/* Private types, constants, and macros. */

/* Packed status word of motion_get_status(). */
#define STATUS_MODE_MASK 0x07u
#define STATUS_DONE_SHIFT 3u
#define STATUS_DONE_MASK 0x03u

/* Packed line word of motion_set_line_input(). */
#define LINE_ERROR_MASK 0x0000FFFFu
#define LINE_SEEN 0x00010000u
#define LINE_SEQUENCE_SHIFT 17u
#define LINE_SEQUENCE_MASK 0x7FFFu
#define LINE_ERROR_LIMIT 1000

/* Each message takes the kernel's INT header plus the 4-byte word. */
#define CMD_SIZE ((INT)sizeof(uint32_t))
#define CMD_BUFFER_SIZE                                                        \
    ((SZ)(MOTION_CMD_QUEUE_DEPTH * (sizeof(INT) + sizeof(uint32_t))))

#define TIMER_NONE 0xFFFFFFFFu

/* motion_get_odometry() gives up after this many torn reads. */
#define ODOMETRY_READ_TRIES 4u

#if APP_TEST
/* motion_task_hold() waits this many kernel ticks for tMotion. */
#define HOLD_WAIT_TICKS 10
#endif

/* The modes reported are the module's states, plus INHIBITED. */
_Static_assert((INT)MOTION_STATE_STOPPED == (INT)ROBOT_MOTION_MODE_STOPPED,
               "mode STOPPED");
_Static_assert((INT)MOTION_STATE_MOVE == (INT)ROBOT_MOTION_MODE_MOVE,
               "mode MOVE");
_Static_assert((INT)MOTION_STATE_STALLED == (INT)ROBOT_MOTION_MODE_STALLED,
               "mode STALLED");
_Static_assert((INT)MOTION_STATE_TURN == (INT)ROBOT_MOTION_MODE_TURN,
               "mode TURN");
_Static_assert((INT)MOTION_STATE_TIMED_OUT == (INT)ROBOT_MOTION_MODE_TIMED_OUT,
               "mode TIMED_OUT");
_Static_assert((INT)MOTION_STATE_FOLLOW == (INT)ROBOT_MOTION_MODE_FOLLOW,
               "mode FOLLOW");
_Static_assert((INT)MOTION_DONE_TARGET == (INT)ROBOT_MOTION_DONE_DIST,
               "done DIST");
_Static_assert((INT)MOTION_DONE_LINE == (INT)ROBOT_MOTION_DONE_LINE,
               "done LINE");
_Static_assert(MOTION_LINE_ERROR_MAX == LINE_ERROR_LIMIT, "line error range");

/* Static data. */

ID motion_flg_ctrl;
ID motion_mbf_cmd;
ID motion_flg_event;

static ID cyc_motion;
/* Queued commands/inhibit take control until an explicit FOLLOW request. */
static bool b_line_control = MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST;

/* Written by tMotion only; read by any task as one word. */
static volatile uint32_t status_word;

/*
 * TODO(step 9, line_follow): interim line input until line_state_t under
 * MTX_LINE exists. Written by one task only, read by tMotion.
 */
static volatile uint32_t line_word;

/*
 * Odometry for motion_get_odometry(), written by tMotion only. The
 * sequence is odd while tMotion writes; a reader that sees it change (or
 * odd) reads again. Both cores' accesses to SRAM are uncached and in
 * order, so volatile access order is the order the other core sees.
 */
static volatile uint32_t odometry_sequence;
static volatile motion_shared_odometry_t shared_odometry;

/* Written by tMotion; read by the test task only while held. */
static motion_task_timing_t timing;
static uint32_t last_wake_us;

#if APP_TEST
static volatile bool b_hold_request;
static volatile bool b_hold_ack;
#endif

/* Private function declarations. */
static void motion_task (INT stacd, void * p_exinf);
static void tick_handler (void * p_exinf);
static void run_commands (void);
static void run_command (uint32_t cmd);
static void flush_commands (void);
static uint16_t feed_line (uint16_t last_sequence);
static void report_change (motion_state_t before, motion_state_t after);
static void publish_status (bool b_inhibited);
static void publish_odometry (void);
static uint32_t timer_us (void);
static void timing_reset (void);
static void timing_update (uint32_t wake_us);

/* Public function definitions. */

ER
motion_init (void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};
    T_CCYC ccyc = {0};

    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0u;
    motion_flg_ctrl = tk_cre_flg(&cflg);
    if (motion_flg_ctrl < E_OK)
    {
        return motion_flg_ctrl;
    }

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    motion_flg_event = tk_cre_flg(&cflg);
    if (motion_flg_event < E_OK)
    {
        return motion_flg_event;
    }

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = CMD_BUFFER_SIZE;
    cmbf.maxmsz = CMD_SIZE;
    motion_mbf_cmd = tk_cre_mbf(&cmbf);
    if (motion_mbf_cmd < E_OK)
    {
        return motion_mbf_cmd;
    }

    /* Created stopped; motion_start() starts it once tMotion exists. */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)tick_handler;
    ccyc.cyctim = MOTION_UPDATE_PERIOD_MS;
    ccyc.cycphs = MOTION_UPDATE_PERIOD_MS;
    cyc_motion = tk_cre_cyc(&ccyc);
    if (cyc_motion < E_OK)
    {
        return cyc_motion;
    }

    status_word = (uint32_t)ROBOT_MOTION_MODE_STOPPED;
    timing.inhibit_brake_us = TIMER_NONE;
    timing_reset();
    return motion_command_init();
}

ER
motion_start (void)
{
    /* Configured here, before tMotion exists, then owned by tMotion. */
    if ((motion_drive_init() != MOTION_OK) || (motion_move_init() != MOTION_OK))
    {
        tm_printf(
            (UB *)"motion: failed to initialize the motors or encoders\n");
        return E_SYS;
    }
    tm_printf((UB *)"motion: motors initialized and braked\n");
    tm_printf((UB *)"motion: encoders counting on PIO0\n");

    ER const result = motion_command_start();
    if (result < E_OK)
    {
        return result;
    }

    /* TODO(step 9, team): priority 3 (design freeze) or 14 (TEAM.md). */
    ID tid = app_start_task(motion_task, MOTION_PRI_DRIVE, 0u, "tMotion");
    if (tid < E_OK)
    {
        return tid;
    }
    return tk_sta_cyc(cyc_motion);
}

motion_task_status_t
motion_get_status (void)
{
    uint32_t word = status_word;
    motion_task_status_t status;

    status.mode = (robot_motion_mode_t)(word & STATUS_MODE_MASK);
    status.done =
        (robot_motion_done_t)((word >> STATUS_DONE_SHIFT) & STATUS_DONE_MASK);
    return status;
}

bool
motion_get_odometry (motion_shared_odometry_t * p_odometry)
{
    if (NULL == p_odometry)
    {
        return false;
    }
    for (UINT i = 0u; i < ODOMETRY_READ_TRIES; i++)
    {
        uint32_t before = odometry_sequence;

        if (0u == (before & 1u))
        {
            p_odometry->updates = shared_odometry.updates;
            p_odometry->distance_mm = shared_odometry.distance_mm;
            p_odometry->heading_deg = shared_odometry.heading_deg;
            p_odometry->speed_mm_s = shared_odometry.speed_mm_s;
            p_odometry->left_speed_mm_s = shared_odometry.left_speed_mm_s;
            p_odometry->right_speed_mm_s = shared_odometry.right_speed_mm_s;
            if (odometry_sequence == before)
            {
                return true;
            }
        }
    }
    return false;
}

void
motion_set_line_input (int16_t error, bool b_seen)
{
    int16_t clamped = error;
    uint32_t sequence;

    if (clamped > LINE_ERROR_LIMIT)
    {
        clamped = LINE_ERROR_LIMIT;
    }
    else if (clamped < -LINE_ERROR_LIMIT)
    {
        clamped = -LINE_ERROR_LIMIT;
    }

    /* One writer, so the read-modify-write of the sequence is safe. */
    sequence = ((line_word >> LINE_SEQUENCE_SHIFT) + 1u) & LINE_SEQUENCE_MASK;
    if (0u == sequence)
    {
        sequence = 1u; /* 0 means no reading yet. */
    }
    line_word = ((uint32_t)(uint16_t)clamped & LINE_ERROR_MASK) |
                (b_seen ? LINE_SEEN : 0u) | (sequence << LINE_SEQUENCE_SHIFT);
}

#if APP_TEST
ER
motion_task_hold (bool b_hold)
{
    b_hold_request = b_hold;
    for (INT i = 0; i < HOLD_WAIT_TICKS; i++)
    {
        if (b_hold_ack == b_hold)
        {
            return E_OK;
        }
        (void)tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
    }
    return (b_hold_ack == b_hold) ? E_OK : E_TMOUT;
}

void
motion_task_get_timing (motion_task_timing_t * p_timing)
{
    if (p_timing != NULL)
    {
        *p_timing = timing;
    }
}
#endif

/* Private function definitions. */

static void
motion_task (INT stacd, void * p_exinf)
{
    bool b_inhibited = false;
    bool b_held = false;
    uint16_t line_sequence = 0u;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        UINT flags = 0u;
        /* While inhibited or held, INHIBIT stays set: wait on ticks only. */
        UINT wait = (b_inhibited || b_held)
                        ? MOTION_FLG_TICK
                        : (MOTION_FLG_TICK | MOTION_FLG_INHIBIT);

        if (tk_wai_flg(motion_flg_ctrl, wait, TWF_ORW, &flags, TMO_FEVR) !=
            E_OK)
        {
            /* Should not happen; brake rather than run unsupervised. */
            (void)motion_move_stop();
            (void)tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
            continue;
        }
        uint32_t wake_us = timer_us();
        bool b_tick = (flags & MOTION_FLG_TICK) != 0u;
        bool b_inhibit = (flags & MOTION_FLG_INHIBIT) != 0u;

        if (b_tick)
        {
            (void)tk_clr_flg(motion_flg_ctrl, ~(UINT)MOTION_FLG_TICK);
        }

#if APP_TEST
        if (b_hold_request)
        {
            if (!b_held)
            {
                (void)motion_move_stop();
                b_held = true;
                b_hold_ack = true;
            }
            continue;
        }
        if (b_held)
        {
            b_held = false;
            timing_reset();
            b_hold_ack = false;
        }
#endif

        /* NFR6: brake at once, not on the next tick. */
        if (b_inhibit && !b_inhibited)
        {
            b_line_control = false;
            (void)motion_move_stop();
            timing.inhibit_brake_us = timer_us();
            b_inhibited = true;
            publish_status(b_inhibited);
        }
        if (!b_tick)
        {
            continue; /* Inhibit-only wake: update on the next tick. */
        }
        timing_update(wake_us);

        if (b_inhibited)
        {
            /* Discard commands sent while inhibited, including on clear. */
            flush_commands();
            b_inhibited = b_inhibit;
        }
        else
        {
            line_sequence = feed_line(line_sequence);
            run_commands();
        }

        motion_state_t before = motion_move_get_state();
        motion_command_service(b_line_control);
        (void)motion_move_update();
        report_change(before, motion_move_get_state());
        publish_status(b_inhibited);
        publish_odometry();

        uint32_t busy_us = timer_us() - wake_us;
        if (busy_us > timing.max_busy_us)
        {
            timing.max_busy_us = busy_us;
        }
    }
}

/* CYC_MOTION: wake tMotion for one update. Task-independent context. */
static void
tick_handler (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_set_flg(motion_flg_ctrl, MOTION_FLG_TICK);
}

/* Run the queued commands in order; each replaces the one before. */
static void
run_commands (void)
{
    /* At most one buffer's worth, so a busy sender cannot starve updates. */
    for (UINT i = 0u; i < MOTION_CMD_QUEUE_DEPTH; i++)
    {
        uint32_t cmd = 0u;

        if (tk_rcv_mbf(motion_mbf_cmd, &cmd, TMO_POL) != CMD_SIZE)
        {
            return;
        }
        run_command(cmd);
    }
}

/* Start one command; set MOTION_EVT_REJECTED if it is invalid or refused. */
static void
run_command (uint32_t cmd)
{
    motion_status_t result = MOTION_ERR_PARAM;
    robot_motion_cmd_type_t type = robot_motion_cmd_get_type(cmd);
    bool b_reverse = robot_motion_cmd_get_reverse(cmd);

    if ((cmd & ROBOT_MOTION_CMD_RESERVED_MASK) != 0u)
    {
        (void)tk_set_flg(motion_flg_event, MOTION_EVT_REJECTED);
        return;
    }

    switch (type)
    {
        case ROBOT_MOTION_CMD_STOP:
            result = motion_move_stop();
            break;

        case ROBOT_MOTION_CMD_MOVE:
        {
            uint16_t distance_mm = robot_motion_cmd_get_distance(cmd);
            result = b_reverse ? motion_move_backward(distance_mm)
                               : motion_move_forward(distance_mm);
            break;
        }

        case ROBOT_MOTION_CMD_TURN:
        {
            uint16_t angle_deg = robot_motion_cmd_get_angle(cmd);
            if (angle_deg <= ROBOT_MOTION_CMD_ANGLE_MAX_DEG)
            {
                result = b_reverse ? motion_move_turn_right(angle_deg)
                                   : motion_move_turn_left(angle_deg);
            }
            break;
        }

        case ROBOT_MOTION_CMD_FOLLOW:
#if MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST
            /* The adapter selects tracking/alignment/search speed. It is
             * serviced below, still inside tMotion, using fresh line data. */
            if (motion_move_get_state() == MOTION_STATE_STALLED)
            {
                result = MOTION_ERR_STALLED;
            }
            else if (motion_move_get_state() == MOTION_STATE_TIMED_OUT)
            {
                result = MOTION_ERR_TIMED_OUT;
            }
            else
            {
                b_line_control = true;
                result = MOTION_OK;
            }
#else
            result = motion_move_follow(MOTION_FOLLOW_SPEED_MM_S);
#endif
            break;

        default:
            break;
    }

    if ((result == MOTION_OK) && (type != ROBOT_MOTION_CMD_FOLLOW))
    {
        b_line_control = false;
    }

    /* Stop at line applies to moves and turns; ignored on STOP / FOLLOW. */
    if ((MOTION_OK == result) && robot_motion_cmd_get_stop_at_line(cmd) &&
        ((ROBOT_MOTION_CMD_MOVE == type) || (ROBOT_MOTION_CMD_TURN == type)))
    {
        result = motion_move_stop_at_line();
    }
    if (result != MOTION_OK)
    {
        (void)tk_set_flg(motion_flg_event, MOTION_EVT_REJECTED);
    }
}

/* Empty MBF_MOTION_CMD without running anything. */
static void
flush_commands (void)
{
    uint32_t cmd = 0u;

    while (tk_rcv_mbf(motion_mbf_cmd, &cmd, TMO_POL) == CMD_SIZE)
    {
        /* Discarded. */
    }
}

/*
 * Pass a line reading newer than @p last_sequence to the motion module.
 * TODO(step 9, line_follow): lock MTX_LINE and copy line_state_t instead.
 */
static uint16_t
feed_line (uint16_t last_sequence)
{
    uint32_t word = line_word;
    uint16_t sequence =
        (uint16_t)((word >> LINE_SEQUENCE_SHIFT) & LINE_SEQUENCE_MASK);

    if ((sequence != 0u) && (sequence != last_sequence))
    {
        (void)motion_move_set_line((int16_t)(uint16_t)(word & LINE_ERROR_MASK),
                                   (word & LINE_SEEN) != 0u);
    }
    return sequence;
}

/* Raise the event for a state change made by motion_move_update(). */
static void
report_change (motion_state_t before, motion_state_t after)
{
    UINT event = 0u;

    if (before == after)
    {
        return;
    }
    if (MOTION_STATE_STALLED == after)
    {
        event = MOTION_EVT_STALL;
    }
    else if (MOTION_STATE_TIMED_OUT == after)
    {
        event = MOTION_EVT_ENC_FAULT;
    }
    else if ((MOTION_STATE_STOPPED == after) &&
             (motion_move_get_done_reason() != MOTION_DONE_NONE))
    {
        event = MOTION_EVT_MOVE_DONE;
    }
    else if ((MOTION_STATE_FOLLOW == before) && (MOTION_STATE_STOPPED == after))
    {
        event = MOTION_EVT_FOLLOW_ENDED;
    }
    else
    {
        /* Not reported. */
    }

    if (event != 0u)
    {
        (void)tk_set_flg(motion_flg_event, event);
    }
}

/* Publish the mode and done reason as one word for motion_get_status(). */
static void
publish_status (bool b_inhibited)
{
    uint32_t mode = b_inhibited ? (uint32_t)ROBOT_MOTION_MODE_INHIBITED
                                : (uint32_t)motion_move_get_state();
    uint32_t done = (uint32_t)motion_move_get_done_reason();

    status_word = (mode & STATUS_MODE_MASK) |
                  ((done & STATUS_DONE_MASK) << STATUS_DONE_SHIFT);
}

/* Copy this update's odometry for motion_get_odometry(). */
static void
publish_odometry (void)
{
    motion_odometry_t odometry;

    if (motion_move_get_odometry(&odometry) != MOTION_OK)
    {
        return;
    }
    odometry_sequence++; /* Odd: readers retry. */
    shared_odometry.updates++;
    shared_odometry.distance_mm = odometry.distance_mm;
    shared_odometry.heading_deg = odometry.heading_deg;
    shared_odometry.speed_mm_s = odometry.speed_mm_s;
    shared_odometry.left_speed_mm_s = odometry.left_speed_mm_s;
    shared_odometry.right_speed_mm_s = odometry.right_speed_mm_s;
    odometry_sequence++; /* Even: whole again. */
}

/* The free-running 1 MHz TIMER (TEAM.md section 3: read only). */
static uint32_t
timer_us (void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/* Start new timing statistics; keeps the last inhibit time. */
static void
timing_reset (void)
{
    timing.updates = 0u;
    timing.min_period_us = TIMER_NONE;
    timing.max_period_us = 0u;
    timing.max_busy_us = 0u;
    last_wake_us = 0u;
}

/* Count one update wake-up and the time since the one before. */
static void
timing_update (uint32_t wake_us)
{
    if (timing.updates > 0u)
    {
        uint32_t period_us = wake_us - last_wake_us;

        if (period_us < timing.min_period_us)
        {
            timing.min_period_us = period_us;
        }
        if (period_us > timing.max_period_us)
        {
            timing.max_period_us = period_us;
        }
    }
    timing.updates++;
    last_wake_us = wake_us;
}
