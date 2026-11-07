/**
 * @file motion_command.c
 * @brief Pass the line task's requests to the motion module.
 *
 * Two slots pass the latest request with no lock: the line task fills the one
 * tMotion is not reading, then publishes it. A lock both sides only polled
 * could lose every request when the two 20 ms tasks kept meeting at it: the
 * likely cause of the crawl of 2026-11-07, one request a second, so the car
 * drove for one 100 ms timeout at a time. tMotion services this adapter once
 * per update before its single motion_move_update() call. This module creates
 * no task. Dry-run and APP_TEST builds never call motion APIs from it.
 *
 * The line task decides everything; this adapter only maps its request on
 * the motion module: TRACK holds the requested heading at a constant speed,
 * SEARCH pivots to the requested heading once per new target, STOP brakes.
 */

#include "motion_command.h"
#include "motion.h"
#include "motion_command_config.h"
#include "motion_move.h"

/* Private types, constants, and macros. */
#define ACTUATION_ACTIVE (MOTION_COMMAND_ACTUATION_ENABLED && !APP_TEST)
#define SLOT_COUNT 2u
#define STALL_RETRY_UPDATES                                                    \
    (MOTION_COMMAND_STALL_RETRY_MS / MOTION_UPDATE_PERIOD_MS)

_Static_assert(MOTION_COMMAND_FOLLOW_SPEED_MM_S > 0u &&
                   MOTION_COMMAND_FOLLOW_SPEED_MM_S <= MOTION_MAX_SPEED_MM_S,
               "Invalid follow speed");

typedef struct
{
    uint32_t received_ms;
    line_request_t request;
    bool b_valid;
} command_slot_t;

/* Static data. */
static bool b_ready;
static bool b_started;
#if ACTUATION_ACTIVE
static uint32_t startup_ms;
static bool b_control_previous;
#endif
/*
 * The line task writes the slot after the published one, then publishes it.
 * The next write to the published slot is two requests away, 40 ms, and
 * tMotion's copy takes microseconds at the top priority of its core.
 */
static command_slot_t slots[SLOT_COUNT];
static uint32_t published; /* Requests published; picks the slot. */
/* The following state belongs exclusively to the motion task. */
static command_slot_t active_slot;
static uint32_t taken; /* The last request published that was copied. */
static volatile uint32_t taken_count;
#if ACTUATION_ACTIVE
static bool b_fault_latched;
static uint32_t stall_updates;
static bool b_search_active;
static bool b_reverse_active;
static int32_t search_target_deg;
#endif

/* Private function declarations. */
static bool take_request (void);
#if ACTUATION_ACTIVE
static void update_controller (bool b_new);
static motion_status_t apply_request (void);
static motion_status_t start_search (int32_t target_deg);
static void latch_fault (motion_status_t status);
#else
static void report_request (void);
#endif

/* Public function definitions. */

ER
motion_command_init (void)
{
    if (b_ready)
    {
        return E_OBJ;
    }
    b_ready = true;
    return E_OK;
}

ER
motion_command_start (void)
{
    if (!b_ready || b_started)
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
motion_command_submit (line_request_t const * p_request)
{
    command_slot_t slot = {.request = {.mode = LINE_MODE_STOP}};
    bool const b_mode_valid =
        (p_request == NULL) ||
        ((unsigned int)p_request->mode <= (unsigned int)LINE_MODE_STOP);
    SYSTIM now;

    if ((p_request != NULL) && b_mode_valid)
    {
        slot.request = *p_request;
        slot.b_valid = true;
    }
    if (!b_ready)
    {
        return E_OBJ;
    }
    ER const result = tk_get_otm(&now);
    if (result == E_OK)
    {
        uint32_t const next = published + 1u;
        slot.received_ms = (uint32_t)now.lo;
        slots[next % SLOT_COUNT] = slot;
        __atomic_store_n(&published, next, __ATOMIC_RELEASE);
    }
    return (result < E_OK) ? result : (b_mode_valid ? E_OK : E_PAR);
}

void
motion_command_get_counts (uint32_t * p_published, uint32_t * p_taken)
{
    *p_published = __atomic_load_n(&published, __ATOMIC_RELAXED);
    *p_taken = taken_count;
}

/* Public owner hook: tMotion remains the sole caller of motion APIs. */
void
motion_command_service (bool b_control_enabled)
{
    SYSTIM now;
    bool const b_new = take_request();
    bool const b_clock = (tk_get_otm(&now) == E_OK);
    if (!b_clock || ((uint32_t)((uint32_t)now.lo - active_slot.received_ms) >=
                     MOTION_LINE_TIMEOUT_MS))
    {
        active_slot.b_valid = false;
    }
#if ACTUATION_ACTIVE
    bool const b_take_over = b_control_enabled && !b_control_previous;
    b_control_previous = b_control_enabled;
    if (!b_control_enabled)
    {
        b_search_active = false;
        b_reverse_active = false;
        /* External MOVE/TURN stop-at-line still receives current visibility,
         * but no line request may replace the queued command or inhibit. */
        if (b_new && active_slot.b_valid)
        {
            (void)motion_move_set_line(active_slot.request.b_seen);
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

/* Copy the latest request, if there is a new one. */
static bool
take_request (void)
{
    uint32_t const latest = __atomic_load_n(&published, __ATOMIC_ACQUIRE);
    bool const b_new = latest != taken;
    if (b_new)
    {
        active_slot = slots[latest % SLOT_COUNT];
        taken = latest;
        taken_count++;
    }
    return b_new;
}

#if ACTUATION_ACTIVE
static void
update_controller (bool b_new)
{
    motion_state_t const state = motion_move_get_state();
    if (state == MOTION_STATE_STALLED)
    {
        latch_fault(MOTION_ERR_STALLED);
        /* Retry instead of staying stopped for good. */
        if (++stall_updates >= STALL_RETRY_UPDATES)
        {
            stall_updates = 0u;
            (void)motion_move_stop();
            b_fault_latched = false;
            b_search_active = false;
            tm_printf((UB *)"motion: stall cleared, following again\n");
        }
    }
    else if (state == MOTION_STATE_TIMED_OUT)
    {
        latch_fault(MOTION_ERR_TIMED_OUT);
    }
    if (!b_fault_latched)
    {
        motion_status_t status = MOTION_OK;
        if (!active_slot.b_valid)
        {
            b_search_active = false;
            if (motion_move_is_busy())
            {
                tm_printf((UB *)"motion: no line request for %u ms, braking\n",
                          (UINT)MOTION_LINE_TIMEOUT_MS);
                status = motion_move_stop();
            }
        }
        else if (b_new)
        {
            status = apply_request();
        }
        if (status != MOTION_OK)
        {
            latch_fault(status);
        }
    }
}

static motion_status_t
apply_request (void)
{
    line_request_t const * p_request = &active_slot.request;
    motion_status_t status = motion_move_set_line(p_request->b_seen);
    if (status != MOTION_OK)
    {
        return status;
    }
    if (p_request->reverse_mm > 0u)
    {
        /* One move; the line task moves on when it has ended. */
        b_search_active = false;
        if (!b_reverse_active)
        {
            b_reverse_active = true;
            status = motion_move_backward(p_request->reverse_mm);
        }
        return status;
    }
    b_reverse_active = false;
    uint16_t const speed_mm_s = p_request->b_slow
                                    ? MOTION_COMMAND_SLOW_SPEED_MM_S
                                    : MOTION_COMMAND_FOLLOW_SPEED_MM_S;
    switch (p_request->mode)
    {
        case LINE_MODE_TRACK:
            b_search_active = false;
            status = (motion_move_get_state() != MOTION_STATE_FOLLOW)
                         ? motion_move_follow(speed_mm_s)
                         : motion_move_set_follow_speed(speed_mm_s);
            if (status == MOTION_OK)
            {
                status = motion_move_set_follow_heading(
                    p_request->target_heading_deg);
            }
            break;

        case LINE_MODE_SEARCH_LEFT:
        case LINE_MODE_SEARCH_RIGHT:
            /* One pivot per target; the line task moves the target on. */
            if (!b_search_active ||
                (search_target_deg != p_request->target_heading_deg))
            {
                status = start_search(p_request->target_heading_deg);
            }
            break;

        default:
            b_search_active = false;
            if (motion_move_is_busy())
            {
                status = motion_move_stop();
            }
            break;
    }
    return status;
}

/* Pivot from the current heading to @p target_deg. */
static motion_status_t
start_search (int32_t target_deg)
{
    motion_odometry_t odometry;
    motion_status_t status = motion_move_get_odometry(&odometry);
    if (status != MOTION_OK)
    {
        return status;
    }
    b_search_active = true;
    search_target_deg = target_deg;

    int32_t const turn = target_deg - odometry.heading_deg;
    uint32_t magnitude = (uint32_t)((turn < 0) ? -turn : turn);
    if (magnitude > MOTION_MAX_TURN_DEG)
    {
        magnitude = MOTION_MAX_TURN_DEG;
    }
    if (0u == magnitude)
    {
        return motion_move_is_busy() ? motion_move_stop() : MOTION_OK;
    }
    return (turn > 0) ? motion_move_turn_left((uint16_t)magnitude)
                      : motion_move_turn_right((uint16_t)magnitude);
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
#else
static void
report_request (void)
{
    static line_mode_t previous = LINE_MODE_STOP;
    static bool b_previous_valid;
    line_mode_t const mode =
        active_slot.b_valid ? active_slot.request.mode : LINE_MODE_STOP;
    /* Log transitions only, so the dry run does not flood the console. */
    if ((mode != previous) || (active_slot.b_valid != b_previous_valid))
    {
        tm_printf((UB *)"motion: %s target=%d seen=%u valid=%u (dry run)\n",
                  line_track_mode_name(mode),
                  (INT)active_slot.request.target_heading_deg,
                  (UINT)active_slot.request.b_seen, (UINT)active_slot.b_valid);
        previous = mode;
        b_previous_valid = active_slot.b_valid;
    }
}
#endif /* ACTUATION_ACTIVE */
