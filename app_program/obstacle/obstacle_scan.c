/**
 * @file obstacle_scan.c
 * @brief Coordinate servo settling and fresh ultrasonic measurements.
 */

#include "obstacle_scan.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "obstacle_config.h"
#include "obstacle_decision.h"
#include "obstacle_width.h"
/* Use the toolchain size_t alongside the kernel headers. */
#define PROHIBIT_DEF_SIZE_T
#include <stdbool.h>
#include <stddef.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#if APP_TEST
#include <bsp/libbsp.h>
#include <tk/syslib.h>
#endif

/* Private types, constants, and macros. */

#define GPIO_COUNT 30
#define CENTRE_ANGLE ((int)OBSTACLE_CENTRE_DEG)
#define MAX_SERVO_TRAVEL_DEGREES 180u
#define CENTRE_INDEX ((uint8_t)OBSTACLE_CENTRE_INDEX)
#define LAST_INDEX ((uint8_t)(OBSTACLE_SWEEP_POINTS - 1u))

/* Packed scan_state: phase 0..1, count 2..3, side 4 (coarse side, then fine
 * side: 1 = right), scanning 5, retries 6..7, invalid position 8, left side
 * result 9..10, critical 11, critical hold 12, fine walk 13, corridor sweep
 * 14. Bit 15 is unused.
 */
#define SCAN_PHASE_MASK 0x0003u
#define SCAN_PHASE_SHIFT 0u
#define SCAN_PING_MASK 0x000Cu
#define SCAN_PING_SHIFT 2u
#define SCAN_SIDE_MASK 0x0010u
#define SCAN_SIDE_SHIFT 4u
#define SCAN_SCANNING_SIDES 0x0020u
#define SCAN_RETRY_MASK 0x00C0u
#define SCAN_RETRY_SHIFT 6u
#define SCAN_POSITION_INVALID 0x0100u
#define SCAN_LEFT_RESULT_MASK 0x0600u
#define SCAN_LEFT_RESULT_SHIFT 9u
#define SCAN_CRITICAL 0x0800u
#define SCAN_CRITICAL_HOLD 0x1000u
#define SCAN_FINE 0x2000u
#define SCAN_CORRIDOR 0x4000u

_Static_assert(OBSTACLE_PINGS_PER_ANGLE >= 1u && OBSTACLE_PINGS_PER_ANGLE <= 3u,
               "Ping count must fit two bits and the small median helper");
_Static_assert(ULTRASONIC_FAULT <= 3u,
               "Ultrasonic results must fit the two-bit left result field");
_Static_assert(OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE <= 3u,
               "Extra attempts must fit the two-bit retry field");
_Static_assert(OBSTACLE_CENTRE_CONFIRM_PINGS <= OBSTACLE_PINGS_PER_ANGLE,
               "pings_us must hold every centre vote");

typedef enum
{
    SERVO_SETTLING,
    SCAN_READY,
    SCAN_MEASURING
} scan_phase_t;

/* Static data. */

/* Servo should scan these angles to the left and right of centre. */
static const int scan_angles[] = {OBSTACLE_SWEEP_FIRST_DEG,
                                  OBSTACLE_SWEEP_LAST_DEG};
static obstacle_servo_timing_t servo_timing;
static int sensor_angle;
static uint16_t scan_state;
static uint32_t settle_start_ms;
static uint32_t settle_duration_ms;
static uint16_t pings_us[OBSTACLE_PINGS_PER_ANGLE];
static uint16_t left_echo_us;
static uint8_t centre_near_count;
static bool b_stop_requested = true;
static bool b_forward_triggered;
static uint32_t last_forward_trigger_ms;
static robot_obstacle_result_t result;
static uint32_t last_usable_centre_ms;
static uint32_t side_scan_start_ms;

/* Per-sweep state, reset only by reset_sweep(), never by servo moves. */
static uint16_t centre_echo_us;
static uint8_t edge_indices; /* Probe, then hit, index per side. */
static uint16_t left_edge_echo_us;
static uint16_t right_edge_echo_us;
static uint8_t pending_flags; /* Coarse status and direction. */

/* Written only by publish_result(), so it always matches result. */
static obstacle_scan_geometry_t published_geometry;

/* Corridor sweep: published copy, working copy and next grid index. */
static obstacle_scan_corridor_t corridor;
static obstacle_scan_corridor_t corridor_work;
static uint8_t corridor_index;
static uint32_t corridor_start_ms;

static const obstacle_width_config_t width_config = {
    .scale_mm_per_10k = OBSTACLE_SOUND_MM_PER_10K_US,
    .echo_offset_mm = OBSTACLE_ECHO_OFFSET_MM,
    .radial_offset_mm = OBSTACLE_SENSOR_RADIAL_OFFSET_MM,
    .edge_correction_deg = OBSTACLE_EDGE_CORRECTION_DEG};

#if APP_TEST
/* Created before either task starts; no ISR uses this mutex or mailbox. */
static ID test_mutex;
static obstacle_scan_test_snapshot_t test_snapshot;
static bool test_hold_requested;
static uint32_t test_hold_epoch;
static bool test_inflight;
static bool test_initialized;
static uint32_t test_receipt_us;
static uint32_t test_critical_count;
static uint32_t test_critical_receipt_us;
static uint32_t test_critical_flag_us;
static uint16_t test_critical_echo_us;
static uint32_t test_stale_count;
static uint32_t test_stale_ms;
static uint32_t test_timeout_count;
static uint32_t test_timeout_ms;
static obstacle_scan_test_reading_t test_script[OBSTACLE_SCAN_TEST_SCRIPT_MAX];
static uint8_t test_script_count;
static uint8_t test_script_next;
static obstacle_scan_test_reading_t
    test_pending_script[OBSTACLE_SCAN_TEST_SCRIPT_MAX];
static uint8_t test_pending_count;
static bool test_command_pending;
static obstacle_scan_test_command_t test_command;
static uint8_t test_rearm_sequence;
static uint32_t test_corridor_refused;
#endif

/* Private function declarations. */
#if APP_TEST
static robot_obstacle_result_t scan_update (void);
static void script_take (ultrasonic_result_t * p_measurement,
                         uint16_t * p_echo_us);
static void run_test_command (void);
#endif

static uint16_t state_get (uint16_t mask, uint8_t shift);
static void state_set (uint16_t mask, uint8_t shift, uint16_t value);
static bool state_has (uint16_t mask);
static void state_flag (uint16_t mask, bool enabled);
static void reset_position (void);
static void reset_sweep (void);
static void read_centre (ultrasonic_result_t measurement, uint16_t echo_us);
static void request_stop (void);
static void scan_centre (obstacle_centre_action_t action);
static void scan_sides (ultrasonic_result_t measurement, uint16_t echo_us);
static void start_fine_side (bool b_right, ultrasonic_result_t end,
                             uint16_t end_echo_us);
static void probe_fine (bool b_right, uint8_t index);
static void read_fine (ultrasonic_result_t measurement, uint16_t echo_us);
static void record_hit (bool b_right, uint8_t index, uint16_t echo_us);
static void next_fine_side (bool b_right);
static void finish_sweep (obstacle_width_result_t const * p_width);
static uint8_t edge_index (bool b_right);
static int index_angle (uint8_t index);
static char const * measurement_name (ultrasonic_result_t measurement);
static void move_sensor (int angle);
static uint32_t now_ms (void);
static robot_obstacle_result_t read_result (void);
static void publish_result (robot_obstacle_status_t status,
                            robot_obstacle_direction_t direction,
                            obstacle_width_result_t const * p_width,
                            obstacle_scan_geometry_t const * p_geometry);
static obstacle_scan_geometry_t
sweep_geometry (obstacle_width_result_t const * p_width);
static void check_watchdogs (void);
static void stop_side_scan (void);
static void cancel_side_scan (void);
static void record_corridor (ultrasonic_result_t measurement, uint16_t echo_us);
static void finish_corridor (bool b_complete);
static void update_critical (ultrasonic_result_t measurement, uint16_t echo_us);

/* Public function definitions. */

int
obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin,
                    const obstacle_servo_timing_t * timing)
{
    /* Validate before either component configures hardware. */
    if (trig_pin < 0 || trig_pin >= GPIO_COUNT || echo_pin < 0 ||
        echo_pin >= GPIO_COUNT || servo_pin < 0 || servo_pin >= GPIO_COUNT ||
        trig_pin == echo_pin || trig_pin == servo_pin ||
        echo_pin == servo_pin || timing == NULL ||
        (timing->travel_ms_per_60_degrees == 0u &&
         timing->settle_margin_ms == 0u))
    {
        return 1;
    }
    if (servo_init(servo_pin) || ultrasonic_init(trig_pin, echo_pin))
    {
        return 1;
    }

    servo_timing = *timing;
    sensor_angle = -1;
    scan_state = 0u;
    left_echo_us = 0u;
    b_stop_requested = true;
    b_forward_triggered = false;
    last_forward_trigger_ms = 0u;
    last_usable_centre_ms = now_ms();
    side_scan_start_ms = 0u;
    reset_sweep();
    result = (robot_obstacle_result_t){
        .flags = robot_obstacle_pack_flags(
            ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, false, false),
        .edges = robot_obstacle_pack_edges(ROBOT_OBSTACLE_EDGE_NONE,
                                           ROBOT_OBSTACLE_EDGE_NONE),
        .width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE,
        .sequence = 0u};
    published_geometry = (obstacle_scan_geometry_t){0u, 0, 0};
    corridor = (obstacle_scan_corridor_t){0};
    corridor_work = corridor;
    corridor_index = 0u;
    corridor_start_ms = 0u;
    move_sensor(CENTRE_ANGLE);
#if APP_TEST
    test_initialized = true;
#endif
    return 0;
}

#if APP_TEST
int
obstacle_scan_test_init (void)
{
    T_CMTX params = {.mtxatr = TA_INHERIT};
    test_mutex = tk_cre_mtx(&params);
    return test_mutex > 0 ? 0 : 1;
}

int
obstacle_scan_test_request (bool hold)
{
    if (test_mutex <= 0 || tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK)
    {
        return 1;
    }
    if (test_hold_requested != hold)
    {
        test_snapshot.hold_acknowledged = false;
        ++test_hold_epoch;
    }
    test_hold_requested = hold;
    (void)tk_unl_mtx(test_mutex);
    return 0;
}

int
obstacle_scan_test_snapshot (obstacle_scan_test_snapshot_t * snapshot)
{
    if (snapshot == NULL || test_mutex <= 0 ||
        tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK)
    {
        return 1;
    }
    *snapshot = test_snapshot;
    snapshot->hold_requested = test_hold_requested;
    (void)tk_unl_mtx(test_mutex);
    return 0;
}

int
obstacle_scan_test_script (obstacle_scan_test_reading_t const * p_readings,
                           uint8_t count)
{
    if ((count > OBSTACLE_SCAN_TEST_SCRIPT_MAX) ||
        ((p_readings == NULL) && (count != 0u)) || (test_mutex <= 0) ||
        (tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK))
    {
        return 1;
    }
    for (uint8_t index = 0u; index < count; ++index)
    {
        test_script[index] = p_readings[index];
    }
    test_script_count = count;
    test_script_next = 0u;
    test_snapshot.script_remaining = count;
    (void)tk_unl_mtx(test_mutex);
    return 0;
}

int
obstacle_scan_test_command (obstacle_scan_test_command_t command,
                            obstacle_scan_test_reading_t const * p_readings,
                            uint8_t count)
{
    if ((count > OBSTACLE_SCAN_TEST_SCRIPT_MAX) ||
        ((p_readings == NULL) && (count != 0u)) ||
        ((command != OBSTACLE_SCAN_TEST_REARM) &&
         (command != OBSTACLE_SCAN_TEST_CORRIDOR)) ||
        (test_mutex <= 0) || (tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK))
    {
        return 1;
    }
    int status = 1;
    if (!test_command_pending)
    {
        for (uint8_t index = 0u; index < count; ++index)
        {
            test_pending_script[index] = p_readings[index];
        }
        test_pending_count = count;
        test_command = command;
        test_command_pending = true;
        test_snapshot.command_pending = true;
        status = 0;
    }
    (void)tk_unl_mtx(test_mutex);
    return status;
}

robot_obstacle_result_t
obstacle_scan_update (void)
{
    bool hold;
    uint32_t hold_epoch;
    if (tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK)
    {
        return result;
    }
    hold = test_hold_requested;
    hold_epoch = test_hold_epoch;
    (void)tk_unl_mtx(test_mutex);
    if (hold)
    {
        /* Freeze new triggers, consuming any old transaction before ack. */
        if (test_inflight)
        {
            uint16_t discard_us;
            if (ultrasonic_read(&discard_us) != ULTRASONIC_PENDING)
            {
                test_inflight = false;
                if (state_get(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT) ==
                    SCAN_MEASURING)
                {
                    state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
                }
            }
        }
        check_watchdogs();
    }
    else
    {
        run_test_command();
        (void)scan_update();
    }
    if (tk_loc_mtx(test_mutex, TMO_FEVR) == E_OK)
    {
        test_snapshot.initialized = test_initialized;
        ++test_snapshot.heartbeat;
        test_snapshot.stop_requested = b_stop_requested;
        test_snapshot.scanning_sides = state_has(SCAN_SCANNING_SIDES);
        test_snapshot.status = robot_obstacle_get_status(result.flags);
        test_snapshot.hold_acknowledged =
            hold && !test_inflight && hold_epoch == test_hold_epoch;
        test_snapshot.last_usable_centre_ms = last_usable_centre_ms;
        test_snapshot.side_scan_start_ms = side_scan_start_ms;
        test_snapshot.stale_count = test_stale_count;
        test_snapshot.stale_ms = test_stale_ms;
        test_snapshot.timeout_count = test_timeout_count;
        test_snapshot.timeout_ms = test_timeout_ms;
        test_snapshot.critical_count = test_critical_count;
        test_snapshot.critical_receipt_us = test_critical_receipt_us;
        test_snapshot.critical_flag_us = test_critical_flag_us;
        test_snapshot.critical_echo_us = test_critical_echo_us;
        test_snapshot.result = result;
        test_snapshot.geometry = published_geometry;
        test_snapshot.script_remaining =
            (uint8_t)(test_script_count - test_script_next);
        test_snapshot.corridor = corridor;
        test_snapshot.rearm_sequence = test_rearm_sequence;
        test_snapshot.command_pending = test_command_pending;
        test_snapshot.corridor_refused = test_corridor_refused;
        (void)tk_unl_mtx(test_mutex);
    }
    return result;
}

static robot_obstacle_result_t
scan_update (void)
#else
robot_obstacle_result_t
obstacle_scan_update (void)
#endif
{
    check_watchdogs();
    switch ((scan_phase_t)state_get(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT))
    {
        case SERVO_SETTLING:
        {
            if ((uint32_t)(now_ms() - settle_start_ms) < settle_duration_ms)
            {
                return result;
            }
            /* Drop any reading still in flight from before the move. */
            uint16_t discard_us;
#if APP_TEST
            ultrasonic_result_t discarded = ultrasonic_read(&discard_us);
            if (test_inflight && discarded == ULTRASONIC_PENDING)
            {
                return result;
            }
            test_inflight = false;
#else
            (void)ultrasonic_read(&discard_us);
#endif
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            return result;
        }
        case SCAN_READY:
        {
            bool b_centre =
                !state_has(SCAN_SCANNING_SIDES) && !state_has(SCAN_CORRIDOR);
            uint32_t trigger_ms = now_ms();
            if (b_centre && b_forward_triggered &&
                ((uint32_t)(trigger_ms - last_forward_trigger_ms) <
                 OBSTACLE_FORWARD_PERIOD_MS))
            {
                return result;
            }
            /* The driver also enforces minimum spacing across all angles. */
            if (ultrasonic_trigger())
            {
#if APP_TEST
                test_inflight = true;
#endif
                if (b_centre)
                {
                    last_forward_trigger_ms = trigger_ms;
                    b_forward_triggered = true;
                }
                state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_MEASURING);
            }
            return result;
        }
        case SCAN_MEASURING:
            return read_result();
        default:
            if (state_has(SCAN_SCANNING_SIDES))
            {
                uint32_t recovery_ms = now_ms();
                tm_printf((UB *)"[%lu ms]obstacle: side scan aborted for state "
                                "recovery after %lu ms\n",
                          (unsigned long)recovery_ms,
                          (unsigned long)(uint32_t)(recovery_ms -
                                                    side_scan_start_ms));
            }
            else
            {
                tm_printf((UB *)"[%lu ms]obstacle: state recovery\n",
                          (unsigned long)now_ms());
            }
            sensor_angle = -1;
            cancel_side_scan();
            return result;
    }
}

bool
obstacle_scan_stop_requested (void)
{
    return b_stop_requested;
}

obstacle_scan_geometry_t
obstacle_scan_geometry (void)
{
    return published_geometry;
}

uint8_t
obstacle_scan_rearm (void)
{
    if (state_has(SCAN_CORRIDOR))
    {
        finish_corridor(false);
    }
    stop_side_scan();
    b_stop_requested = true;
    publish_result(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, NULL,
                   NULL);
    /* move_sensor() also discards pings, votes and retries. */
    move_sensor(CENTRE_ANGLE);
    tm_printf((UB *)"[%lu ms]obstacle: rearmed, sequence %u\n",
              (unsigned long)now_ms(), (unsigned int)result.sequence);
    return result.sequence;
}

bool
obstacle_scan_request_corridor (void)
{
    if (state_has(SCAN_SCANNING_SIDES) || state_has(SCAN_CORRIDOR))
    {
        return false;
    }
    request_stop();
    corridor_work = (obstacle_scan_corridor_t){0};
    corridor_index = 0u;
    corridor_start_ms = now_ms();
    state_flag(SCAN_CORRIDOR, true);
    move_sensor(index_angle(0u));
    tm_printf((UB *)"[%lu ms]obstacle: corridor sweep started\n",
              (unsigned long)corridor_start_ms);
    return true;
}

obstacle_scan_corridor_t
obstacle_scan_corridor (void)
{
    return corridor;
}

char const *
obstacle_scan_status_name (robot_obstacle_status_t status)
{
    switch (status)
    {
        case ROBOT_OBSTACLE_SCANNING:
            return "SCANNING";
        case ROBOT_OBSTACLE_CLEAR:
            return "CLEAR";
        case ROBOT_OBSTACLE_PATH_FOUND:
            return "PATH_FOUND";
        case ROBOT_OBSTACLE_BLOCKED:
            return "BLOCKED";
        case ROBOT_OBSTACLE_UNKNOWN:
            return "UNKNOWN";
        default:
            return "INVALID";
    }
}

/* Private function definitions. */

/* Only the scan task accesses scan_state; no shared-state lock is needed. */
static uint16_t
state_get (uint16_t mask, uint8_t shift)
{
    return (uint16_t)((scan_state & mask) >> shift);
}

static void
state_set (uint16_t mask, uint8_t shift, uint16_t value)
{
    scan_state =
        (uint16_t)((scan_state & (uint16_t)~mask) | ((value << shift) & mask));
}

static bool
state_has (uint16_t mask)
{
    return (scan_state & mask) != 0u;
}

static void
state_flag (uint16_t mask, bool enabled)
{
    if (enabled)
    {
        scan_state |= mask;
    }
    else
    {
        scan_state &= (uint16_t)~mask;
    }
}

static void
reset_position (void)
{
    state_set(SCAN_PING_MASK, SCAN_PING_SHIFT, 0u);
    state_set(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT, 0u);
    state_flag(SCAN_POSITION_INVALID, false);
    state_flag(SCAN_CRITICAL_HOLD, false);
    centre_near_count = 0u;
    for (size_t index = 0u; index < OBSTACLE_PINGS_PER_ANGLE; ++index)
    {
        pings_us[index] = 0u;
    }
}

/* Sweep state outlives servo moves: reset at sweep start, end and cancel. */
static void
reset_sweep (void)
{
    state_flag(SCAN_FINE, false);
    centre_echo_us = 0u;
    edge_indices = robot_obstacle_pack_edges(ROBOT_OBSTACLE_EDGE_NONE,
                                             ROBOT_OBSTACLE_EDGE_NONE);
    left_edge_echo_us = 0u;
    right_edge_echo_us = 0u;
    pending_flags = robot_obstacle_pack_flags(
        ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, false, false);
}

static robot_obstacle_result_t
read_result (void)
{
    uint16_t echo_us;
    ultrasonic_result_t measurement = ultrasonic_read(&echo_us);
    if (measurement == ULTRASONIC_PENDING)
    {
        return result;
    }
#if APP_TEST
    test_inflight = false;
    test_receipt_us = (uint32_t)in_w(TIMER_TIMERAWL);
    script_take(&measurement, &echo_us);
#endif
    if (!state_has(SCAN_SCANNING_SIDES) && !state_has(SCAN_CORRIDOR))
    {
        read_centre(measurement, echo_us);
        return result;
    }
    if (state_has(SCAN_FINE))
    {
        read_fine(measurement, echo_us);
        return result;
    }
    if ((measurement != ULTRASONIC_VALID) &&
        (measurement != ULTRASONIC_NO_ECHO))
    {
        uint16_t retries = state_get(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT);
        if (retries < OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE)
        {
            state_set(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT, retries + 1u);
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            return result;
        }
        state_flag(SCAN_POSITION_INVALID, true);
        echo_us = 0u;
    }
    else if (measurement == ULTRASONIC_NO_ECHO)
    {
        echo_us = 0u;
    }

    uint16_t count = state_get(SCAN_PING_MASK, SCAN_PING_SHIFT);
    pings_us[count] = echo_us;
    ++count;
    state_set(SCAN_PING_MASK, SCAN_PING_SHIFT, count);
    if (count < OBSTACLE_PINGS_PER_ANGLE)
    {
        /* Every additional ping must pass through the trigger phase again. */
        state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
        return result;
    }

    /* No-echo pings are stored as zero, which the median ignores. */
    echo_us = obstacle_decision_median(pings_us, OBSTACLE_PINGS_PER_ANGLE);
    measurement =
        obstacle_decision_position(echo_us, state_has(SCAN_POSITION_INVALID));
    if (state_has(SCAN_CORRIDOR))
    {
        record_corridor(measurement, echo_us);
    }
    else
    {
        scan_sides(measurement, echo_us);
    }
    return result;
}

/* Centre faults consume retries, never votes. A group has three usable
 * pings; the triggering near ping is its first vote during clear monitoring.
 */
static void
read_centre (ultrasonic_result_t measurement, uint16_t echo_us)
{
    obstacle_centre_action_t action =
        obstacle_decision_centre(measurement, echo_us);
    if (action == OBSTACLE_CENTRE_UNKNOWN)
    {
        uint16_t retries = state_get(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT);
#if APP_TEST
        /* Software consumption trace, not the physical disconnect instant. */
        if (measurement == ULTRASONIC_FAULT)
        {
            tm_printf((UB *)"[%lu ms]obstacle test: centre FAULT consumed "
                            "retry %u attempt %u\n",
                      (unsigned long)now_ms(), (unsigned int)retries,
                      (unsigned int)(retries + 1u));
        }
#endif
        request_stop();
        if (retries < OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE)
        {
            state_set(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT, retries + 1u);
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
        }
        else
        {
            /* Discard the incomplete group; recovery needs three new votes. */
            scan_centre(OBSTACLE_CENTRE_UNKNOWN);
        }
        return;
    }

    last_usable_centre_ms = now_ms();
    if (action == OBSTACLE_CENTRE_SCAN_SIDES)
    {
        request_stop();
        /* Keep each near echo: their median is the sweep reference. */
        if (centre_near_count < OBSTACLE_PINGS_PER_ANGLE)
        {
            pings_us[centre_near_count] = echo_us;
        }
        ++centre_near_count;
    }
    update_critical(measurement, echo_us);
    if (!b_stop_requested)
    {
        /* A clear ping is beyond the release distance, so it clears too. */
        state_flag(SCAN_CRITICAL, false);
        scan_centre(OBSTACLE_CENTRE_CLEAR);
        return;
    }

    uint16_t count = state_get(SCAN_PING_MASK, SCAN_PING_SHIFT) + 1u;
    state_set(SCAN_PING_MASK, SCAN_PING_SHIFT, count);
    if (count < OBSTACLE_CENTRE_CONFIRM_PINGS)
    {
        state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
        return;
    }
    /* A complete group with nothing inside 100 mm clears the flag. */
    if (!state_has(SCAN_CRITICAL_HOLD))
    {
        state_flag(SCAN_CRITICAL, false);
    }
    if (centre_near_count >= OBSTACLE_CENTRE_NEAR_VOTES)
    {
        /* Capture the reference before the first move clears pings_us. */
        reset_sweep();
        centre_echo_us = obstacle_decision_median(
            pings_us, (uint8_t)OBSTACLE_CENTRE_CONFIRM_PINGS);
        scan_centre(OBSTACLE_CENTRE_SCAN_SIDES);
    }
    else
    {
        scan_centre(OBSTACLE_CENTRE_CLEAR);
    }
}

static void
request_stop (void)
{
    if (!b_stop_requested)
    {
        b_stop_requested = true;
        publish_result(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, NULL,
                       NULL);
    }
}

static void
scan_centre (obstacle_centre_action_t action)
{
    switch (action)
    {
        case OBSTACLE_CENTRE_CLEAR:
            b_stop_requested = false;
            publish_result(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90,
                           NULL, NULL);
            reset_position();
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            break;
        case OBSTACLE_CENTRE_SCAN_SIDES:
            /* Retain the sequence while both sides are being sampled. */
            result.flags = robot_obstacle_set_critical(
                robot_obstacle_pack_flags(ROBOT_OBSTACLE_SCANNING,
                                          ROBOT_OBSTACLE_DIR_NONE, false,
                                          false),
                state_has(SCAN_CRITICAL));
            state_flag(SCAN_SCANNING_SIDES, true);
            state_set(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT, 0u);
            state_set(SCAN_LEFT_RESULT_MASK, SCAN_LEFT_RESULT_SHIFT,
                      ULTRASONIC_PENDING);
            left_echo_us = 0u;
            side_scan_start_ms = now_ms();
            move_sensor(scan_angles[0]);
            tm_printf((UB *)"[%lu ms]obstacle: side scan started\n",
                      (unsigned long)side_scan_start_ms);
            break;
        case OBSTACLE_CENTRE_UNKNOWN:
        default:
            b_stop_requested = true;
            publish_result(ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE,
                           NULL, NULL);
            reset_position();
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            break;
    }
}

static void
scan_sides (ultrasonic_result_t measurement, uint16_t echo_us)
{
    if (state_get(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT) == 0u)
    {
        /* Keep the left reading until the right position completes. */
        state_set(SCAN_LEFT_RESULT_MASK, SCAN_LEFT_RESULT_SHIFT,
                  (uint16_t)measurement);
        left_echo_us = echo_us;
        state_set(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT, 1u);
        move_sensor(scan_angles[1]);
        return;
    }

    /* The direction comes from both ends; publish after the fine walks. */
    ultrasonic_result_t left = (ultrasonic_result_t)state_get(
        SCAN_LEFT_RESULT_MASK, SCAN_LEFT_RESULT_SHIFT);
    obstacle_side_decision_t decision = obstacle_decision_sides(
        left, left_echo_us, measurement, echo_us, centre_echo_us);
    pending_flags = robot_obstacle_pack_flags(decision.status,
                                              decision.direction, false, false);
    if ((obstacle_width_classify(left, left_echo_us, centre_echo_us) ==
         OBSTACLE_WIDTH_INVALID) ||
        (obstacle_width_classify(measurement, echo_us, centre_echo_us) ==
         OBSTACLE_WIDTH_INVALID))
    {
        /* A faulty end is no evidence of an edge: no width, same direction. */
        obstacle_width_result_t width = obstacle_width_unavailable();
        finish_sweep(&width);
        return;
    }
    start_fine_side(true, measurement, echo_us);
}

/* An end on the obstacle is its outermost hit; otherwise walk inward. */
static void
start_fine_side (bool b_right, ultrasonic_result_t end, uint16_t end_echo_us)
{
    uint8_t end_index = b_right ? LAST_INDEX : 0u;
    if (obstacle_width_classify(end, end_echo_us, centre_echo_us) ==
        OBSTACLE_WIDTH_HIT)
    {
        record_hit(b_right, end_index, end_echo_us);
        next_fine_side(b_right);
        return;
    }
    state_flag(SCAN_FINE, true);
    state_set(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT, b_right ? 1u : 0u);
    probe_fine(b_right, b_right ? (uint8_t)(end_index - 1u) : 1u);
}

/* The centre is known to be on the obstacle, so the walk ends there. */
static void
probe_fine (bool b_right, uint8_t index)
{
    if (index == CENTRE_INDEX)
    {
        record_hit(b_right, CENTRE_INDEX, centre_echo_us);
        next_fine_side(b_right);
        return;
    }
    record_hit(b_right, index, 0u);
    move_sensor(index_angle(index));
}

/* One ping per fine angle. Only completed faults consume retries. */
static void
read_fine (ultrasonic_result_t measurement, uint16_t echo_us)
{
    bool b_right = state_get(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT) != 0u;
    uint8_t index = edge_index(b_right);
    obstacle_width_class_t position =
        obstacle_width_classify(measurement, echo_us, centre_echo_us);
    if (position == OBSTACLE_WIDTH_INVALID)
    {
        uint16_t retries = state_get(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT);
        if (retries < OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE)
        {
            state_set(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT, retries + 1u);
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            return;
        }
    }
    tm_printf((UB *)"[%lu ms]obstacle: fine %d deg %s %u us %s\n",
              (unsigned long)now_ms(), index_angle(index),
              measurement_name(measurement),
              (unsigned int)((measurement == ULTRASONIC_VALID) ? echo_us : 0u),
              (position == OBSTACLE_WIDTH_HIT)
                  ? "HIT"
                  : ((position == OBSTACLE_WIDTH_MISS) ? "MISS" : "INVALID"));
    if (position == OBSTACLE_WIDTH_INVALID)
    {
        obstacle_width_result_t width = obstacle_width_unavailable();
        finish_sweep(&width);
        return;
    }
    if (position == OBSTACLE_WIDTH_HIT)
    {
        record_hit(b_right, index, echo_us);
        next_fine_side(b_right);
        return;
    }
    probe_fine(b_right,
               b_right ? (uint8_t)(index - 1u) : (uint8_t)(index + 1u));
}

static void
record_hit (bool b_right, uint8_t index, uint16_t echo_us)
{
    uint8_t left = robot_obstacle_get_left_edge(edge_indices);
    uint8_t right = robot_obstacle_get_right_edge(edge_indices);
    if (b_right)
    {
        edge_indices = robot_obstacle_pack_edges(left, index);
        right_edge_echo_us = echo_us;
    }
    else
    {
        edge_indices = robot_obstacle_pack_edges(index, right);
        left_edge_echo_us = echo_us;
    }
}

/* The right walk runs first because the servo is already at 120 degrees. */
static void
next_fine_side (bool b_right)
{
    if (b_right)
    {
        start_fine_side(false,
                        (ultrasonic_result_t)state_get(SCAN_LEFT_RESULT_MASK,
                                                       SCAN_LEFT_RESULT_SHIFT),
                        left_echo_us);
    }
    else
    {
        finish_sweep(NULL);
    }
}

/* Publish direction and width together, once, then return to centre. */
static void
finish_sweep (obstacle_width_result_t const * p_width)
{
    obstacle_width_result_t width;
    if (p_width != NULL)
    {
        width = *p_width;
    }
    else
    {
        obstacle_width_edge_t left = {.index = edge_index(false),
                                      .echo_us = left_edge_echo_us};
        obstacle_width_edge_t right = {.index = edge_index(true),
                                       .echo_us = right_edge_echo_us};
        width = obstacle_width_estimate(left, right, &width_config);
    }
    uint32_t complete_ms = now_ms();
    /* Build the geometry before reset_sweep() clears the centre echo. */
    obstacle_scan_geometry_t geometry = sweep_geometry(&width);
    publish_result(robot_obstacle_get_status(pending_flags),
                   robot_obstacle_get_direction(pending_flags), &width,
                   &geometry);
    if (width.width_mm != ROBOT_OBSTACLE_WIDTH_UNAVAILABLE)
    {
        tm_printf((UB *)"[%lu ms]obstacle: width %u mm hits %d..%d bounds "
                        "%u.%u..%u.%u%s\n",
                  (unsigned long)complete_ms, (unsigned int)width.width_mm,
                  index_angle(robot_obstacle_get_left_edge(width.edges)),
                  index_angle(robot_obstacle_get_right_edge(width.edges)),
                  (unsigned int)(width.left_bound_ddeg / 10u),
                  (unsigned int)(width.left_bound_ddeg % 10u),
                  (unsigned int)(width.right_bound_ddeg / 10u),
                  (unsigned int)(width.right_bound_ddeg % 10u),
                  width.b_collapsed ? " collapsed" : "");
    }
    else
    {
        tm_printf((UB *)"[%lu ms]obstacle: width unavailable\n",
                  (unsigned long)complete_ms);
    }
    tm_printf((UB *)"[%lu ms]obstacle: side scan completed after %lu ms\n",
              (unsigned long)complete_ms,
              (unsigned long)(uint32_t)(complete_ms - side_scan_start_ms));
    state_flag(SCAN_SCANNING_SIDES, false);
    reset_sweep();
    move_sensor(CENTRE_ANGLE);
}

static uint8_t
edge_index (bool b_right)
{
    return b_right ? robot_obstacle_get_right_edge(edge_indices)
                   : robot_obstacle_get_left_edge(edge_indices);
}

static int
index_angle (uint8_t index)
{
    return (int)(OBSTACLE_SWEEP_FIRST_DEG + (OBSTACLE_SWEEP_STEP_DEG * index));
}

static char const *
measurement_name (ultrasonic_result_t measurement)
{
    switch (measurement)
    {
        case ULTRASONIC_VALID:
            return "VALID";
        case ULTRASONIC_NO_ECHO:
            return "NO_ECHO";
        case ULTRASONIC_FAULT:
            return "FAULT";
        case ULTRASONIC_PENDING:
        default:
            return "PENDING";
    }
}

static void
move_sensor (int angle)
{
    uint32_t travel_degrees = MAX_SERVO_TRAVEL_DEGREES;
    if (sensor_angle >= 0)
    {
        int delta = angle - sensor_angle;
        travel_degrees = (uint32_t)(delta < 0 ? -delta : delta);
    }
    uint32_t travel_ms =
        (travel_degrees * servo_timing.travel_ms_per_60_degrees + 59u) / 60u;
    reset_position();
    servo_set_angle(angle + OBSTACLE_SERVO_TRIM_DEG);
    sensor_angle = angle;
    settle_start_ms = now_ms();
    settle_duration_ms = travel_ms + servo_timing.settle_margin_ms;
    state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SERVO_SETTLING);
}

/* Only a finished sweep passes a width and geometry; NULL publishes the
 * width unavailable and the geometry as zeros. */
static void
publish_result (robot_obstacle_status_t status,
                robot_obstacle_direction_t direction,
                obstacle_width_result_t const * p_width,
                obstacle_scan_geometry_t const * p_geometry)
{
    obstacle_width_result_t width =
        (p_width != NULL) ? *p_width : obstacle_width_unavailable();
    result = (robot_obstacle_result_t){
        .flags = robot_obstacle_set_critical(
            robot_obstacle_pack_flags(status, direction, width.b_left_found,
                                      width.b_right_found),
            state_has(SCAN_CRITICAL)),
        .edges = width.edges,
        .width_mm = width.width_mm,
        .sequence = (uint8_t)(result.sequence + 1u)};
    published_geometry = (p_geometry != NULL)
                             ? *p_geometry
                             : (obstacle_scan_geometry_t){0u, 0, 0};
}

/* Centre range from the pivot, rounded and saturated; 0 if unusable. */
static obstacle_scan_geometry_t
sweep_geometry (obstacle_width_result_t const * p_width)
{
    obstacle_scan_geometry_t geometry = {.centre_mm = 0u,
                                         .left_x_mm = p_width->left_x_mm,
                                         .right_x_mm = p_width->right_x_mm};
    float range_mm;
    if (obstacle_width_range_mm(centre_echo_us, &width_config, &range_mm))
    {
        geometry.centre_mm = (range_mm >= ((float)UINT16_MAX - 0.5f))
                                 ? UINT16_MAX
                                 : (uint16_t)(range_mm + 0.5f);
    }
    return geometry;
}

/* Kernel time since boot in ms; wraps after 49 days, so compare by
   unsigned subtraction. Resolution is one kernel tick (10 ms). */
static uint32_t
now_ms (void)
{
    SYSTIM tim;
    (void)tk_get_otm(&tim);
    return (uint32_t)tim.lo;
}

static void
check_watchdogs (void)
{
    uint32_t now = now_ms();
    if (state_has(SCAN_SCANNING_SIDES))
    {
        if (obstacle_decision_elapsed_exceeds(now, side_scan_start_ms,
                                              OBSTACLE_SCAN_TIMEOUT_MS))
        {
            tm_printf(
                (UB *)"[%lu ms]obstacle: side scan timed out after %lu ms\n",
                (unsigned long)now,
                (unsigned long)(uint32_t)(now - side_scan_start_ms));
#if APP_TEST
            ++test_timeout_count;
            test_timeout_ms = now;
#endif
            cancel_side_scan();
        }
    }
    else if (state_has(SCAN_CORRIDOR))
    {
        if (obstacle_decision_elapsed_exceeds(now, corridor_start_ms,
                                              OBSTACLE_CORRIDOR_TIMEOUT_MS))
        {
            tm_printf((UB *)"[%lu ms]obstacle: corridor sweep timed out\n",
                      (unsigned long)now);
            finish_corridor(false);
            (void)obstacle_scan_rearm();
        }
    }
    else if (!b_stop_requested &&
             obstacle_decision_elapsed_exceeds(now, last_usable_centre_ms,
                                               OBSTACLE_FORWARD_MAX_AGE_MS))
    {
        tm_printf((UB *)"[%lu ms]obstacle: forward data stale (%lu ms old)\n",
                  (unsigned long)now,
                  (unsigned long)(now - last_usable_centre_ms));
#if APP_TEST
        ++test_stale_count;
        test_stale_ms = now;
#endif
        b_stop_requested = true;
        publish_result(ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE, NULL,
                       NULL);
        reset_position();
    }
}

/* Leave side scanning, keeping the stop request and published result. */
static void
stop_side_scan (void)
{
    state_flag(SCAN_SCANNING_SIDES, false);
    state_set(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT, 0u);
    state_set(SCAN_LEFT_RESULT_MASK, SCAN_LEFT_RESULT_SHIFT,
              ULTRASONIC_PENDING);
    left_echo_us = 0u;
    reset_sweep();
}

static void
cancel_side_scan (void)
{
    b_stop_requested = true;
    if (state_has(SCAN_CORRIDOR))
    {
        finish_corridor(false);
    }
    stop_side_scan();
    publish_result(ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE, NULL, NULL);
    move_sensor(CENTRE_ANGLE);
}

static void
update_critical (ultrasonic_result_t measurement, uint16_t echo_us)
{
    /* One critical reading sets the flag at once; no voting. */
    if (obstacle_decision_is_critical(measurement, echo_us) &&
        !state_has(SCAN_CRITICAL))
    {
        state_flag(SCAN_CRITICAL, true);
        /* Flag only, no new decision: keep the sequence unchanged. */
        result.flags = robot_obstacle_set_critical(result.flags, true);
#if APP_TEST
        test_critical_flag_us = (uint32_t)in_w(TIMER_TIMERAWL);
        ++test_critical_count;
        test_critical_receipt_us = test_receipt_us;
        test_critical_echo_us = echo_us;
        tm_printf((UB *)"obstacle test: critical event %lu echo %u us "
                        "receipt %lu us flag %lu us elapsed %lu us\n",
                  (unsigned long)test_critical_count, (unsigned int)echo_us,
                  (unsigned long)test_critical_receipt_us,
                  (unsigned long)test_critical_flag_us,
                  (unsigned long)(uint32_t)(test_critical_flag_us -
                                            test_critical_receipt_us));
#endif
    }
    /* Any reading inside the release distance holds it for this group. */
    if (obstacle_decision_blocks_critical_release(measurement, echo_us))
    {
        state_flag(SCAN_CRITICAL_HOLD, true);
    }
}

/* A faulted position stays invalid; NO_ECHO is a usable zero. */
static void
record_corridor (ultrasonic_result_t measurement, uint16_t echo_us)
{
    if (measurement != ULTRASONIC_FAULT)
    {
        corridor_work.valid_mask |= (uint16_t)(1u << corridor_index);
        corridor_work.echo_us[corridor_index] =
            (measurement == ULTRASONIC_VALID) ? echo_us : 0u;
    }
    ++corridor_index;
    if (corridor_index < OBSTACLE_SWEEP_POINTS)
    {
        move_sensor(index_angle(corridor_index));
        return;
    }
    finish_corridor(true);
    (void)obstacle_scan_rearm();
}

/* Publish the sweep; an incomplete one has no valid positions. */
static void
finish_corridor (bool b_complete)
{
    if (!b_complete)
    {
        corridor_work.valid_mask = 0u;
    }
    corridor_work.sequence = (uint8_t)(corridor.sequence + 1u);
    corridor = corridor_work;
    state_flag(SCAN_CORRIDOR, false);
    uint32_t done_ms = now_ms();
    tm_printf((UB *)"[%lu ms]obstacle: corridor sweep %s, valid 0x%04X "
                    "after %lu ms\n",
              (unsigned long)done_ms, b_complete ? "completed" : "cancelled",
              (unsigned int)corridor.valid_mask,
              (unsigned long)(uint32_t)(done_ms - corridor_start_ms));
}

#if APP_TEST
/* Scan task: replace a completed reading with the next scripted one. */
static void
script_take (ultrasonic_result_t * p_measurement, uint16_t * p_echo_us)
{
    if (tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }
    if (test_script_next < test_script_count)
    {
        *p_measurement = test_script[test_script_next].measurement;
        *p_echo_us = (*p_measurement == ULTRASONIC_VALID)
                         ? test_script[test_script_next].echo_us
                         : 0u;
        ++test_script_next;
    }
    (void)tk_unl_mtx(test_mutex);
}

/* Scan task: run a requested command, then start its script. */
static void
run_test_command (void)
{
    if (tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }
    bool b_pending = test_command_pending;
    obstacle_scan_test_command_t command = test_command;
    (void)tk_unl_mtx(test_mutex);
    if (!b_pending)
    {
        return;
    }
    uint8_t sequence = test_rearm_sequence;
    bool b_refused = false;
    if (command == OBSTACLE_SCAN_TEST_REARM)
    {
        sequence = obstacle_scan_rearm();
    }
    else
    {
        b_refused = !obstacle_scan_request_corridor();
    }
    if (tk_loc_mtx(test_mutex, TMO_FEVR) != E_OK)
    {
        return;
    }
    test_rearm_sequence = sequence;
    if (b_refused)
    {
        ++test_corridor_refused;
    }
    for (uint8_t index = 0u; index < test_pending_count; ++index)
    {
        test_script[index] = test_pending_script[index];
    }
    test_script_count = test_pending_count;
    test_script_next = 0u;
    test_command_pending = false;
    (void)tk_unl_mtx(test_mutex);
}
#endif
