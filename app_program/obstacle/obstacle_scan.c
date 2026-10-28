/**
 * @file obstacle_scan.c
 * @brief Coordinate servo settling and fresh ultrasonic measurements.
 */

#include "obstacle_scan.h"
#include "obstacle_config.h"
#include "obstacle_decision.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
/* Use the toolchain size_t alongside the kernel headers. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>
#include <stdbool.h>

/* Private types, constants, and macros. */

#define GPIO_COUNT 30
#define CENTRE_ANGLE ((int)OBSTACLE_CENTRE_DEG)
#define MAX_SERVO_TRAVEL_DEGREES 180u

/* Packed scan_state: phase 0..1, count 2..3, side 4, scanning 5,
 * retries 6..7, invalid position 8, left side result 9..10.
 * Bits 11..15 are unused.
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

_Static_assert(OBSTACLE_PINGS_PER_ANGLE >= 1u && OBSTACLE_PINGS_PER_ANGLE <= 3u,
               "Ping count must fit two bits and the small median helper");
_Static_assert(ULTRASONIC_FAULT <= 3u,
               "Ultrasonic results must fit the two-bit left result field");
_Static_assert(OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE <= 3u,
               "Extra attempts must fit the two-bit retry field");

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

/* Private function declarations. */

static uint16_t state_get (uint16_t mask, uint8_t shift);
static void state_set (uint16_t mask, uint8_t shift, uint16_t value);
static bool state_has (uint16_t mask);
static void state_flag (uint16_t mask, bool enabled);
static void reset_position (void);
static void read_centre (ultrasonic_result_t measurement, uint16_t echo_us);
static void request_stop (void);
static void scan_centre (obstacle_centre_action_t action);
static void scan_sides (ultrasonic_result_t measurement, uint16_t echo_us);
static void move_sensor (int angle);
static uint32_t now_ms (void);
static robot_obstacle_result_t read_result (void);
static void publish_result (robot_obstacle_status_t status,
                            robot_obstacle_direction_t direction);

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
    result = (robot_obstacle_result_t){
        .flags = robot_obstacle_pack_flags(
            ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, false, false),
        .edges = robot_obstacle_pack_edges(ROBOT_OBSTACLE_EDGE_NONE,
                                           ROBOT_OBSTACLE_EDGE_NONE),
        .width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE,
        .sequence = 0u};
    move_sensor(CENTRE_ANGLE);
    return 0;
}

robot_obstacle_result_t
obstacle_scan_update (void)
{
    switch ((scan_phase_t)state_get(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT))
    {
        case SERVO_SETTLING:
            if ((uint32_t)(now_ms() - settle_start_ms) < settle_duration_ms)
            {
                return result;
            }
            /* Servo has settled, start scanning. */
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            return result;
        case SCAN_READY:
        {
            bool b_centre = !state_has(SCAN_SCANNING_SIDES);
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
            /* Recover with an unknown position and a fresh fault result. */
            scan_state = 0u;
            b_stop_requested = true;
            sensor_angle = -1;
            publish_result(ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE);
            move_sensor(CENTRE_ANGLE);
            return result;
    }
}

bool
obstacle_scan_stop_requested (void)
{
    return b_stop_requested;
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
    centre_near_count = 0u;
    for (size_t index = 0u; index < OBSTACLE_PINGS_PER_ANGLE; ++index)
    {
        pings_us[index] = 0u;
    }
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
    if (!state_has(SCAN_SCANNING_SIDES))
    {
        read_centre(measurement, echo_us);
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
    scan_sides(measurement, echo_us);
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
        request_stop();
        uint16_t retries = state_get(SCAN_RETRY_MASK, SCAN_RETRY_SHIFT);
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

    if (action == OBSTACLE_CENTRE_SCAN_SIDES)
    {
        request_stop();
        ++centre_near_count;
    }
    if (!b_stop_requested)
    {
        /* While already clear, each individual clear ping refreshes status. */
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
    scan_centre((centre_near_count >= OBSTACLE_CENTRE_NEAR_VOTES)
                    ? OBSTACLE_CENTRE_SCAN_SIDES
                    : OBSTACLE_CENTRE_CLEAR);
}

static void
request_stop (void)
{
    if (!b_stop_requested)
    {
        b_stop_requested = true;
        publish_result(ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE);
    }
}

static void
scan_centre (obstacle_centre_action_t action)
{
    switch (action)
    {
        case OBSTACLE_CENTRE_CLEAR:
            b_stop_requested = false;
            publish_result(ROBOT_OBSTACLE_CLEAR, ROBOT_OBSTACLE_DIR_CENTRE_90);
            reset_position();
            state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SCAN_READY);
            break;
        case OBSTACLE_CENTRE_SCAN_SIDES:
            /* Retain the sequence while both sides are being sampled. */
            result.flags = robot_obstacle_pack_flags(
                ROBOT_OBSTACLE_SCANNING, ROBOT_OBSTACLE_DIR_NONE, false, false);
            state_flag(SCAN_SCANNING_SIDES, true);
            state_set(SCAN_SIDE_MASK, SCAN_SIDE_SHIFT, 0u);
            state_set(SCAN_LEFT_RESULT_MASK, SCAN_LEFT_RESULT_SHIFT,
                      ULTRASONIC_PENDING);
            left_echo_us = 0u;
            move_sensor(scan_angles[0]);
            break;
        case OBSTACLE_CENTRE_UNKNOWN:
        default:
            b_stop_requested = true;
            publish_result(ROBOT_OBSTACLE_UNKNOWN, ROBOT_OBSTACLE_DIR_NONE);
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

    /* Publish only after both positions, even if the first side was clear. */
    obstacle_side_decision_t decision = obstacle_decision_sides(
        (ultrasonic_result_t)state_get(SCAN_LEFT_RESULT_MASK,
                                       SCAN_LEFT_RESULT_SHIFT),
        left_echo_us, measurement, echo_us);
    publish_result(decision.status, decision.direction);
    state_flag(SCAN_SCANNING_SIDES, false);
    move_sensor(CENTRE_ANGLE);
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
    servo_set_angle(angle);
    sensor_angle = angle;
    settle_start_ms = now_ms();
    settle_duration_ms = travel_ms + servo_timing.settle_margin_ms;
    state_set(SCAN_PHASE_MASK, SCAN_PHASE_SHIFT, SERVO_SETTLING);
}

static void
publish_result (robot_obstacle_status_t status,
                robot_obstacle_direction_t direction)
{
    result = (robot_obstacle_result_t){
        .flags = robot_obstacle_pack_flags(status, direction, false, false),
        .edges = robot_obstacle_pack_edges(ROBOT_OBSTACLE_EDGE_NONE,
                                           ROBOT_OBSTACLE_EDGE_NONE),
        .width_mm = ROBOT_OBSTACLE_WIDTH_UNAVAILABLE,
        .sequence = (uint8_t)(result.sequence + 1u)};
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
