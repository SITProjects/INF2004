/**
 * @file obstacle_scan.c
 * @brief Coordinate servo settling and fresh ultrasonic measurements.
 */

#include "obstacle_scan.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include <stdint.h>
#include <tk/tkernel.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Private types, constants, and macros. */

#define OBSTACLE_THRESHOLD_CM 5.0f
#define SERVO_SETTLE_MS 1000u
#define CENTRE_ANGLE 90
#define DEGREES_TO_RADIANS (3.14159265358979323846f / 180.0f)

typedef enum
{
    SCAN_SETTLING,
    SCAN_READY,
    SCAN_MEASURING
} scan_phase_t;

/* Static data. */

static const int scan_angles[] = {60, 120};
static scan_phase_t scan_phase;
static uint32_t settle_start_ms;
static size_t side_index;
static int clear_side_angle;
static bool scanning_sides;
static bool scan_had_invalid_echo;
static float front_distance_cm;
static float obstacle_width_cm;
static obstacle_scan_result_t result;

/* Private function declarations. */

static void process_centre_measurement (float distance_cm);
static void scan_sides (float distance_cm);
static float calculate_sample_span (float centre_distance_cm,
                                    float side_distance_cm,
                                    float offset_degrees);
static void move_sensor (int angle);
static uint32_t now_ms (void);

/* Public function definitions. */

obstacle_scan_result_t
obstacle_scan_update (void)
{
    switch (scan_phase)
    {
        case SCAN_SETTLING:
            if ((uint32_t)(now_ms() - settle_start_ms) < SERVO_SETTLE_MS)
            {
                return result;
            }
            scan_phase = SCAN_READY;
            return result;
        case SCAN_READY:
            if (ultrasonic_trigger())
            {
                scan_phase = SCAN_MEASURING;
            }
            return result;
        case SCAN_MEASURING:
            break;
        default:
            scanning_sides = false;
            result = (obstacle_scan_result_t){OBSTACLE_UNKNOWN, -1, -1.0f};
            move_sensor(CENTRE_ANGLE);
            return result;
    }

    float distance_cm;
    if (!ultrasonic_read(&distance_cm))
    {
        return result;
    }

    // Scan centre then sides
    if (scanning_sides)
    {
        scan_sides(distance_cm);
    }
    else
    {
        process_centre_measurement(distance_cm);
    }

    return result;
}

int
obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin)
{
    /* Validate before either component configures hardware. */
    if (trig_pin < 0 || trig_pin >= 30 || echo_pin < 0 ||
        echo_pin >= 30 || servo_pin < 0 ||
        servo_pin >= 30 || trig_pin == echo_pin ||
        trig_pin == servo_pin || echo_pin == servo_pin)
    {
        return 1;
    }
    if (servo_init(servo_pin) || ultrasonic_init(trig_pin, echo_pin))
    {
        return 1;
    }

    scanning_sides = false;
    result = (obstacle_scan_result_t){OBSTACLE_SCANNING, -1, -1.0f};
    move_sensor(CENTRE_ANGLE);
    return 0;
}

char const *
obstacle_scan_status_name (obstacle_status_t status)
{
    switch (status)
    {
        case OBSTACLE_SCANNING:
            return "SCANNING";
        case OBSTACLE_CLEAR:
            return "CLEAR";
        case OBSTACLE_PATH_FOUND:
            return "PATH_FOUND";
        case OBSTACLE_BLOCKED:
            return "BLOCKED";
        case OBSTACLE_UNKNOWN:
            return "UNKNOWN";
        default:
            return "INVALID";
    }
}

/* Private function definitions. */

static void
process_centre_measurement (float distance_cm)
{
    if (distance_cm < 0.0f)
    {
        result = (obstacle_scan_result_t){OBSTACLE_UNKNOWN, -1, -1.0f};
        scan_phase = SCAN_READY;
    }
    else if (distance_cm >= OBSTACLE_THRESHOLD_CM)
    {
        result = (obstacle_scan_result_t){OBSTACLE_CLEAR, CENTRE_ANGLE, -1.0f};
        scan_phase = SCAN_READY;
    }
    else
    {
        /* Save the centre reading before measuring both sides. */
        result = (obstacle_scan_result_t){OBSTACLE_SCANNING, -1, -1.0f};
        scanning_sides = true;
        scan_had_invalid_echo = false;
        front_distance_cm = distance_cm;
        obstacle_width_cm = 0.0f;
        side_index = 0;
        clear_side_angle = -1;
        move_sensor(scan_angles[side_index]);
    }
}

static void
scan_sides (float distance_cm)
{
    if (distance_cm < 0.0f)
    {
        scan_had_invalid_echo = true;
    }
    else
    {
        if (distance_cm >= OBSTACLE_THRESHOLD_CM && clear_side_angle < 0)
        {
            clear_side_angle = scan_angles[side_index];
        }
        float offset_degrees =
            fabsf((float)(scan_angles[side_index] - CENTRE_ANGLE));
        obstacle_width_cm += calculate_sample_span(front_distance_cm,
                                                   distance_cm, offset_degrees);
    }

    ++side_index;
    if (side_index < sizeof(scan_angles) / sizeof(scan_angles[0]))
    {
        move_sensor(scan_angles[side_index]);
        return;
    }

    /* Publish a result only after both side measurements have completed. */
    if (clear_side_angle >= 0)
    {
        result = (obstacle_scan_result_t){OBSTACLE_PATH_FOUND, clear_side_angle,
                                          -1.0f};
    }
    else
    {
        result = (obstacle_scan_result_t){
            scan_had_invalid_echo ? OBSTACLE_UNKNOWN : OBSTACLE_BLOCKED, -1,
            -1.0f};
    }
    result.width_cm = scan_had_invalid_echo ? -1.0f : obstacle_width_cm;

    scanning_sides = false;
    move_sensor(CENTRE_ANGLE);
}

static float
calculate_sample_span (float centre_distance_cm, float side_distance_cm,
                       float offset_degrees)
{
    /* Law of cosines: chord between centre and side samples. */
    float angle_radians = offset_degrees * DEGREES_TO_RADIANS;
    return sqrtf(side_distance_cm * side_distance_cm +
                 centre_distance_cm * centre_distance_cm -
                 2.0f * side_distance_cm * centre_distance_cm *
                     cosf(angle_radians));
}

static void
move_sensor (int angle)
{
    servo_set_angle(angle);
    settle_start_ms = now_ms();
    scan_phase = SCAN_SETTLING;
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
