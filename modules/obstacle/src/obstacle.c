/**
 * @file obstacle.c
 * @brief Coordinate servo settling and fresh ultrasonic measurements.
 */

#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "shared.h"

/* Private types, constants, and macros. */

#define OBSTACLE_THRESHOLD_CM 5.0f
#define CENTRE_ANGLE 90
#define MAX_SERVO_TRAVEL_DEGREES 180u
#define ACCEPTABLE_RESULT_FROM_SIDES 1.3f
typedef enum
{
    SERVO_SETTLING,
    SCAN_READY,
    SCAN_MEASURING
} scan_phase_t;

/* Static data. */

/* Servo should scan these angles to the left and right of centre. */
static const int scan_angles[] = {60, 120};
static float scan_offset_cosines[sizeof(scan_angles) / sizeof(scan_angles[0])];
static obstacle_servo_timing_t servo_timing;
static int sensor_angle;
static scan_phase_t scan_phase;
static absolute_time_t servo_turning_deadline;
static size_t side_index;
static float front_distance_cm;
static float sampled_span_cm;
static uint8_t contributing_sides;
static obstacle_scan_result_t result;
static float farthest_clear_distance_cm;
static int path_found_angle;
static bool scanning_sides;
static bool scan_had_invalid_echo;

/* Private function declarations. */

static void scan_centre (float distance_cm);
static void scan_sides (float distance_cm);
static float calculate_sample_span (float centre_distance_cm,
                                    float side_distance_cm,
                                    float offset_cosine);
static void move_sensor (int angle);
static obstacle_scan_result_t read_result (void);
static void publish_result (obstacle_status_t status, float span_cm, int angle,
                            uint8_t side_count);

/* Public function definitions. */

int
obstacle_init (int trig_pin, int echo_pin, int servo_pin,
               const obstacle_servo_timing_t * timing)
{
    /* Validate before either component configures hardware. */
    if (trig_pin < 0 || trig_pin >= (int)NUM_BANK0_GPIOS || echo_pin < 0 ||
        echo_pin >= (int)NUM_BANK0_GPIOS || servo_pin < 0 ||
        servo_pin >= (int)NUM_BANK0_GPIOS || trig_pin == echo_pin ||
        trig_pin == servo_pin || echo_pin == servo_pin || timing == NULL ||
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
    for (size_t index = 0; index < sizeof(scan_angles) / sizeof(scan_angles[0]);
         ++index)
    {
        float offset_radians =
            (float)(scan_angles[index] - CENTRE_ANGLE) * DEG_TO_RAD;
        scan_offset_cosines[index] = cosf(offset_radians);
    }
    scanning_sides = false;
    result = (obstacle_scan_result_t){.status = OBSTACLE_SCANNING,
                                      .sampled_span_cm = -1.0f,
                                      .path_found_angle = -1};
    move_sensor(CENTRE_ANGLE);
    return 0;
}

obstacle_scan_result_t
obstacle_scan (void)
{
    switch (scan_phase)
    {
        case SERVO_SETTLING:
            if (!time_reached(servo_turning_deadline))
            {
                return result;
            }
            /* Servo has settled, start scanning. */
            scan_phase = SCAN_READY;
            return result;
        case SCAN_READY:
            /* Trigger ultrasonic measurement. */
            if (ultrasonic_trigger())
            {
                scan_phase = SCAN_MEASURING;
            }
            return result;
        case SCAN_MEASURING:
            return read_result();
        default:
            /* Recover with an unknown position and a fresh fault result. */
            scanning_sides = false;
            sensor_angle = -1;
            publish_result(OBSTACLE_UNKNOWN, -1.0f, -1, 0u);
            move_sensor(CENTRE_ANGLE);
            return result;
    }
}

/* Private function definitions. */

static obstacle_scan_result_t
read_result (void)
{
    float distance_cm;
    /* False means no completed measurement; invalid echoes return -1 cm. */
    if (!ultrasonic_read(&distance_cm))
    {
        return result;
    }
    if (scanning_sides)
    {
        scan_sides(distance_cm);
    }
    else
    {
        scan_centre(distance_cm);
    }
    return result;
}

static void
scan_centre (float distance_cm)
{
    if (distance_cm <= 0.0f)
    {
        publish_result(OBSTACLE_UNKNOWN, -1.0f, -1, 0u);
        scan_phase = SCAN_READY;
    }
    else if (distance_cm >= OBSTACLE_THRESHOLD_CM)
    {
        publish_result(OBSTACLE_CLEAR, -1.0f, CENTRE_ANGLE, 0u);
        scan_phase = SCAN_READY;
    }
    else
    {
        /* Save the centre reading before measuring both sides. */
        path_found_angle = -1;
        result = (obstacle_scan_result_t){.status = OBSTACLE_SCANNING,
                                          .sequence = result.sequence,
                                          .sampled_span_cm = -1.0f,
                                          .path_found_angle = -1};
        scanning_sides = true;
        front_distance_cm = distance_cm;
        sampled_span_cm = 0.0f;
        contributing_sides = 0u;
        scan_had_invalid_echo = false;
        side_index = 0;
        farthest_clear_distance_cm = -1.0f;
        move_sensor(scan_angles[side_index]);
    }
}

static void
scan_sides (float distance_cm)
{
    if (distance_cm <= 0.0f)
    {
        /* An invalid echo provides neither a span nor a clear direction. */
        scan_had_invalid_echo = true;
    }
    else if (distance_cm >= OBSTACLE_THRESHOLD_CM)
    {
        /* A clear side contributes zero span, even inside the filter. */
        if (distance_cm > farthest_clear_distance_cm)
        {
            farthest_clear_distance_cm = distance_cm;
            path_found_angle = scan_angles[side_index];
        }
    }
    else if (distance_cm <= front_distance_cm * ACCEPTABLE_RESULT_FROM_SIDES)
    {
        sampled_span_cm += calculate_sample_span(
            front_distance_cm, distance_cm, scan_offset_cosines[side_index]);
        ++contributing_sides;
    }
    ++side_index;
    if (side_index < sizeof(scan_angles) / sizeof(scan_angles[0]))
    {
        move_sensor(scan_angles[side_index]);
        return;
    }
    obstacle_status_t status = OBSTACLE_BLOCKED;
    if (path_found_angle != -1)
    {
        status = OBSTACLE_PATH_FOUND;
    }
    else if (scan_had_invalid_echo)
    {
        status = OBSTACLE_UNKNOWN;
    }
    publish_result(status, sampled_span_cm > 0.0f ? sampled_span_cm : -1.0f,
                   path_found_angle, contributing_sides);
    scanning_sides = false;
    move_sensor(CENTRE_ANGLE);
}

static float
calculate_sample_span (float centre_distance_cm, float side_distance_cm,
                       float offset_cosine)
{
    /* Law of cosines: chord between centre and side samples. */
    return sqrtf(side_distance_cm * side_distance_cm +
                 centre_distance_cm * centre_distance_cm -
                 2.0f * side_distance_cm * centre_distance_cm * offset_cosine);
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
    servo_set_angle(angle);
    sensor_angle = angle;
    servo_turning_deadline =
        make_timeout_time_ms(travel_ms + servo_timing.settle_margin_ms);
    scan_phase = SERVO_SETTLING;
}

static void
publish_result (obstacle_status_t status, float span_cm, int angle,
                uint8_t side_count)
{
    result = (obstacle_scan_result_t){.status = status,
                                      .sequence = result.sequence + 1u,
                                      .sampled_span_cm = span_cm,
                                      .contributing_sides = side_count,
                                      .path_found_angle = angle};
}
