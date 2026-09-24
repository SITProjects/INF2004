/**
 * @file obstacle.c
 * @brief Coordinate servo settling and fresh ultrasonic measurements.
 */
#include "obstacle.h"
#include "ultrasonic.h"
#include "servo.h"
#include "pico/time.h"
#include "hardware/gpio.h"

#define OBSTACLE_THRESHOLD_CM 5.0f
#define SERVO_SETTLE_MS 1000
#define CENTRE_ANGLE 90

typedef enum
{
    SCAN_SETTLING,
    SCAN_READY,
    SCAN_MEASURING
} scan_phase_t;

static const int scan_angles[] = {60, 120};
static scan_phase_t scan_phase;
static absolute_time_t settle_deadline;
static int side_index;
static bool scanning_sides;
static bool scan_had_invalid_echo;
static obstacle_scan_result_t result;

static void move_sensor (int angle);

obstacle_scan_result_t
obstacle_scan (void)
{
    if (scan_phase == SCAN_SETTLING)
    {
        if (!time_reached(settle_deadline))
        {
            return result;
        }
        scan_phase = SCAN_READY;
    }

    if (scan_phase == SCAN_READY)
    {
        if (ultrasonic_trigger())
        {
            scan_phase = SCAN_MEASURING;
        }
        return result;
    }

    float distance_cm;
    if (!ultrasonic_read(&distance_cm))
    {
        return result;
    }

    if (!scanning_sides)
    {
        if (distance_cm < 0.0f)
        {
            result = (obstacle_scan_result_t){OBSTACLE_UNKNOWN, -1};
            scan_phase = SCAN_READY;
        }
        else if (distance_cm >= OBSTACLE_THRESHOLD_CM)
        {
            result = (obstacle_scan_result_t){OBSTACLE_CLEAR, CENTRE_ANGLE};
            scan_phase = SCAN_READY;
        }
        else
        {
            result = (obstacle_scan_result_t){OBSTACLE_SCANNING, -1};
            scanning_sides = true;
            scan_had_invalid_echo = false;
            side_index = 0;
            move_sensor(scan_angles[side_index]);
        }
        return result;
    }

    if (distance_cm >= OBSTACLE_THRESHOLD_CM)
    {
        result = (obstacle_scan_result_t){OBSTACLE_PATH_FOUND,
                                          scan_angles[side_index]};
    }
    else
    {
        scan_had_invalid_echo |= distance_cm < 0.0f;
        ++side_index;
        if (side_index < (int)(sizeof(scan_angles) / sizeof(scan_angles[0])))
        {
            move_sensor(scan_angles[side_index]);
            return result;
        }
        result = (obstacle_scan_result_t){
            scan_had_invalid_echo ? OBSTACLE_UNKNOWN : OBSTACLE_BLOCKED, -1};
    }

    scanning_sides = false;
    move_sensor(CENTRE_ANGLE);
    return result;
}

int
obstacle_init (int trig_pin, int echo_pin, int servo_pin)
{
    /* Validate before either component configures hardware. */
    if (trig_pin < 0 || trig_pin >= (int)NUM_BANK0_GPIOS || echo_pin < 0 ||
        echo_pin >= (int)NUM_BANK0_GPIOS || servo_pin < 0 ||
        servo_pin >= (int)NUM_BANK0_GPIOS || trig_pin == echo_pin ||
        trig_pin == servo_pin || echo_pin == servo_pin)
    {
        return 1;
    }
    if (servo_init(servo_pin) || ultrasonic_init(trig_pin, echo_pin))
    {
        return 1;
    }

    scanning_sides = false;
    result = (obstacle_scan_result_t){OBSTACLE_SCANNING, -1};
    move_sensor(CENTRE_ANGLE);
    return 0;
}

static void
move_sensor (int angle)
{
    servo_set_angle(angle);
    settle_deadline = make_timeout_time_ms(SERVO_SETTLE_MS);
    scan_phase = SCAN_SETTLING;
}
