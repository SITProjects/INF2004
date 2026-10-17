/**
 * @file obstacle_scan.c
 * @brief Coordinate servo settling and fresh ultrasonic measurements.
 */
#include "obstacle_scan.h"
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "drv_ultrasonic.h"
#include "drv_servo.h"

/* Private types, constants, and macros. */
#define OBSTACLE_THRESHOLD_CM 5.0f
#define SERVO_SETTLE_MS 1000u
#define CENTRE_ANGLE 90

/* Bank-0 GPIOs on the RP2040. */
#define GPIO_COUNT 30

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
static int side_index;
static bool scanning_sides;
static bool scan_had_invalid_echo;
static obstacle_scan_result_t result;

/* Private function declarations. */
static void move_sensor (int angle);
static uint32_t now_ms (void);

/* Public function definitions. */

obstacle_scan_result_t
obstacle_scan_update (void)
{
    if (scan_phase == SCAN_SETTLING)
    {
        if ((uint32_t)(now_ms() - settle_start_ms) < SERVO_SETTLE_MS)
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
obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin)
{
    /* Validate before either component configures hardware. */
    if (trig_pin < 0 || trig_pin >= GPIO_COUNT || echo_pin < 0 ||
        echo_pin >= GPIO_COUNT || servo_pin < 0 || servo_pin >= GPIO_COUNT ||
        trig_pin == echo_pin || trig_pin == servo_pin || echo_pin == servo_pin)
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
