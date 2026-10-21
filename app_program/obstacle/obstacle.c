/**
 * @file obstacle.c
 * @brief Obstacle subsystem: starts and runs the scan task.
 *
 * The scan task owns the ultrasonic sensor, the servo's PWM slice and the
 * GPIO bank interrupt (TEAM.md section 3). It runs on core 0 because the
 * echo interrupt is enabled in core 0's interrupt controller.
 */

#include "obstacle.h"
#include "obstacle_scan.h"
#include <stdbool.h>

/* Private types, constants, and macros. */

/* obstacle_scan_update() never waits, so poll it once per kernel tick. */
#define SCAN_PERIOD_MS 10

/* Static data. */

/* Preserve the one-second wait until travel and settling are measured. */
static const obstacle_servo_timing_t servo_timing = {
    .travel_ms_per_60_degrees = 0u, .settle_margin_ms = 1000u};

ID obstacle_tid_scan;

/* Private function declarations. */
static void scan_task (INT stacd, void * p_exinf);

/* Public function definitions. */

ER
obstacle_init (void)
{
    /* No kernel objects; the scan task sets up its own hardware. */
    return E_OK;
}

ER
obstacle_start (void)
{
    obstacle_tid_scan = app_start_task(scan_task, OBSTACLE_PRI_SCAN, TP_PRC1,
                                       "obstacle_scan_task");
    return (obstacle_tid_scan < E_OK) ? obstacle_tid_scan : E_OK;
}

/* Private function definitions. */

static void
scan_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    if (obstacle_scan_init(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN,
                           OBSTACLE_SERVO_PIN, &servo_timing) != 0)
    {
        tm_printf((UB *)"obstacle: failed to initialize obstacle scanning\n");
        tk_ext_tsk();
    }
    tm_printf((UB *)"obstacle: scanning started\n");

    uint32_t last_sequence = 0u;
    for (;;)
    {
        obstacle_scan_result_t scan = obstacle_scan_update();
        bool new_result = scan.sequence != last_sequence;
        last_sequence = scan.sequence;
        if (new_result)
        {
            tm_printf((UB *)"obstacle: %s, angle %d\n",
                      obstacle_scan_status_name(scan.status),
                      scan.path_found_angle);
            if (scan.sampled_span_cm >= 0.0f)
            {
                /* The kernel console does not support floating-point output. */
                int span_mm = (int)(scan.sampled_span_cm * 10.0f + 0.5f);
                tm_printf((UB *)"obstacle: sampled span %d mm (%u sides)\n",
                          span_mm, (unsigned int)scan.contributing_sides);
            }
            else
            {
                tm_printf((UB *)"obstacle: sampled span unavailable\n");
            }
        }

        switch (scan.status)
        {
            case OBSTACLE_CLEAR:
                /* TODO: Allow forward motion. */
                break;
            case OBSTACLE_PATH_FOUND:
                if (!new_result)
                {
                    break;
                }
                /* TODO: Start a turn toward scan.path_found_angle once. */
                break;
            case OBSTACLE_SCANNING:
            case OBSTACLE_BLOCKED:
            case OBSTACLE_UNKNOWN:
            default:
                /* TODO: Keep motion stopped until a direction is available. */
                break;
        }
        (void)tk_dly_tsk(SCAN_PERIOD_MS);
    }
}
