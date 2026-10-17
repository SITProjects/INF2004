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

/* Private types, constants, and macros. */

/* obstacle_scan_update() never waits, so poll it once per kernel tick. */
#define SCAN_PERIOD_MS 10

/* Static data. */
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
                           OBSTACLE_SERVO_PIN) != 0)
    {
        tm_printf((UB *)"obstacle: failed to initialize obstacle scanning\n");
        tk_ext_tsk();
    }
    tm_printf((UB *)"obstacle: scanning started\n");

    /* An impossible angle, so the first result is always printed. */
    obstacle_scan_result_t last = {OBSTACLE_SCANNING, -2};
    for (;;)
    {
        obstacle_scan_result_t scan = obstacle_scan_update();
        if ((scan.status != last.status) || (scan.angle != last.angle))
        {
            tm_printf((UB *)"obstacle: %s, angle %d\n",
                      obstacle_scan_status_name(scan.status), scan.angle);
            last = scan;
        }

        switch (scan.status)
        {
            case OBSTACLE_CLEAR:
                /* TODO: Allow forward motion. */
                break;
            case OBSTACLE_PATH_FOUND:
                /* TODO: Turn toward scan.angle before moving forward. */
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
