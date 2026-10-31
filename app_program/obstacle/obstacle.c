/**
 * @file obstacle.c
 * @brief Obstacle subsystem: starts and runs the scan task.
 *
 * The scan task owns the ultrasonic sensor, the servo's PWM slice and the
 * GPIO bank interrupt (TEAM.md section 3). It runs on core 0 because the
 * echo interrupt is enabled in core 0's interrupt controller.
 */

#include "obstacle.h"
#include "obstacle_calibration.h"
#include "obstacle_scan.h"
#include <stdbool.h>

/* Private types, constants, and macros. */

/* Static data. */

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
/* Use configured travel and settling values; calibration remains pending. */
static const obstacle_servo_timing_t servo_timing = {
    .travel_ms_per_60_degrees = OBSTACLE_SERVO_TRAVEL_MS_PER_60_DEG,
    .settle_margin_ms = OBSTACLE_SERVO_SETTLE_MARGIN_MS};
#endif

ID obstacle_tid_scan;

/* Private function declarations. */
static void scan_task (INT stacd, void * p_exinf);

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
static int direction_degrees (robot_obstacle_direction_t direction);
static uint32_t log_time_ms (void);

#endif

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

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
static uint32_t
log_time_ms (void)
{
    SYSTIM tim;
    (void)tk_get_otm(&tim);
    return (uint32_t)tim.lo;
}
#endif

static void
scan_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

#if OBSTACLE_CALIBRATION_MODE != OBSTACLE_CAL_MODE_OFF
    obstacle_calibration_run(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN,
                             OBSTACLE_SERVO_PIN);
#else
    if (obstacle_scan_init(OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN,
                           OBSTACLE_SERVO_PIN, &servo_timing) != 0)
    {
        tm_printf(
            (UB *)"[%lu ms]obstacle: failed to initialize obstacle scanning\n",
            log_time_ms());
        tk_ext_tsk();
    }
    tm_printf((UB *)"[%lu ms]obstacle: scanning started\n", log_time_ms());

    uint8_t last_sequence = 0u;
    bool b_last_stop_requested = obstacle_scan_stop_requested();
    bool b_last_critical = false;
    tm_printf(
        (UB *)"[%lu ms]obstacle: stop request ACTIVE (sensing output only)\n",
        log_time_ms());
    for (;;)
    {
        robot_obstacle_result_t scan = obstacle_scan_update();
        bool b_stop = obstacle_scan_stop_requested();
        bool b_critical = robot_obstacle_get_critical(scan.flags);
        if (b_critical != b_last_critical)
        {
            tm_printf((UB *)"[%lu ms]obstacle: CRITICAL %s\n", log_time_ms(),
                      b_critical ? "set" : "cleared");
            b_last_critical = b_critical;
        }
        if (b_stop != b_last_stop_requested)
        {
            tm_printf((UB *)"[%lu ms]obstacle: stop request %s\n",
                      log_time_ms(), b_stop ? "ACTIVE" : "RELEASED");
            b_last_stop_requested = b_stop;
        }
        /* Integration: send the stop request to the motor owner here. */
        robot_obstacle_status_t status = robot_obstacle_get_status(scan.flags);
        robot_obstacle_direction_t direction =
            robot_obstacle_get_direction(scan.flags);
        bool new_result = scan.sequence != last_sequence;
        last_sequence = scan.sequence;
        if (new_result)
        {
            tm_printf((UB *)"[%lu ms]obstacle: [0x%02X] %s dir %d\n",
                      log_time_ms(), (unsigned int)scan.flags,
                      obstacle_scan_status_name(status),
                      direction_degrees(direction));
        }

        (void)tk_dly_tsk(OBSTACLE_UPDATE_PERIOD_MS);
    }
#endif
}

#if OBSTACLE_CALIBRATION_MODE == OBSTACLE_CAL_MODE_OFF
/* Convert the packed direction code only for logging, without geometry. */
static int
direction_degrees (robot_obstacle_direction_t direction)
{
    switch (direction)
    {
        case ROBOT_OBSTACLE_DIR_LEFT_60:
            return (int)OBSTACLE_SWEEP_FIRST_DEG;
        case ROBOT_OBSTACLE_DIR_CENTRE_90:
            return (int)OBSTACLE_CENTRE_DEG;
        case ROBOT_OBSTACLE_DIR_RIGHT_120:
            return (int)OBSTACLE_SWEEP_LAST_DEG;
        case ROBOT_OBSTACLE_DIR_NONE:
        default:
            return -1;
    }
}

#endif
