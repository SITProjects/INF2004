/**
 * @file terrain.c
 * @brief Terrain subsystem: starts and runs the terrain task.
 *
 * The terrain task owns I2C0 (TEAM.md section 3). It sets up the GY-511,
 * retrying while the sensor is missing, then calls
 * terrain_monitor_update() every 10 ms and reports calibration changes.
 * With TERRAIN_DEBUG_PRINT_MS set it also prints pitch and roll.
 */

#include "terrain.h"
#include "terrain_config.h"
#include "terrain_monitor.h"
#include "terrain_platform.h"

/* Private types, constants, and macros. */
#define UPDATE_PERIOD_MS 10
#define INIT_RETRY_MS 1000
#define TENTHS_PER_UNIT 10

/* Static data. */
ID terrain_tid_monitor;

/* Private function declarations. */
static void monitor_task (INT stacd, void * p_exinf);
#if TERRAIN_DEBUG_PRINT_MS > 0
static void print_tilt (void);
static void print_angle (char const * p_name, float deg, bool b_held);
#endif

/* Public function definitions. */

ER
terrain_init (void)
{
    /* Before any task can call the terrain monitor. */
    terrain_platform_init();
    return E_OK;
}

ER
terrain_start (void)
{
    terrain_tid_monitor =
        app_start_task(monitor_task, TERRAIN_PRI_MONITOR, 0, "terrain_task");
    return (terrain_tid_monitor < E_OK) ? terrain_tid_monitor : E_OK;
}

/* Private function definitions. */

static void
monitor_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    /* Retry until the sensor answers; print each new reason once. */
    terrain_status_t status = terrain_monitor_init();
    terrain_status_t reported = TERRAIN_OK;
    while (status != TERRAIN_OK)
    {
        if (status != reported)
        {
            tm_printf((UB *)"terrain: setup failed: %s; retrying\n",
                      terrain_monitor_status_text(status));
            reported = status;
        }
        (void)tk_dly_tsk(INIT_RETRY_MS);
        status = terrain_monitor_init();
    }
    tm_printf((UB *)"terrain: IMU ready; calibrating, keep the car still "
                    "on level ground\n");

    terrain_calib_state_t last_state = TERRAIN_CALIB_CALIBRATING;
#if TERRAIN_DEBUG_PRINT_MS > 0
    INT since_print_ms = 0;
#endif
    for (;;)
    {
        (void)terrain_monitor_update();

        terrain_calib_info_t info;
        if ((terrain_monitor_get_calibration(&info) == TERRAIN_OK) &&
            (info.state != last_state))
        {
            tm_printf((UB *)"terrain: calibration %s\n",
                      terrain_monitor_calib_text(info.state));
            last_state = info.state;
        }

#if TERRAIN_DEBUG_PRINT_MS > 0
        since_print_ms += UPDATE_PERIOD_MS;
        if (since_print_ms >= TERRAIN_DEBUG_PRINT_MS)
        {
            since_print_ms = 0;
            print_tilt();
        }
#endif
        (void)tk_dly_tsk(UPDATE_PERIOD_MS);
    }
}

#if TERRAIN_DEBUG_PRINT_MS > 0
/* One line of pitch and roll, once calibrated. */
static void
print_tilt (void)
{
    terrain_data_t data;
    if (terrain_monitor_get_latest(&data) != TERRAIN_OK)
    {
        return;
    }

    tm_printf((UB *)"terrain:");
    print_angle("pitch", data.pitch_deg, data.b_pitch_held);
    print_angle("roll", data.roll_deg, data.b_roll_held);
    tm_printf((UB *)"\n");
}

/* tm_printf has no %f: print the angle as whole degrees and tenths. */
static void
print_angle (char const * p_name, float deg, bool b_held)
{
    float const rounding = (deg < 0.0f) ? -0.5f : 0.5f;
    INT tenths = (INT)((deg * (float)TENTHS_PER_UNIT) + rounding);
    char const * p_sign = "";
    if (tenths < 0)
    {
        p_sign = "-";
        tenths = -tenths;
    }

    tm_printf((UB *)" %s %s%d.%d%s", p_name, p_sign, tenths / TENTHS_PER_UNIT,
              tenths % TENTHS_PER_UNIT, b_held ? " (held)" : "");
}
#endif
