/**
 * @file terrain.c
 * @brief Terrain subsystem: starts and runs the terrain task.
 *
 * The terrain task owns I2C0 (TEAM.md section 3). It sets up the GY-511,
 * retrying while the sensor is missing, then calls
 * terrain_monitor_update() every 10 ms and reports calibration and IMU
 * fault changes. It is the one reader of the event queue: it prints every
 * hump event as it happens (the LEVEL line carries the hump's height) and
 * any events dropped from a full queue.
 * With TERRAIN_DEBUG_PRINT_MS set it also prints pitch, roll and the
 * hump state, or the calibration progress until calibrated.
 */

#include "terrain.h"
#include "terrain_config.h"
#include "terrain_fault.h"
#include "terrain_monitor.h"
#include "terrain_platform.h"

/* Private types, constants, and macros. */
#define UPDATE_PERIOD_MS 10
#define INIT_RETRY_MS 1000
#define TENTHS_PER_UNIT 10
#define US_PER_MS 1000u
#define HUNDREDTHS_PER_UNIT 100

/* Static data. */
ID terrain_tid_monitor;

/* Private function declarations. */
static void monitor_task (INT stacd, void * p_exinf);
static void print_events (void);
static void print_event (terrain_event_t const * p_event);
#if TERRAIN_DEBUG_PRINT_MS > 0
static void print_status (void);
static void print_calibrating (terrain_data_t const * p_data);
static void print_angle (char const * p_name, int16_t ddeg, bool b_held);
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
    bool b_last_fault = false;
    uint8_t last_dropped = 0u;
#if TERRAIN_DEBUG_PRINT_MS > 0
    /* Real time, not loop passes: a pass can take longer than 10 ms. */
    uint64_t last_print_us = terrain_platform_time_us();
#endif
    for (;;)
    {
        (void)terrain_monitor_update();

        terrain_calib_info_t info;
        if (terrain_monitor_get_calibration(&info) == TERRAIN_OK)
        {
            terrain_calib_state_t const state =
                terrain_monitor_calib_state(&info);
            if (state != last_state)
            {
                tm_printf((UB *)"terrain: calibration %s\n",
                          terrain_monitor_calib_text(state));
                last_state = state;
            }
        }

        bool const b_fault = terrain_fault_is_active();
        if (b_fault != b_last_fault)
        {
            tm_printf((UB *)"terrain: IMU fault %s\n",
                      b_fault ? "raised" : "cleared");
            b_last_fault = b_fault;
        }

        print_events();
        uint8_t const dropped = terrain_monitor_events_dropped();
        if (dropped != last_dropped)
        {
            tm_printf((UB *)"terrain: events dropped %d (queue full)\n",
                      (INT)dropped);
            last_dropped = dropped;
        }

#if TERRAIN_DEBUG_PRINT_MS > 0
        uint64_t const now_us = terrain_platform_time_us();
        if ((now_us - last_print_us) >=
            ((uint64_t)TERRAIN_DEBUG_PRINT_MS * US_PER_MS))
        {
            last_print_us = now_us;
            print_status();
        }
#endif
        (void)tk_dly_tsk(UPDATE_PERIOD_MS);
    }
}

/* Print and remove every queued event; the queue is filled only by
 * terrain_monitor_update() in this task, so it cannot grow meanwhile. */
static void
print_events (void)
{
    terrain_event_t event = {0u};
    for (;;)
    {
        if (terrain_monitor_poll_event(&event) != TERRAIN_OK)
        {
            return;
        }
        if (terrain_monitor_event_type(&event) == TERRAIN_EVENT_NONE)
        {
            return;
        }
        print_event(&event);
    }
}

/*
 * One line per event, with its time since boot in seconds (comes round
 * every 655.36 s). ASCENDING and DESCENDING carry the pitch then, LEVEL
 * the hump's height.
 */
static void
print_event (terrain_event_t const * p_event)
{
    terrain_event_type_t const type = terrain_monitor_event_type(p_event);
    INT const value = (INT)terrain_monitor_event_value(p_event);
    INT const time_10ms = (INT)terrain_monitor_event_time_10ms(p_event);

    tm_printf((UB *)"terrain: hump %s", terrain_monitor_event_text(type));
    if (type == TERRAIN_EVENT_LEVEL)
    {
        tm_printf((UB *)" height %d mm", value);
    }
    else
    {
        INT tenths = value;
        char const * p_sign = "";
        if (tenths < 0)
        {
            p_sign = "-";
            tenths = -tenths;
        }
        tm_printf((UB *)" pitch %s%d.%d deg", p_sign, tenths / TENTHS_PER_UNIT,
                  tenths % TENTHS_PER_UNIT);
    }
    tm_printf((UB *)" t %d.%02d s\n", time_10ms / HUNDREDTHS_PER_UNIT,
              time_10ms % HUNDREDTHS_PER_UNIT);
}

#if TERRAIN_DEBUG_PRINT_MS > 0
/* One line of pitch, roll, hump state and highest hump; calibration
 * progress before. */
static void
print_status (void)
{
    terrain_data_t data = {0};
    terrain_status_t const status = terrain_monitor_get_latest(&data);
    if (status == TERRAIN_ERR_NOT_CALIBRATED)
    {
        print_calibrating(&data);
        return;
    }
    if (status != TERRAIN_OK)
    {
        return;
    }

    tm_printf((UB *)"terrain:");
    print_angle("pitch", terrain_monitor_data_pitch_ddeg(&data),
                terrain_monitor_data_is_pitch_held(&data));
    print_angle("roll", terrain_monitor_data_roll_ddeg(&data),
                terrain_monitor_data_is_roll_held(&data));
    tm_printf((UB *)" hump %s highest %d mm\n",
              terrain_monitor_hump_text(terrain_monitor_data_hump_state(&data)),
              (INT)terrain_monitor_data_highest_hump_mm(&data));
}

/*
 * Why calibration has not finished: samples not arriving (the count stands
 * still), or attempts rejected as moved (retries go up with the spread).
 */
static void
print_calibrating (terrain_data_t const * p_data)
{
    terrain_calib_info_t info;
    if (terrain_monitor_get_calibration(&info) != TERRAIN_OK)
    {
        return;
    }

    tm_printf((UB *)"terrain: %s samples %d attempts %d retries %d "
                    "spread %d mg tilt %d.%d deg\n",
              terrain_monitor_calib_text(terrain_monitor_calib_state(&info)),
              (INT)terrain_monitor_data_sample_count(p_data),
              (INT)terrain_monitor_calib_attempts(&info),
              (INT)terrain_monitor_calib_retries(&info),
              (INT)terrain_monitor_calib_spread_mg(&info),
              (INT)terrain_monitor_calib_tilt_ddeg(&info) / TENTHS_PER_UNIT,
              (INT)terrain_monitor_calib_tilt_ddeg(&info) % TENTHS_PER_UNIT);
}

/* tm_printf has no %f: print tenths as whole degrees and tenths. */
static void
print_angle (char const * p_name, int16_t ddeg, bool b_held)
{
    INT tenths = (INT)ddeg;
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
