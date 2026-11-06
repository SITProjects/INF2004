/**
 * @file terrain.c
 * @brief Terrain subsystem: starts and runs the terrain task.
 *
 * The terrain task owns I2C0 (TEAM.md section 3). It sets up the GY-511,
 * retrying while the sensor is missing, then calls
 * terrain_monitor_update() every 10 ms and reports calibration and IMU
 * fault changes. It is the one reader of the event queue: it prints every
 * hump, collision, turning and speed event as it happens (the LEVEL line
 * carries the hump's height, the COLLISION line the push) and any events
 * dropped from a full queue. After a LEVEL, COLLISION or TURNING event it
 * prints the run summary and its packed message.
 * With TERRAIN_DEBUG_PRINT_MS set it also prints pitch, roll, the hump
 * state, the turn rate and the speed state, or the calibration progress
 * until calibrated.
 * It also reads the magnetometer (terrain_compass.h) on every update and,
 * with TERRAIN_COMPASS_PRINT_MS set, prints it for the calibration spin.
 */

#include "terrain.h"
#include "terrain_compass.h"
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
static bool print_events (void);
static void print_event (terrain_event_t const * p_event);
static void print_summary (void);
#if TERRAIN_COMPASS_PRINT_MS > 0
static void print_compass (void);
#endif
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
    terrain_status_t const compass = terrain_compass_init();
    tm_printf((UB *)"terrain: compass %s\n",
              (TERRAIN_OK == compass) ? "ready"
                                      : terrain_monitor_status_text(compass));

    terrain_calib_state_t last_state = TERRAIN_CALIB_CALIBRATING;
    bool b_last_fault = false;
    uint8_t last_dropped = 0u;
#if TERRAIN_DEBUG_PRINT_MS > 0
    /* Real time, not loop passes: a pass can take longer than 10 ms. */
    uint64_t last_print_us = terrain_platform_time_us();
#endif
#if TERRAIN_COMPASS_PRINT_MS > 0
    uint64_t last_compass_us = terrain_platform_time_us();
#endif
    for (;;)
    {
        (void)terrain_monitor_update();
        terrain_compass_update();

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

        if (print_events())
        {
            print_summary();
        }
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
#if TERRAIN_COMPASS_PRINT_MS > 0
        uint64_t const compass_us = terrain_platform_time_us();
        if ((compass_us - last_compass_us) >=
            ((uint64_t)TERRAIN_COMPASS_PRINT_MS * US_PER_MS))
        {
            last_compass_us = compass_us;
            print_compass();
        }
#endif
        (void)tk_dly_tsk(UPDATE_PERIOD_MS);
    }
}

/*
 * Print and remove every queued event; the queue is filled only by
 * terrain_monitor_update() in this task, so it cannot grow meanwhile.
 * Returns true if one of them is counted in the run summary.
 */
static bool
print_events (void)
{
    bool b_counted = false;
    terrain_event_t event = {0u};
    for (;;)
    {
        if (terrain_monitor_poll_event(&event) != TERRAIN_OK)
        {
            return b_counted;
        }
        terrain_event_type_t const type = terrain_monitor_event_type(&event);
        if (type == TERRAIN_EVENT_NONE)
        {
            return b_counted;
        }
        print_event(&event);
        if ((type == TERRAIN_EVENT_LEVEL) ||
            (type == TERRAIN_EVENT_COLLISION) ||
            (type == TERRAIN_EVENT_TURNING))
        {
            b_counted = true;
        }
    }
}

/*
 * The run summary in one line: hump count, the kept heights in order, the
 * highest, collisions and turns, then the packed message in hex, byte 0
 * first, exactly as it would be sent.
 */
static void
print_summary (void)
{
    terrain_summary_t summary = {{0u}};
    uint8_t msg[TERRAIN_SUMMARY_BYTES] = {0u};
    if ((terrain_monitor_get_summary(&summary) != TERRAIN_OK) ||
        (terrain_monitor_summary_pack(&summary, msg) != TERRAIN_OK))
    {
        return;
    }

    uint32_t const humps = terrain_monitor_summary_hump_count(&summary);
    uint32_t const kept =
        (humps < TERRAIN_SUMMARY_HUMPS) ? humps : TERRAIN_SUMMARY_HUMPS;
    tm_printf((UB *)"terrain: summary humps %d (mm:", (INT)humps);
    for (uint32_t i = 0u; i < kept; i++)
    {
        tm_printf((UB *)" %d",
                  (INT)terrain_monitor_summary_hump_mm(&summary, i));
    }
    tm_printf((UB *)") highest %d mm collisions %d turns %d msg",
              (INT)terrain_monitor_summary_highest_mm(&summary),
              (INT)terrain_monitor_summary_collisions(&summary),
              (INT)terrain_monitor_summary_turns(&summary));
    for (uint32_t i = 0u; i < TERRAIN_SUMMARY_BYTES; i++)
    {
        tm_printf((UB *)" %02X", (UINT)msg[i]);
    }
    tm_printf((UB *)"\n");
}

/*
 * One line per event, with its time since boot in seconds (comes round
 * every 655.36 s). ASCENDING and DESCENDING carry the lean (size of roll)
 * then, LEVEL the hump's height, COLLISION the push in tenths of a g;
 * TURNING, STRAIGHT and the speed states carry nothing.
 */
static void
print_event (terrain_event_t const * p_event)
{
    terrain_event_type_t const type = terrain_monitor_event_type(p_event);
    INT const value = (INT)terrain_monitor_event_value(p_event);
    INT const time_10ms = (INT)terrain_monitor_event_time_10ms(p_event);

    if (type == TERRAIN_EVENT_COLLISION)
    {
        tm_printf((UB *)"terrain: COLLISION push %d.%d g",
                  value / TENTHS_PER_UNIT, value % TENTHS_PER_UNIT);
    }
    else if (type == TERRAIN_EVENT_LEVEL)
    {
        tm_printf((UB *)"terrain: hump LEVEL height %d mm", value);
    }
    else if ((type == TERRAIN_EVENT_TURNING) ||
             (type == TERRAIN_EVENT_STRAIGHT))
    {
        tm_printf((UB *)"terrain: %s", terrain_monitor_event_text(type));
    }
    else if ((type >= TERRAIN_EVENT_STATIONARY) &&
             (type <= TERRAIN_EVENT_DECELERATING))
    {
        tm_printf((UB *)"terrain: speed %s", terrain_monitor_event_text(type));
    }
    else
    {
        tm_printf((UB *)"terrain: hump %s", terrain_monitor_event_text(type));
        INT tenths = value;
        char const * p_sign = "";
        if (tenths < 0)
        {
            p_sign = "-";
            tenths = -tenths;
        }
        tm_printf((UB *)" lean %s%d.%d deg", p_sign, tenths / TENTHS_PER_UNIT,
                  tenths % TENTHS_PER_UNIT);
    }
    tm_printf((UB *)" t %d.%02d s\n", time_10ms / HUNDREDTHS_PER_UNIT,
              time_10ms % HUNDREDTHS_PER_UNIT);
}

#if TERRAIN_COMPASS_PRINT_MS > 0
/*
 * Raw car-axis field, heading in tenths of a degree, and the X/Y centre
 * (the offset to set) and span seen so far.
 */
static void
print_compass (void)
{
    terrain_compass_t reading;
    if (!terrain_compass_get(&reading))
    {
        tm_printf((UB *)"terrain: compass no reading (%d samples)\n",
                  (INT)reading.samples);
        return;
    }
    terrain_compass_range_t range;
    terrain_compass_get_range(&range);
    tm_printf((UB *)"terrain: compass x %d y %d z %d heading %d ddeg "
                    "centre %d %d span %d %d\n",
              (INT)reading.x, (INT)reading.y, (INT)reading.z,
              (INT)reading.heading_ddeg,
              ((INT)range.min_x + (INT)range.max_x) / 2,
              ((INT)range.min_y + (INT)range.max_y) / 2,
              (INT)range.max_x - (INT)range.min_x,
              (INT)range.max_y - (INT)range.min_y);
}
#endif

#if TERRAIN_DEBUG_PRINT_MS > 0
/* One line of pitch, roll, hump state, highest hump, turning and speed;
 * calibration progress before. */
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
    tm_printf((UB *)" hump %s highest %d mm",
              terrain_monitor_hump_text(terrain_monitor_data_hump_state(&data)),
              (INT)terrain_monitor_data_highest_hump_mm(&data));
    tm_printf(
        (UB *)" turn %d deg/s %s",
        (INT)terrain_monitor_data_turn_rate_dps(&data),
        terrain_monitor_turn_text(terrain_monitor_data_turn_state(&data)));
    tm_printf(
        (UB *)" speed %s\n",
        terrain_monitor_speed_text(terrain_monitor_data_speed_state(&data)));
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
