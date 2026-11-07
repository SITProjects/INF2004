/**
 * @file barcode.c
 * @brief Barcode subsystem: reads Code 39 with a third IR sensor.
 *
 * tBarcode sets up GP28 and its edge interrupt, then every BARCODE_POLL_MS
 * drains the timestamped edges into a scan (barcode_scan.c). When a quiet
 * zone ends a scan it decodes it (barcode_code39.c), prints the result and
 * publishes it over MQTT in MQTT builds. What the car does with a barcode
 * is not decided yet.
 */

#include "barcode.h"
#include "barcode_code39.h"
#include "barcode_config.h"
#include "barcode_scan.h"
#include "drv_barcode.h"
#include "../communication/remote_command.h"
#include "../line_follow/line_follow.h"
#include "../terrain/terrain_monitor.h"
#if TM_MQTT
#include "../communication/telemetry_queue.h"
#endif

/* Private types, constants, and macros. */
/* Fewer elements than one character is noise, not a failed read. */
#define MIN_REPORT_ELEMENTS BARCODE_CODE39_ELEMENTS
#define US_PER_TENTH_MS 100u
#define CALIBRATION_REPORT_US 500000u
#define SHORT_SCAN_MIN_ELEMENTS 2u

/* Static data. */
ID barcode_tid_read;
static barcode_scan_t scan;
static barcode_trigger_t trigger;
static uint32_t last_edge_us; /* Latest edge, or start of this phase. */

/* Private function declarations. */
static void read_task (INT stacd, void * p_exinf);
static void finish_scan (void);
static void take_edge (barcode_edge_t const * p_edge,
                       line_barcode_phase_t phase);
static line_barcode_phase_t follow_phase (void);
static bool on_hump (void);
static void trace_widths (void);
#if BARCODE_EDGE_REPORT
static void calibration_report (void);
#endif
static uint32_t now_us (void);
#if TM_MQTT
static void publish (char const * p_text, bool b_reversed);
#endif

/* Public function definitions. */

ER
barcode_init (void)
{
    /* No kernel objects; the task sets up its pin and interrupt. */
    return E_OK;
}

ER
barcode_start (void)
{
    /* Core 0: drv_gpio_irq enables the interrupt in that core's NVIC. */
    barcode_tid_read =
        app_start_task(read_task, BARCODE_PRI_READ, TP_PRC1, "tBarcode");
    return (barcode_tid_read < E_OK) ? barcode_tid_read : E_OK;
}

/* Private function definitions. */

static void
read_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    ER const result = drv_barcode_init();
    if (result != E_OK)
    {
        app_check(result, "barcode interrupt");
        tk_ext_tsk();
    }
    tm_printf((UB *)"barcode: Code 39 reader on GP28 started\n");

    barcode_scan_init(&scan, drv_barcode_black(), now_us());
    barcode_trigger_init(&trigger);
    uint32_t overflows = drv_barcode_overflows();
    for (;;)
    {
        line_barcode_phase_t const phase = follow_phase();
        barcode_trigger_hump(&trigger, on_hump(), now_us());
        barcode_edge_t edge;
        while (drv_barcode_pop(&edge))
        {
            take_edge(&edge, phase);
        }
        if (drv_barcode_overflows() != overflows)
        {
            /* Lost edges: the widths no longer line up. */
            overflows = drv_barcode_overflows();
            barcode_scan_reset(&scan);
            tm_printf((UB *)"barcode: edge queue overflow, scan dropped\n");
        }
        if (barcode_scan_poll(&scan, now_us()) == BARCODE_SCAN_DONE)
        {
            finish_scan();
        }
        barcode_scan_sync(&scan, drv_barcode_black());
#if BARCODE_EDGE_REPORT
        calibration_report();
#endif
        (void)tk_dly_tsk(BARCODE_POLL_MS);
    }
}

/*
 * Keep up with the line task's slow pass. A scan cannot span a change of
 * phase, as the speed changes with it. After the pass, what is left of the
 * barcode must not start another. A pass with no edges was a false alarm.
 */
static line_barcode_phase_t
follow_phase (void)
{
    static line_barcode_phase_t last_phase = LINE_BARCODE_NONE;
    line_barcode_phase_t const phase = line_follow_barcode_phase();
    uint32_t const now = now_us();
    if (phase != last_phase)
    {
        barcode_scan_reset(&scan);
        last_edge_us = now;
        if (LINE_BARCODE_NONE == phase)
        {
            barcode_trigger_hold(&trigger, now + BARCODE_TRIGGER_HOLDOFF_US);
        }
        last_phase = phase;
    }
    if ((LINE_BARCODE_CREEP == phase) &&
        ((now - last_edge_us) > BARCODE_CREEP_QUIET_US))
    {
        line_follow_barcode_done();
    }
    return phase;
}

/* Level until calibrated: the terrain task sees humps only then. */
static bool
on_hump (void)
{
    terrain_data_t data = {{0u}};
    (void)terrain_monitor_get_latest(&data);
    return terrain_monitor_data_hump_state(&data) != TERRAIN_HUMP_LEVEL;
}

/* While stopping or backing up, widths mean nothing: drop the edge. */
static void
take_edge (barcode_edge_t const * p_edge, line_barcode_phase_t phase)
{
    if ((LINE_BARCODE_BRAKE == phase) || (LINE_BARCODE_BACK == phase))
    {
        return;
    }
    last_edge_us = p_edge->time_us;
    if ((LINE_BARCODE_NONE == phase) &&
        barcode_trigger_edge(&trigger, p_edge->time_us))
    {
        line_follow_barcode_ahead();
    }
    if (barcode_scan_edge(&scan, p_edge->time_us, p_edge->b_black) ==
        BARCODE_SCAN_DONE)
    {
        finish_scan();
        (void)barcode_scan_edge(&scan, p_edge->time_us, p_edge->b_black);
    }
}

/* Decode, report and drop the finished scan. */
static void
finish_scan (void)
{
    if (scan.b_long_black && (scan.count >= SHORT_SCAN_MIN_ELEMENTS))
    {
        tm_printf((UB *)"barcode: long black ended the scan, elements=%u\n",
                  (UINT)scan.count);
    }
    if (scan.b_overflow)
    {
        tm_printf((UB *)"barcode: FAIL too many elements\n");
    }
    else if (scan.count >= MIN_REPORT_ELEMENTS)
    {
        char text[BARCODE_MAX_TEXT];
        bool b_reversed = false;
        barcode_code39_status_t const status = barcode_code39_decode(
            scan.widths, scan.count, text, sizeof(text), &b_reversed);
        /* A partial read has one frame, not two: it can be wrong, so only
         * a barcode the dashboard knows counts (2026-11-06: a false '.'). */
        bool const b_partial =
            (status != BARCODE_CODE39_OK) &&
            (BARCODE_CODE39_OK ==
             barcode_code39_decode_partial(scan.widths, scan.count, text,
                                           sizeof(text), &b_reversed)) &&
            remote_command_mapped(text);
        if ((BARCODE_CODE39_OK == status) || b_partial)
        {
            line_turn_t const turn = remote_command_turn(text);
            line_follow_set_turn(turn);
            line_follow_barcode_done();
            tm_printf((UB *)"barcode: %s dir=%s elements=%u narrow=%u us "
                            "turn=%s%s\n",
                      text, b_reversed ? "rev" : "fwd", (UINT)scan.count,
                      (UINT)scan.narrowest_us, line_track_turn_name(turn),
                      b_partial ? " partial" : "");
#if TM_MQTT
            publish(text, b_reversed);
#endif
        }
        else
        {
            tm_printf((UB *)"barcode: FAIL %s elements=%u narrow=%u us\n",
                      barcode_code39_status_name(status), (UINT)scan.count,
                      (UINT)scan.narrowest_us);
            trace_widths();
            if (!scan.b_long_black)
            {
                /* The barcode has passed; after a long black, more follows. */
                line_follow_barcode_done();
            }
        }
    }
    else
    {
        /* Too short to be a barcode. */
#if BARCODE_EDGE_REPORT
        if (scan.count >= SHORT_SCAN_MIN_ELEMENTS)
        {
            tm_printf((UB *)"barcode: short scan elements=%u\n",
                      (UINT)scan.count);
            trace_widths();
        }
#endif
    }
    barcode_scan_reset(&scan);
}

/* The failed scan's widths, tenths of a ms, black first, for the log. */
static void
trace_widths (void)
{
#if BARCODE_TRACE_WIDTHS > 0u
    uint32_t const count =
        (scan.count < BARCODE_TRACE_WIDTHS) ? scan.count : BARCODE_TRACE_WIDTHS;
    tm_printf((UB *)"barcode: widths");
    for (uint32_t i = 0u; i < count; i++)
    {
        tm_printf((UB *)" %u", (UINT)(scan.widths[i] / US_PER_TENTH_MS));
    }
    tm_printf((UB *)"\n");
#endif
}

#if BARCODE_EDGE_REPORT
/*
 * For tuning the potentiometer: the level, and the edges since the last
 * report. raw stays near 0 on plain white or black when DO is clean; a
 * high raw with kept 0 is DO oscillating at its threshold.
 */
static void
calibration_report (void)
{
    static uint32_t last_us;
    static uint32_t last_raw;
    static uint32_t last_kept;
    uint32_t const now = now_us();
    if ((now - last_us) < CALIBRATION_REPORT_US)
    {
        return;
    }
    uint32_t raw = 0u;
    uint32_t kept = 0u;
    drv_barcode_edge_counts(&raw, &kept);
    tm_printf((UB *)"barcode: B=%u raw=%u kept=%u\n", (UINT)drv_barcode_black(),
              (UINT)(raw - last_raw), (UINT)(kept - last_kept));
    last_us = now;
    last_raw = raw;
    last_kept = kept;
}
#endif

/* Low 32 bits of the 1 MHz TIMER, as the edge timestamps. */
static uint32_t
now_us (void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

#if TM_MQTT
/* A full or busy queue drops it: the console line is the record. */
static void
publish (char const * p_text, bool b_reversed)
{
    char record[TELEMETRY_TEXT_SIZE];
    SYSTIM now;
    uint32_t sequence = 0u;
    (void)tm_sprintf((UB *)record, (const UB *)"barcode %s %s", p_text,
                     b_reversed ? "rev" : "fwd");
    uint32_t const now_ms = (tk_get_otm(&now) == E_OK) ? (uint32_t)now.lo : 0u;
    (void)telemetry_queue_push(now_ms, record, &sequence);
}
#endif
