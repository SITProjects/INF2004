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
#if TM_MQTT
#include "../communication/telemetry_queue.h"
#endif

/* Private types, constants, and macros. */
/* Fewer elements than one character is noise, not a failed read. */
#define MIN_REPORT_ELEMENTS BARCODE_CODE39_ELEMENTS

/* Static data. */
ID barcode_tid_read;
static barcode_scan_t scan;

/* Private function declarations. */
static void read_task (INT stacd, void * p_exinf);
static void finish_scan (void);
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
    uint32_t overflows = drv_barcode_overflows();
    for (;;)
    {
        barcode_edge_t edge;
        while (drv_barcode_pop(&edge))
        {
            if (barcode_scan_edge(&scan, edge.time_us, edge.b_black) ==
                BARCODE_SCAN_DONE)
            {
                finish_scan();
                (void)barcode_scan_edge(&scan, edge.time_us, edge.b_black);
            }
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
        (void)tk_dly_tsk(BARCODE_POLL_MS);
    }
}

/* Decode, report and drop the finished scan. */
static void
finish_scan (void)
{
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
        if (BARCODE_CODE39_OK == status)
        {
            tm_printf((UB *)"barcode: %s dir=%s elements=%u narrow=%u us\n",
                      text, b_reversed ? "rev" : "fwd", (UINT)scan.count,
                      (UINT)scan.narrowest_us);
#if TM_MQTT
            publish(text, b_reversed);
#endif
        }
        else
        {
            tm_printf((UB *)"barcode: FAIL %s elements=%u narrow=%u us\n",
                      barcode_code39_status_name(status), (UINT)scan.count,
                      (UINT)scan.narrowest_us);
        }
    }
    else
    {
        /* Too short to be a barcode. */
    }
    barcode_scan_reset(&scan);
}

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
