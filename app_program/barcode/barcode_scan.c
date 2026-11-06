/**
 * @file barcode_scan.c
 * @brief Turn timestamped barcode-sensor edges into one scan's widths.
 *
 * Times are the low 32 bits of the 1 MHz TIMER; unsigned subtraction
 * handles its wrap.
 */

#include "barcode_scan.h"

/* Private types, constants, and macros. */

/* Private function declarations. */
static void end_scan (barcode_scan_t * p_scan);
static void append (barcode_scan_t * p_scan, uint32_t width_us);

/* Public function definitions. */

void
barcode_filter_init (barcode_filter_t * p_filter, bool b_black, uint32_t now_us)
{
    p_filter->last_us = now_us;
    p_filter->b_settled = b_black;
}

bool
barcode_filter_edge (barcode_filter_t * p_filter, uint32_t edge_us,
                     bool b_black, bool * p_level)
{
    bool const b_transition =
        (edge_us - p_filter->last_us) >= BARCODE_MIN_PULSE_US;
    if (b_transition)
    {
        *p_level = !p_filter->b_settled;
    }
    p_filter->last_us = edge_us;
    p_filter->b_settled = b_black;
    return b_transition;
}

void
barcode_trigger_init (barcode_trigger_t * p_trigger)
{
    p_trigger->next = 0u;
    p_trigger->count = 0u;
    p_trigger->b_held = false;
}

void
barcode_trigger_hold (barcode_trigger_t * p_trigger, uint32_t until_us)
{
    p_trigger->next = 0u;
    p_trigger->count = 0u;
    p_trigger->hold_until_us = until_us;
    p_trigger->b_held = true;
}

bool
barcode_trigger_edge (barcode_trigger_t * p_trigger, uint32_t edge_us)
{
    if (p_trigger->b_held)
    {
        if ((int32_t)(edge_us - p_trigger->hold_until_us) < 0)
        {
            return false;
        }
        p_trigger->b_held = false;
    }
    /* The ring holds the edges before this one, oldest at next. */
    bool const b_trigger = (p_trigger->count == (BARCODE_TRIGGER_EDGES - 1u)) &&
                           ((edge_us - p_trigger->edges_us[p_trigger->next]) <=
                            BARCODE_TRIGGER_WINDOW_US);
    if (b_trigger)
    {
        p_trigger->next = 0u;
        p_trigger->count = 0u;
        return true;
    }
    p_trigger->edges_us[p_trigger->next] = edge_us;
    p_trigger->next = (p_trigger->next + 1u) % (BARCODE_TRIGGER_EDGES - 1u);
    if (p_trigger->count < (BARCODE_TRIGGER_EDGES - 1u))
    {
        p_trigger->count++;
    }
    return false;
}

void
barcode_scan_sync (barcode_scan_t * p_scan, bool b_black)
{
    if (!p_scan->b_active)
    {
        p_scan->b_black = b_black;
    }
}

void
barcode_scan_init (barcode_scan_t * p_scan, bool b_black, uint32_t now_us)
{
    p_scan->b_black = b_black;
    p_scan->last_us = now_us;
    barcode_scan_reset(p_scan);
}

void
barcode_scan_reset (barcode_scan_t * p_scan)
{
    p_scan->count = 0u;
    p_scan->b_active = false;
    p_scan->b_overflow = false;
    p_scan->b_long_black = false;
    p_scan->narrowest_us = UINT32_MAX;
}

barcode_scan_result_t
barcode_scan_edge (barcode_scan_t * p_scan, uint32_t edge_us, bool b_black)
{
    if (b_black == p_scan->b_black)
    {
        return BARCODE_SCAN_BUSY;
    }
    uint32_t const width = edge_us - p_scan->last_us;
    if (p_scan->b_active)
    {
        if (width > BARCODE_QUIET_MAX_US)
        {
            /* The white before this edge was a quiet zone, or the black
             * was too long for a bar. */
            end_scan(p_scan);
            return BARCODE_SCAN_DONE;
        }
        append(p_scan, width);
    }
    else if (b_black)
    {
        p_scan->b_active = true;
    }
    else
    {
        /* White after a black outside a scan: wait for the next bar. */
    }
    p_scan->b_black = b_black;
    p_scan->last_us = edge_us;
    return BARCODE_SCAN_BUSY;
}

barcode_scan_result_t
barcode_scan_poll (barcode_scan_t * p_scan, uint32_t now_us)
{
    if (!p_scan->b_active)
    {
        return BARCODE_SCAN_BUSY;
    }
    uint32_t const elapsed = now_us - p_scan->last_us;
    if ((int32_t)elapsed < 0)
    {
        /* An edge newer than @p now_us was already taken. */
        return BARCODE_SCAN_BUSY;
    }
    if (elapsed <= BARCODE_QUIET_MAX_US)
    {
        return BARCODE_SCAN_BUSY;
    }
    end_scan(p_scan);
    return BARCODE_SCAN_DONE;
}

/* Private function definitions. */

/* Over-long black ends the scan at the last bar, dropping the white after it:
 * the elements before it may still read (2026-11-06: bars merged 2 s into a
 * barcode, and dropping the whole scan lost the 19 good elements before). */
static void
end_scan (barcode_scan_t * p_scan)
{
    if (p_scan->b_black)
    {
        p_scan->b_long_black = true;
        if (p_scan->count > 0u)
        {
            p_scan->count--;
        }
    }
}

static void
append (barcode_scan_t * p_scan, uint32_t width_us)
{
    if (p_scan->count >= BARCODE_MAX_ELEMENTS)
    {
        p_scan->b_overflow = true;
        return;
    }
    p_scan->widths[p_scan->count] = width_us;
    p_scan->count++;
    if (width_us < p_scan->narrowest_us)
    {
        p_scan->narrowest_us = width_us;
    }
}
