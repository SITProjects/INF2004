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
static uint32_t quiet_us (barcode_scan_t const * p_scan);
static void append (barcode_scan_t * p_scan, uint32_t width_us);

/* Public function definitions. */

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
        if (!p_scan->b_black && (width > quiet_us(p_scan)))
        {
            /* The white before this edge was a quiet zone. */
            return BARCODE_SCAN_DONE;
        }
        if (p_scan->b_black && (width > BARCODE_QUIET_MAX_US))
        {
            /* Too long for a bar: not a barcode. */
            barcode_scan_reset(p_scan);
        }
        else
        {
            append(p_scan, width);
        }
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
    if (p_scan->b_black)
    {
        if (elapsed > BARCODE_QUIET_MAX_US)
        {
            barcode_scan_reset(p_scan);
        }
        return BARCODE_SCAN_BUSY;
    }
    return (elapsed > quiet_us(p_scan)) ? BARCODE_SCAN_DONE : BARCODE_SCAN_BUSY;
}

/* Private function definitions. */

/* White longer than this ends the scan. */
static uint32_t
quiet_us (barcode_scan_t const * p_scan)
{
    uint64_t const relative =
        (uint64_t)p_scan->narrowest_us * BARCODE_QUIET_FACTOR;
    return (relative < BARCODE_QUIET_MAX_US) ? (uint32_t)relative
                                             : BARCODE_QUIET_MAX_US;
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
