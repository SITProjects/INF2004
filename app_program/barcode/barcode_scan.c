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

static void scan_flag_set (barcode_scan_t * p_scan, uint32_t flag, bool b_set);
static uint16_t width_units (uint32_t width_us);
static bool trigger_flag (barcode_trigger_t const * p_trigger, uint32_t flag);
static void trigger_flag_set (barcode_trigger_t * p_trigger, uint32_t flag,
                              bool b_set);
static uint32_t trigger_field (barcode_trigger_t const * p_trigger,
                               uint32_t shift, uint32_t bits);
static void trigger_field_set (barcode_trigger_t * p_trigger, uint32_t shift,
                               uint32_t bits, uint32_t value);

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
    trigger_field_set(p_trigger, BARCODE_TRIGGER_NEXT, 0u);
    trigger_field_set(p_trigger, BARCODE_TRIGGER_COUNT, 0u);
    trigger_flag_set(p_trigger, BARCODE_TRIGGER_F_HELD, false);
    trigger_flag_set(p_trigger, BARCODE_TRIGGER_F_HUMP, false);
    p_trigger->hump_start_us = 0u;
}

void
barcode_trigger_hold (barcode_trigger_t * p_trigger, uint32_t until_us)
{
    trigger_field_set(p_trigger, BARCODE_TRIGGER_NEXT, 0u);
    trigger_field_set(p_trigger, BARCODE_TRIGGER_COUNT, 0u);
    p_trigger->hold_until_us = until_us;
    trigger_flag_set(p_trigger, BARCODE_TRIGGER_F_HELD, true);
}

void
barcode_trigger_hump (barcode_trigger_t * p_trigger, bool b_hump,
                      uint32_t now_us)
{
    if (b_hump && !trigger_flag(p_trigger, BARCODE_TRIGGER_F_HUMP))
    {
        p_trigger->hump_start_us = now_us;
    }
    trigger_flag_set(p_trigger, BARCODE_TRIGGER_F_HUMP, b_hump);
    if (b_hump &&
        ((now_us - p_trigger->hump_start_us) < BARCODE_TRIGGER_HUMP_MAX_US))
    {
        uint32_t const until_us = now_us + BARCODE_TRIGGER_HUMP_US;
        if (!trigger_flag(p_trigger, BARCODE_TRIGGER_F_HELD) ||
            ((int32_t)(until_us - p_trigger->hold_until_us) > 0))
        {
            barcode_trigger_hold(p_trigger, until_us);
        }
    }
}

bool
barcode_trigger_edge (barcode_trigger_t * p_trigger, uint32_t edge_us)
{
    if (trigger_flag(p_trigger, BARCODE_TRIGGER_F_HELD))
    {
        if ((int32_t)(edge_us - p_trigger->hold_until_us) < 0)
        {
            return false;
        }
        trigger_flag_set(p_trigger, BARCODE_TRIGGER_F_HELD, false);
    }
    /* The ring holds the edges before this one, oldest at next. */
    bool const b_trigger = (trigger_field(p_trigger, BARCODE_TRIGGER_COUNT) ==
                            (BARCODE_TRIGGER_EDGES - 1u)) &&
                           ((edge_us - p_trigger->edges_us[trigger_field(
                                           p_trigger, BARCODE_TRIGGER_NEXT)]) <=
                            BARCODE_TRIGGER_WINDOW_US);
    if (b_trigger)
    {
        trigger_field_set(p_trigger, BARCODE_TRIGGER_NEXT, 0u);
        trigger_field_set(p_trigger, BARCODE_TRIGGER_COUNT, 0u);
        return true;
    }
    p_trigger->edges_us[trigger_field(p_trigger, BARCODE_TRIGGER_NEXT)] =
        edge_us;
    trigger_field_set(p_trigger, BARCODE_TRIGGER_NEXT,
                      (trigger_field(p_trigger, BARCODE_TRIGGER_NEXT) + 1u) %
                          (BARCODE_TRIGGER_EDGES - 1u));
    if (trigger_field(p_trigger, BARCODE_TRIGGER_COUNT) <
        (BARCODE_TRIGGER_EDGES - 1u))
    {
        trigger_field_set(p_trigger, BARCODE_TRIGGER_COUNT,
                          trigger_field(p_trigger, BARCODE_TRIGGER_COUNT) + 1u);
    }
    return false;
}

void
barcode_scan_sync (barcode_scan_t * p_scan, bool b_black)
{
    if (!barcode_scan_flag(p_scan, BARCODE_SCAN_F_ACTIVE))
    {
        scan_flag_set(p_scan, BARCODE_SCAN_F_BLACK, b_black);
    }
}

void
barcode_scan_init (barcode_scan_t * p_scan, bool b_black, uint32_t now_us)
{
    scan_flag_set(p_scan, BARCODE_SCAN_F_BLACK, b_black);
    p_scan->last_us = now_us;
    barcode_scan_reset(p_scan);
}

void
barcode_scan_reset (barcode_scan_t * p_scan)
{
    p_scan->count = 0u;
    scan_flag_set(p_scan, BARCODE_SCAN_F_ACTIVE, false);
    scan_flag_set(p_scan, BARCODE_SCAN_F_OVERFLOW, false);
    scan_flag_set(p_scan, BARCODE_SCAN_F_LONG_BLACK, false);
    p_scan->narrowest = UINT16_MAX;
}

barcode_scan_result_t
barcode_scan_edge (barcode_scan_t * p_scan, uint32_t edge_us, bool b_black)
{
    if (b_black == barcode_scan_flag(p_scan, BARCODE_SCAN_F_BLACK))
    {
        return BARCODE_SCAN_BUSY;
    }
    uint32_t const width = edge_us - p_scan->last_us;
    if (barcode_scan_flag(p_scan, BARCODE_SCAN_F_ACTIVE))
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
        scan_flag_set(p_scan, BARCODE_SCAN_F_ACTIVE, true);
    }
    else
    {
        /* White after a black outside a scan: wait for the next bar. */
    }
    scan_flag_set(p_scan, BARCODE_SCAN_F_BLACK, b_black);
    p_scan->last_us = edge_us;
    return BARCODE_SCAN_BUSY;
}

barcode_scan_result_t
barcode_scan_poll (barcode_scan_t * p_scan, uint32_t now_us)
{
    if (!barcode_scan_flag(p_scan, BARCODE_SCAN_F_ACTIVE))
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
    if (barcode_scan_flag(p_scan, BARCODE_SCAN_F_BLACK))
    {
        scan_flag_set(p_scan, BARCODE_SCAN_F_LONG_BLACK, true);
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
        scan_flag_set(p_scan, BARCODE_SCAN_F_OVERFLOW, true);
        return;
    }
    uint16_t const width = width_units(width_us);
    p_scan->widths[p_scan->count] = width;
    p_scan->count++;
    if (width < p_scan->narrowest)
    {
        p_scan->narrowest = width;
    }
}

static void
scan_flag_set (barcode_scan_t * p_scan, uint32_t flag, bool b_set)
{
    p_scan->flags = b_set ? (uint8_t)(p_scan->flags | flag)
                          : (uint8_t)(p_scan->flags & (uint8_t)~flag);
}

/* A width in stored steps, rounded and saturated; never 0, so an element
 * still counts however short. */
static uint16_t
width_units (uint32_t width_us)
{
    uint32_t const units =
        (width_us + (BARCODE_WIDTH_UNIT_US / 2u)) / BARCODE_WIDTH_UNIT_US;

    return (0u == units) ? 1u : robot_clamp_u16(units);
}

static bool
trigger_flag (barcode_trigger_t const * p_trigger, uint32_t flag)
{
    return (p_trigger->bits & flag) != 0u;
}

static void
trigger_flag_set (barcode_trigger_t * p_trigger, uint32_t flag, bool b_set)
{
    p_trigger->bits = b_set ? (uint8_t)(p_trigger->bits | flag)
                            : (uint8_t)(p_trigger->bits & (uint8_t)~flag);
}

static uint32_t
trigger_field (barcode_trigger_t const * p_trigger, uint32_t shift,
               uint32_t bits)
{
    return robot_field_get(p_trigger->bits, shift, bits);
}

static void
trigger_field_set (barcode_trigger_t * p_trigger, uint32_t shift, uint32_t bits,
                   uint32_t value)
{
    p_trigger->bits =
        (uint8_t)robot_field_put(p_trigger->bits, shift, bits, value);
}
