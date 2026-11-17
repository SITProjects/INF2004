/* Drive the real Code 39 decoder and scan assembly with synthetic bars. */
#include "../../../app_program/barcode/barcode_code39.h"
#include "../../../app_program/barcode/barcode_scan.h"
#include <stdio.h>
#include <string.h>

/* 4 mm narrow element at 120 mm/s, wide 2.5 x narrow. */
#define NARROW_US 33000u
#define RATIO_X10 25u
#define MAX_WIDTHS 300u
/* The 2026-11-06 car read black 30 ms short of a 75 ms narrow element. */
#define BAR_BIAS_US 13000u

static unsigned int failures;
static uint32_t widths[MAX_WIDTHS];
static uint16_t stored[MAX_WIDTHS];

/* @p p_us widths as barcode_scan_t stores them. */
static uint16_t const *
as_stored (uint32_t const * p_us, uint32_t count)
{
    for (uint32_t i = 0u; i < count; i++)
    {
        uint32_t const units =
            (p_us[i] + (BARCODE_WIDTH_UNIT_US / 2u)) / BARCODE_WIDTH_UNIT_US;
        stored[i] = (units > UINT16_MAX) ? UINT16_MAX : (uint16_t)units;
    }
    return stored;
}
static uint32_t random_state = 12345u;

static void
check (char const * p_name, bool ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", p_name);
    failures += ok ? 0u : 1u;
}

/* Deterministic factor from (100 - jitter)% to (100 + jitter)%. */
static uint32_t
jittered (uint32_t width, uint32_t jitter_pct)
{
    random_state = random_state * 1103515245u + 12345u;
    uint32_t const span = 2u * jitter_pct + 1u;
    uint32_t const pct = 100u - jitter_pct + ((random_state >> 16) % span);
    return (width * pct) / 100u;
}

/* Elements of @p p_symbols (framing included by the caller). */
static uint32_t
encode (char const * p_symbols, uint32_t ratio_x10, uint32_t jitter_pct)
{
    uint32_t count = 0u;
    for (uint32_t c = 0u; p_symbols[c] != '\0'; c++)
    {
        uint32_t const pattern = barcode_code39_pattern(p_symbols[c]);
        if (c > 0u)
        {
            widths[count++] = jittered(NARROW_US, jitter_pct); /* Gap. */
        }
        for (uint32_t i = 0u; i < BARCODE_CODE39_ELEMENTS; i++)
        {
            bool const b_wide = (pattern >> (8u - i)) & 1u;
            uint32_t const width =
                b_wide ? (NARROW_US * ratio_x10) / 10u : NARROW_US;
            widths[count++] = jittered(width, jitter_pct);
        }
    }
    return count;
}

static void
reverse (uint32_t count)
{
    for (uint32_t i = 0u; i < count / 2u; i++)
    {
        uint32_t const swap = widths[i];
        widths[i] = widths[count - 1u - i];
        widths[count - 1u - i] = swap;
    }
}

static barcode_code39_status_t
decode (uint32_t count, char * p_text, uint32_t size, bool * p_reversed)
{
    return barcode_code39_decode(as_stored(widths, count), count, p_text, size, p_reversed);
}

static void
test_every_symbol (void)
{
    static char const symbols[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    bool b_all = true;
    for (uint32_t i = 0u; symbols[i] != '\0'; i++)
    {
        char framed[4] = {'*', symbols[i], '*', '\0'};
        char text[8];
        bool b_reversed = true;
        uint32_t const count = encode(framed, RATIO_X10, 0u);
        bool const b_forward =
            (BARCODE_CODE39_OK ==
             decode(count, text, sizeof(text), &b_reversed)) &&
            !b_reversed && (text[0] == symbols[i]) && (text[1] == '\0');
        reverse(count);
        bool const b_backward =
            (BARCODE_CODE39_OK ==
             decode(count, text, sizeof(text), &b_reversed)) &&
            b_reversed && (text[0] == symbols[i]);
        if (!b_forward || !b_backward)
        {
            printf("  symbol '%c' forward %d backward %d\n", symbols[i],
                   (int)b_forward, (int)b_backward);
            b_all = false;
        }
    }
    check("BT01 every symbol decodes forwards and backwards", b_all);
}

static void
test_jitter (void)
{
    char text[16];
    bool b_reversed = true;
    bool b_all = true;
    for (uint32_t run = 0u; run < 200u; run++)
    {
        uint32_t const count = encode("*INF2004*", RATIO_X10, 20u);
        if (run & 1u)
        {
            reverse(count);
        }
        b_all = b_all &&
                (BARCODE_CODE39_OK ==
                 decode(count, text, sizeof(text), &b_reversed)) &&
                (0 == strcmp(text, "INF2004")) &&
                (b_reversed == ((run & 1u) != 0u));
    }
    check("BT02 +/-20% width jitter at 2.5:1 decodes, both directions", b_all);
}

static void
test_rejects (void)
{
    char text[16];
    bool b_reversed;
    uint32_t count = encode("*AB*", 12u, 0u);
    bool const b_ratio = (BARCODE_CODE39_ERR_RATIO ==
                          decode(count, text, sizeof(text), &b_reversed));
    count = encode("ABC", RATIO_X10, 0u);
    bool const b_frame = (BARCODE_CODE39_ERR_FRAME ==
                          decode(count, text, sizeof(text), &b_reversed)) &&
                         (text[0] == '\0');
    count = encode("*A*B*", RATIO_X10, 0u);
    bool const b_inner = (BARCODE_CODE39_ERR_FRAME ==
                          decode(count, text, sizeof(text), &b_reversed));
    count = encode("*AB*", RATIO_X10, 0u);
    bool const b_length = (BARCODE_CODE39_ERR_LENGTH ==
                           decode(count - 5u, text, sizeof(text), &b_reversed));
    bool const b_space =
        (BARCODE_CODE39_ERR_SPACE == decode(count, text, 2u, &b_reversed));
    count = encode("**", RATIO_X10, 0u);
    bool const b_empty = (BARCODE_CODE39_ERR_LENGTH ==
                          decode(count, text, sizeof(text), &b_reversed));
    check("BT03 rejects a weak ratio, no frame, '*' inside, truncation, "
          "a small buffer and an empty symbol",
          b_ratio && b_frame && b_inner && b_length && b_space && b_empty);
}

/* Feed @p count widths as edges from @p start_us; return the end time. */
static uint32_t
feed_scan (barcode_scan_t * p_scan, uint32_t count, uint32_t start_us,
           bool * p_done_early)
{
    uint32_t now = start_us;
    bool b_black = true;
    *p_done_early = false;
    for (uint32_t i = 0u; i <= count; i++)
    {
        if (barcode_scan_edge(p_scan, now, b_black) == BARCODE_SCAN_DONE)
        {
            *p_done_early = true;
        }
        if (i < count)
        {
            now += widths[i];
        }
        b_black = !b_black;
    }
    return now;
}

/* Decode one logged scan partially; "" if it is refused. */
static bool
partial_is (uint32_t const * p_tenths, uint32_t count, char const * p_expected,
            bool b_expected_reversed)
{
    char text[8] = "";
    bool b_reversed = false;
    for (uint32_t i = 0u; i < count; i++)
    {
        widths[i] = p_tenths[i] * 100u;
    }
    barcode_code39_status_t const status = barcode_code39_decode_partial(
        as_stored(widths, count), count, text, sizeof(text), &b_reversed);
    if ('\0' == p_expected[0])
    {
        return status != BARCODE_CODE39_OK;
    }
    return (BARCODE_CODE39_OK == status) && (0 == strcmp(p_expected, text)) &&
           (b_reversed == b_expected_reversed);
}

/* Scans the car logged on 2026-11-06: each failed the strict decoder. */
static void
test_partial (void)
{
    static uint32_t const small_rev[] = {
        1459, 661, 389, 2046, 246, 587,  297, 814,  382, 2014, 1478, 714, 1570,
        698,  373, 663, 460,  677, 1821, 642, 1626, 611, 486,  1978, 635};
    static uint32_t const large_fwd[] = {
        727, 748,  2171, 786, 2123, 854, 631,  823,  2080,
        825, 579,  883,  602, 2342, 563, 835,  2195, 862,
        544, 2408, 542,  902, 2007, 897, 2107, 878,  566};
    static uint32_t const no_frame[] = {1602, 807,  546, 288,  1270, 347,  1139,
                                        413,  329,  419, 636,  710,  294,  717,
                                        1044, 645,  207, 752,  793,  1353, 154,
                                        470,  1096, 320, 1276, 405,  513};
    static uint32_t const too_short[] = {625,  2484, 556,  729,  2049, 723,
                                         2209, 824,  693,  849,  654,  811,
                                         2125, 833,  2368, 2586, 385};
    bool const b_small =
        partial_is(small_rev, sizeof(small_rev) / 4u, "F", true);
    bool const b_large =
        partial_is(large_fwd, sizeof(large_fwd) / 4u, "A", false);
    bool const b_none = partial_is(no_frame, sizeof(no_frame) / 4u, "", false);
    bool const b_short =
        partial_is(too_short, sizeof(too_short) / 4u, "", false);
    check("BT13 logged scans with one damaged end read F and A; one with no "
          "clean '*', or too short, is refused",
          b_small && b_large && b_none && b_short);

    /* Too long to be one character: *AB* must not read as A. */
    char text[8] = "";
    bool b_reversed = false;
    uint32_t const two = encode("*AB*", RATIO_X10, 0u);
    bool const b_long = BARCODE_CODE39_ERR_LENGTH ==
                        barcode_code39_decode_partial(
                            as_stored(widths, two), two, text, sizeof(text), &b_reversed);
    check("BT14 a longer barcode is never read partially", b_long);
}

static void
test_scan_assembly (void)
{
    barcode_scan_t scan;
    char text[16];
    bool b_reversed;
    bool b_early;

    uint32_t const count = encode("*R2D2*", RATIO_X10, 10u);
    uint32_t sent[MAX_WIDTHS];
    memcpy(sent, widths, sizeof(sent));
    /* Start just below the timer wrap. */
    barcode_scan_init(&scan, false, 0xFFFF0000u);
    uint32_t const end =
        feed_scan(&scan, count, 0xFFFF0000u + 500000u, &b_early);
    bool const b_waiting =
        (BARCODE_SCAN_BUSY == barcode_scan_poll(&scan, end + NARROW_US * 3u));
    bool const b_done =
        (BARCODE_SCAN_DONE ==
         barcode_scan_poll(&scan, end + BARCODE_QUIET_MAX_US + 1u));
    bool const b_widths =
        (scan.count == count) &&
        (0 == memcmp(scan.widths, as_stored(sent, count), count * sizeof(uint16_t)));
    bool const b_decoded = (BARCODE_CODE39_OK ==
                            barcode_code39_decode(scan.widths, scan.count, text,
                                                  sizeof(text), &b_reversed)) &&
                           (0 == strcmp(text, "R2D2"));
    check("BT04 edges become widths across the timer wrap; quiet ends "
          "the scan",
          !b_early && b_waiting && b_done && b_widths && b_decoded);

    /* A second symbol right after a quiet zone ends the first at its edge. */
    barcode_scan_reset(&scan);
    barcode_scan_init(&scan, false, 0u);
    uint32_t const first_end = feed_scan(&scan, count, 1000u, &b_early);
    barcode_scan_result_t const next = barcode_scan_edge(
        &scan, first_end + BARCODE_QUIET_MAX_US + 1000u, true);
    bool const b_split = (BARCODE_SCAN_DONE == next) && (scan.count == count);
    barcode_scan_reset(&scan);
    bool const b_restart =
        (BARCODE_SCAN_BUSY ==
         barcode_scan_edge(&scan, first_end + BARCODE_QUIET_MAX_US + 1000u,
                           true)) &&
        barcode_scan_flag(&scan, BARCODE_SCAN_F_ACTIVE) && (0u == scan.count);
    check("BT05 the next symbol's first bar ends the previous scan",
          b_split && b_restart);

    /* Same level twice is a merged glitch; a very long bar ends the scan. */
    barcode_scan_init(&scan, false, 0u);
    (void)barcode_scan_edge(&scan, 1000u, true);
    (void)barcode_scan_edge(&scan, 2000u, true);
    (void)barcode_scan_edge(&scan, 1000u + NARROW_US, false);
    bool const b_glitch = (1u == scan.count) && (*as_stored(&(uint32_t){NARROW_US}, 1u) == scan.widths[0]);
    (void)barcode_scan_edge(&scan, 2000u + NARROW_US * 2u, true);
    bool const b_ended =
        (BARCODE_SCAN_DONE ==
         barcode_scan_poll(&scan, 2000u + NARROW_US * 2u +
                                      BARCODE_QUIET_MAX_US + 1u));
    check("BT06 a repeated level is ignored; an over-long bar ends the scan "
          "at the last bar",
          b_glitch && b_ended && barcode_scan_flag(&scan, BARCODE_SCAN_F_LONG_BLACK) && (1u == scan.count));

    /* Bars merge partway through: the part before still has to reach the
     * decoder, whether the black is still going or has just ended. */
    uint32_t const all = encode("*A*", RATIO_X10, 0u);
    uint32_t const cut = 2u * 10u; /* "*A", their gap, then the next white. */
    bool b_poll_read = false;
    bool b_edge_read = false;
    for (uint32_t pass = 0u; pass < 2u; pass++)
    {
        barcode_scan_init(&scan, false, 0u);
        uint32_t const start = feed_scan(&scan, cut, 1000u, &b_early);
        uint32_t const late = start + BARCODE_QUIET_MAX_US + 1u;
        barcode_scan_result_t const ended =
            (0u == pass) ? barcode_scan_poll(&scan, late)
                         : barcode_scan_edge(&scan, late, false);
        bool const b_read =
            (BARCODE_SCAN_DONE == ended) && barcode_scan_flag(&scan, BARCODE_SCAN_F_LONG_BLACK) &&
            ((cut - 1u) == scan.count) &&
            (BARCODE_CODE39_OK ==
             barcode_code39_decode_partial(scan.widths, scan.count, text,
                                           sizeof(text), &b_reversed)) &&
            (0 == strcmp("A", text));
        b_poll_read = (0u == pass) ? b_read : b_poll_read;
        b_edge_read = (1u == pass) ? b_read : b_edge_read;
    }
    check("BT15 bars merging partway through keep the elements before them",
          b_poll_read && b_edge_read);

    /* The sensor runs onto the tape just after a whole barcode. */
    barcode_scan_init(&scan, false, 0u);
    uint32_t const after = feed_scan(&scan, all + 1u, 1000u, &b_early);
    bool const b_whole =
        (BARCODE_SCAN_DONE ==
         barcode_scan_poll(&scan, after + BARCODE_QUIET_MAX_US + 1u)) &&
        (all == scan.count) &&
        (BARCODE_CODE39_OK ==
         barcode_code39_decode(scan.widths, scan.count, text, sizeof(text),
                               &b_reversed)) &&
        (0 == strcmp("A", text));
    check("BT16 a long black right after a barcode leaves it whole", b_whole);

    /* A scan that ends on a long black starts the next one cleanly. */
    barcode_scan_reset(&scan);
    (void)barcode_scan_edge(&scan, after + BARCODE_QUIET_MAX_US + 2u, false);
    (void)barcode_scan_edge(&scan, after + BARCODE_QUIET_MAX_US + 3u, true);
    check("BT17 the next bar after a long black starts a new scan",
          barcode_scan_flag(&scan, BARCODE_SCAN_F_ACTIVE) && !barcode_scan_flag(&scan, BARCODE_SCAN_F_LONG_BLACK) && (0u == scan.count));
}

static void
test_trigger (void)
{
    barcode_trigger_t trigger;
    uint32_t const gap = BARCODE_TRIGGER_WINDOW_US / BARCODE_TRIGGER_EDGES;

    /* Across the timer wrap, as a barcode at follow speed. */
    uint32_t now = 0xFFFFFF00u;
    unsigned int fired_at = 0u;
    barcode_trigger_init(&trigger);
    for (unsigned int edge = 1u; edge <= 2u * BARCODE_TRIGGER_EDGES; edge++)
    {
        if (barcode_trigger_edge(&trigger, now) && (0u == fired_at))
        {
            fired_at = edge;
        }
        now += gap;
    }
    check("BT18 edges close together are a barcode, on the last one needed",
          BARCODE_TRIGGER_EDGES == fired_at);

    /* Stray marks: as many edges, but spread out. */
    bool b_fired = false;
    now = 0u;
    barcode_trigger_init(&trigger);
    for (unsigned int edge = 0u; edge < 50u; edge++)
    {
        b_fired = barcode_trigger_edge(&trigger, now) || b_fired;
        now += BARCODE_TRIGGER_WINDOW_US / 2u;
    }
    check("BT19 edges further apart are stray marks, however many", !b_fired);

    /* A hold ignores what is left of a barcode, then ends. */
    barcode_trigger_init(&trigger);
    barcode_trigger_hold(&trigger, 1000000u);
    now = 0u;
    b_fired = false;
    while (now < 1000000u)
    {
        b_fired = barcode_trigger_edge(&trigger, now) || b_fired;
        now += gap;
    }
    bool b_after = false;
    for (unsigned int edge = 0u; edge < BARCODE_TRIGGER_EDGES; edge++)
    {
        b_after = barcode_trigger_edge(&trigger, now) || b_after;
        now += gap;
    }
    check("BT20 nothing triggers during a hold; afterwards edges count again",
          !b_fired && b_after);

    /* Edges 10 ms apart, as a barcode would give at full speed. */
    uint32_t const step = 10000u;
    barcode_trigger_init(&trigger);
    uint32_t t = 10000000u;
    barcode_trigger_hump(&trigger, true, t);
    bool b_any = false;
    for (uint32_t i = 0u; i < 8u; i++)
    {
        t += step;
        barcode_trigger_hump(&trigger, true, t);
        b_any = barcode_trigger_edge(&trigger, t) || b_any;
    }
    check("BT21 edges on a hump do not trigger", !b_any);

    /* Off the hump: still held for BARCODE_TRIGGER_HUMP_US, then free. */
    barcode_trigger_hump(&trigger, false, t);
    uint32_t const flat = t;
    b_any = false;
    for (uint32_t i = 0u; i < 4u; i++)
    {
        t += step;
        barcode_trigger_hump(&trigger, false, t);
        b_any = barcode_trigger_edge(&trigger, t) || b_any;
    }
    check("BT22 nor just after it", !b_any);
    t = flat + BARCODE_TRIGGER_HUMP_US + step;
    bool b_fires = false;
    for (uint32_t i = 0u; i < BARCODE_TRIGGER_EDGES; i++)
    {
        t += step;
        barcode_trigger_hump(&trigger, false, t);
        b_fires = barcode_trigger_edge(&trigger, t) || b_fires;
    }
    check("BT23 once past that, a barcode triggers as usual", b_fires);

    /* A hump must not cut short the hold after a slow pass. */
    barcode_trigger_init(&trigger);
    t = 20000000u;
    barcode_trigger_hold(&trigger, t + BARCODE_TRIGGER_HOLDOFF_US);
    barcode_trigger_hump(&trigger, true, t);
    barcode_trigger_hump(&trigger, false, t + step);
    t += BARCODE_TRIGGER_HUMP_US + step;
    b_any = false;
    for (uint32_t i = 0u; i < BARCODE_TRIGGER_EDGES; i++)
    {
        t += step;
        b_any = barcode_trigger_edge(&trigger, t) || b_any;
    }
    check("BT24 a hump never shortens a longer hold", !b_any);

    /* Leaning for good, as if calibrated on a slope: barcodes come back. */
    barcode_trigger_init(&trigger);
    t = 30000000u;
    uint32_t const leaning = t;
    while ((t - leaning) <
           (BARCODE_TRIGGER_HUMP_MAX_US + BARCODE_TRIGGER_HUMP_US + step))
    {
        t += step;
        barcode_trigger_hump(&trigger, true, t);
    }
    b_fires = false;
    for (uint32_t i = 0u; i < BARCODE_TRIGGER_EDGES; i++)
    {
        t += step;
        barcode_trigger_hump(&trigger, true, t);
        b_fires = barcode_trigger_edge(&trigger, t) || b_fires;
    }
    check("BT25 one hump cannot hold the trigger for ever", b_fires);

    /* Level again, then a new hump holds again. */
    barcode_trigger_hump(&trigger, false, t);
    t += step;
    barcode_trigger_hump(&trigger, true, t);
    b_any = false;
    for (uint32_t i = 0u; i < BARCODE_TRIGGER_EDGES; i++)
    {
        t += step;
        barcode_trigger_hump(&trigger, true, t);
        b_any = barcode_trigger_edge(&trigger, t) || b_any;
    }
    check("BT26 the next hump holds it again", !b_any);
}

/* Run @p count widths through the filter and scan as the sensor would
 * give them: every transition a burst of @p burst edges 1..3 us apart,
 * the level read back at random until the last edge; the first transition
 * also flickers for @p flicker_us. */
static barcode_scan_result_t
chatter_scan (barcode_scan_t * p_scan, uint32_t count, uint32_t burst,
              uint32_t flicker_us)
{
    barcode_filter_t filter;
    uint32_t now_us = 1000000u;
    bool b_level = false;
    barcode_filter_init(&filter, false, 0u);
    barcode_scan_init(p_scan, false, 0u);
    for (uint32_t i = 0u; i <= count; i++)
    {
        b_level = !b_level;
        uint32_t t = now_us;
        for (uint32_t edge = 0u; edge < burst; edge++)
        {
            random_state = random_state * 1103515245u + 12345u;
            bool const b_read =
                (edge == burst - 1u) ? b_level : ((random_state >> 16) & 1u);
            bool b_new = false;
            if (barcode_filter_edge(&filter, t, b_read, &b_new))
            {
                (void)barcode_scan_edge(p_scan, t, b_new);
            }
            t += 1u + ((random_state >> 20) % 3u);
        }
        if ((0u == i) && (flicker_us > 0u))
        {
            /* A short white and back, soon after the bar begins. */
            bool b_new = false;
            if (barcode_filter_edge(&filter, t + flicker_us, false, &b_new))
            {
                (void)barcode_scan_edge(p_scan, t + flicker_us, b_new);
            }
            if (barcode_filter_edge(&filter, t + (2u * flicker_us), true,
                                    &b_new))
            {
                (void)barcode_scan_edge(p_scan, t + (2u * flicker_us), b_new);
            }
        }
        now_us += (i < count) ? widths[i] : 0u;
    }
    return barcode_scan_poll(p_scan, now_us + BARCODE_QUIET_MAX_US + 1u);
}

static void
test_chatter (void)
{
    static barcode_scan_t scan;
    char text[BARCODE_MAX_TEXT];
    bool b_reversed = false;
    uint32_t const count = encode("*INF2004*", RATIO_X10, 0u);
    barcode_scan_result_t const done = chatter_scan(&scan, count, 25u, 0u);
    bool const b_count = (count == scan.count);
    barcode_code39_status_t const status = barcode_code39_decode(
        scan.widths, scan.count, text, sizeof(text), &b_reversed);
    check("BT07 chatter on every edge is filtered: the same elements, and "
          "the barcode decodes",
          (BARCODE_SCAN_DONE == done) && b_count &&
              (BARCODE_CODE39_OK == status) && (0 == strcmp("INF2004", text)) &&
              (((uint32_t)scan.narrowest * BARCODE_WIDTH_UNIT_US) >= NARROW_US - 100u));

    /* A speck: black and back inside one burst, then a real bar. */
    barcode_filter_t filter;
    bool b_new = false;
    barcode_filter_init(&filter, false, 0u);
    bool const b_speck =
        barcode_filter_edge(&filter, 100000u, true, &b_new) && b_new;
    bool const b_swallowed =
        !barcode_filter_edge(&filter, 100002u, false, &b_new);
    bool const b_bar =
        barcode_filter_edge(&filter, 200000u, true, &b_new) && b_new;
    bool const b_white =
        barcode_filter_edge(&filter, 240000u, false, &b_new) && !b_new;
    check("BT08 after a speck the levels stay in step with the bars",
          b_speck && b_swallowed && b_bar && b_white);

    /* As on the car: black reads thin, white wide, and one white gap
     * stretches as the speed changes. It must stay one scan. */
    uint32_t const biased = encode("*A*", RATIO_X10, 0u);
    uint32_t widest = 1u;
    for (uint32_t i = 0u; i < biased; i++)
    {
        widths[i] = ((i % 2u) == 0u) ? widths[i] - BAR_BIAS_US
                                     : widths[i] + BAR_BIAS_US;
        widest =
            (((i % 2u) != 0u) && (widths[i] > widths[widest])) ? i : widest;
    }
    widths[widest] = (widths[widest] * 14u) / 10u;
    barcode_scan_result_t const whole = chatter_scan(&scan, biased, 1u, 0u);
    check("BT09 thin bars and a stretched white gap stay one scan and decode",
          (BARCODE_SCAN_DONE == whole) && (biased == scan.count) &&
              (BARCODE_CODE39_OK ==
               barcode_code39_decode(scan.widths, scan.count, text,
                                     sizeof(text), &b_reversed)) &&
              (0 == strcmp("A", text)));

    /* A flicker at the first bar, as on the car: the scan must keep the
     * whole barcode, not end at the first normal white gap. */
    uint32_t const flickered = encode("*F*", RATIO_X10, 0u);
    barcode_scan_result_t const kept =
        chatter_scan(&scan, flickered, 1u, 6000u);
    check("BT12 a flicker at the first bar does not end the scan early",
          (BARCODE_SCAN_DONE == kept) && (flickered == scan.count) &&
              (BARCODE_CODE39_OK ==
               barcode_code39_decode(scan.widths, scan.count, text,
                                     sizeof(text), &b_reversed)) &&
              (0 == strcmp("F", text)));

    /* The same barcode after a dropped scan left the level black. */
    uint32_t const clean = encode("*A*", RATIO_X10, 0u);
    barcode_scan_init(&scan, true, 0u);
    barcode_scan_sync(&scan, false);
    uint32_t const end = feed_scan(&scan, clean, 1000u, &b_reversed);
    (void)end;
    check("BT10 a stale level between scans does not lose the first bar",
          clean == scan.count);

    /* A scan the car failed on: first bar thin, one wide bar short. */
    static uint32_t const car_scan[] = {
        636, 2148, 710,  736, 2083, 814,  2002, 871, 614, 816,
        618, 1457, 1127, 511, 2090, 2456, 526,  964, 657, 889,
        534, 2362, 566,  868, 2123, 889,  1900, 937, 589};
    uint32_t const car_count = sizeof(car_scan) / sizeof(car_scan[0]);
    for (uint32_t i = 0u; i < car_count; i++)
    {
        widths[i] = car_scan[i] * 100u;
    }
    barcode_code39_status_t const car_status = barcode_code39_decode(
        as_stored(widths, car_count), car_count, text, sizeof(text), &b_reversed);
    check("BT11 a real scan reads *F* forwards, comparing bars with bars",
          (BARCODE_CODE39_OK == car_status) && (0 == strcmp("F", text)) &&
              !b_reversed);
}

int
main (void)
{
    test_every_symbol();
    test_jitter();
    test_rejects();
    test_scan_assembly();
    test_trigger();
    test_partial();
    test_chatter();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
