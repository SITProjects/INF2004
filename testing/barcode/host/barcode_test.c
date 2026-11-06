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
    return barcode_code39_decode(widths, count, p_text, size, p_reversed);
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
        (0 == memcmp(scan.widths, sent, count * sizeof(uint32_t)));
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
        scan.b_active && (0u == scan.count);
    check("BT05 the next symbol's first bar ends the previous scan",
          b_split && b_restart);

    /* Same level twice is a merged glitch; a very long bar is no barcode. */
    barcode_scan_init(&scan, false, 0u);
    (void)barcode_scan_edge(&scan, 1000u, true);
    (void)barcode_scan_edge(&scan, 2000u, true);
    (void)barcode_scan_edge(&scan, 1000u + NARROW_US, false);
    bool const b_glitch = (1u == scan.count) && (NARROW_US == scan.widths[0]);
    (void)barcode_scan_edge(&scan, 2000u + NARROW_US * 2u, true);
    (void)barcode_scan_poll(&scan,
                            2000u + NARROW_US * 2u + BARCODE_QUIET_MAX_US + 1u);
    check("BT06 a repeated level is ignored; an over-long bar drops the scan",
          b_glitch && !scan.b_active && (0u == scan.count));
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
              (scan.narrowest_us >= NARROW_US - 100u));

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
        widths, car_count, text, sizeof(text), &b_reversed);
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
    test_chatter();
    printf("%u failure(s)\n", failures);
    return (0u == failures) ? 0 : 1;
}
