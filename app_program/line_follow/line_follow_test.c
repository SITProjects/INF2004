/**
 * @file line_follow_test.c
 * @brief On-target tests for the line-following subsystem. Built only
 *        with: make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/line_follow/logs/ and record the result in
 * testing/line_follow/TEST_LOG.md. Test IDs: LINE-U01, LINE-U02, ...
 */

#include "line_follow.h"
#include "../test_report.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS 97 /* Not a multiple of the sample interval. */

/* Public function definitions. */

void
line_follow_test (void)
{
    TEST_INIT();

    /*
     * LINE-U01: the line task started (so the ADC device opened) and
     * sleeps between samples instead of busy-looping.
     */
    INT waiting = 0;
    for (INT i = 0; i < SAMPLES; i++)
    {
        T_RTSK rtsk;
        (void)tk_dly_tsk(SAMPLE_GAP_MS);
        if ((tk_ref_tsk(line_follow_tid_sense, &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0u))
        {
            waiting++;
        }
    }
    TEST_CHECK("LINE-U01", waiting >= (SAMPLES - 1),
               "line task runs and sleeps between samples");

    TEST_SUMMARY("line_follow");
}

#endif /* APP_TEST */
