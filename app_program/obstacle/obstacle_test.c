/**
 * @file obstacle_test.c
 * @brief On-target tests for the obstacle subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/obstacle/logs/ and record the result in
 * testing/obstacle/TEST_LOG.md. Test IDs: OBS-U01, OBS-U02, ...
 */

#include "obstacle.h"
#include "../test_report.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS 97 /* Not a multiple of the scan period. */
#define PWM_SLICE_STRIDE 0x14u
#define SERVO_SLICE (((UW)OBSTACLE_SERVO_PIN / 2u) % 8u)
#define SERVO_WRAP 19999u
#define SERVO_DIV_1MHZ ((UW)SYSCLK << 4) /* clk_sys MHz in 8.4 fixed point. */

/* Public function definitions. */

void
obstacle_test (void)
{
    TEST_INIT();

    /*
     * OBS-U01: the scan task started (so obstacle_scan_init() succeeded)
     * and sleeps between scans instead of busy-looping.
     */
    INT waiting = 0;
    for (INT i = 0; i < SAMPLES; i++)
    {
        T_RTSK rtsk;
        (void)tk_dly_tsk(SAMPLE_GAP_MS);
        if ((tk_ref_tsk(obstacle_tid_scan, &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0u))
        {
            waiting++;
        }
    }
    TEST_CHECK("OBS-U01", waiting >= (SAMPLES - 1),
               "scan task runs and sleeps between scans");

    /* OBS-U02: the servo slice runs a 1 MHz counter that wraps at 20 ms. */
    UW base = PWM_BASE + (PWM_SLICE_STRIDE * SERVO_SLICE);
    TEST_CHECK("OBS-U02",
               ((in_w(base + PWM_CHx_CSR) & PWM_CH_CSR_EN) != 0u) &&
                   (in_w(base + PWM_CHx_TOP) == SERVO_WRAP) &&
                   (in_w(base + PWM_CHx_DIV) == SERVO_DIV_1MHZ),
               "servo PWM slice runs at 50 Hz");

    TEST_SUMMARY("obstacle");
}

#endif /* APP_TEST */
