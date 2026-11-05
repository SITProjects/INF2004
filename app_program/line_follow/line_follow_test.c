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
#include "line_follow_config.h"
#include "../test_report.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLES 10
#define SAMPLE_GAP_MS 97 /* Not a multiple of the sample interval. */

/* Private function declarations. */
static BOOL pin_is_digital_input (UW pin);

/* Public function definitions. */

void
line_follow_test (void)
{
    TEST_INIT();

    /*
     * LINE-U01: the line task started and sleeps between samples instead
     * of busy-looping.
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

    /* LINE-U03: both DO pins are SIO inputs, not ADC or outputs. */
    TEST_CHECK("LINE-U03",
               pin_is_digital_input(LINE_LEFT_PIN) &&
                   pin_is_digital_input(LINE_RIGHT_PIN),
               "line DO pins are digital inputs");

    TEST_SUMMARY("line_follow");
}

/* Private function definitions. */

/* Input enabled, output disabled, routed to SIO, not driven. */
static BOOL
pin_is_digital_input (UW pin)
{
    UW const pad = in_w(GPIO(pin));
    UW const funcsel = in_w(GPIO_CTRL(pin)) & 0x1Fu;
    return ((pad & GPIO_IE) != 0u) && ((pad & GPIO_OD) != 0u) &&
           (GPIO_CTRL_FUNCSEL_SIO == funcsel) &&
           ((in_w(GPIO_OE) & (1u << pin)) == 0u);
}

#endif /* APP_TEST */
