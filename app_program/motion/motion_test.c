/**
 * @file motion_test.c
 * @brief On-target tests for the motion subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/motion/logs/ and record the result in
 * testing/motion/TEST_LOG.md. Test IDs: MOT-U01, MOT-U02, ...
 */

#include "motion.h"
#include "../test_report.h"
#include "motion_config.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define PWM_SLICE_STRIDE 0x14u
#define PWM_SLICE_COUNT 8u
#define MOTOR_WRAP_10KHZ (((UW)SYSCLK * 100u) - 1u) /* clk_sys / 10 kHz - 1 */

/* Private function declarations. */
static BOOL slice_braked (UW pin);

/* Public function definitions. */

void
motion_test (void)
{
    TEST_INIT();

    /* MOT-U01: both motors' slices run at 10 kHz with both inputs low. */
    TEST_CHECK("MOT-U01",
               slice_braked(MOTION_LEFT_MOTOR_PIN_A) &&
                   slice_braked(MOTION_RIGHT_MOTOR_PIN_A),
               "both motors run 10 kHz PWM and are braked");

    TEST_SUMMARY("motion");
}

/* Private function definitions. */

/* The slice of @p pin is enabled, wraps for 10 kHz, and both levels are 0. */
static BOOL
slice_braked (UW pin)
{
    UW base = PWM_BASE + (PWM_SLICE_STRIDE * ((pin / 2u) % PWM_SLICE_COUNT));

    return ((in_w(base + PWM_CHx_CSR) & PWM_CH_CSR_EN) != 0u) &&
           (in_w(base + PWM_CHx_TOP) == MOTOR_WRAP_10KHZ) &&
           (in_w(base + PWM_CHx_CC) == 0u);
}

#endif /* APP_TEST */
