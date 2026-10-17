/**
 * @file test_report.h
 * @brief Minimal on-target test helpers.
 *
 * Every check prints ONE line in a fixed format, so a saved console log
 * can be searched for results ("grep '\[TEST\]' log.txt"):
 *
 *     [TEST] OBS-U01 PASS  scan task sleeps between scans
 *     [TEST] OBS-U02 FAIL  servo PWM at 50 Hz   (obstacle_test.c:57)
 *     [TEST] obstacle SUMMARY 1 passed, 1 failed
 *
 * Usage, inside a test function:
 *     TEST_INIT();
 *     TEST_CHECK("OBS-U01", value >= 0 && value <= 4095, "ADC in range");
 *     TEST_SUMMARY("obstacle");
 *
 * Test IDs must match the ones in testing/<name>/TEST_LOG.md.
 *
 * These are macros rather than functions because TEST_INIT() declares the
 * counters in the calling function and TEST_CHECK() reports the caller's
 * file and line.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

/* Public includes. */
#include <tm/tmonitor.h>

/* Public types, constants, and macros. */

/** Declare the pass and fail counters; use once at the top of a test. */
#define TEST_INIT() INT test_pass_ = 0, test_fail_ = 0

/** Check one condition and print a PASS or FAIL line for test @p id. */
#define TEST_CHECK(id, cond, desc)                                             \
    do                                                                         \
    {                                                                          \
        if (cond)                                                              \
        {                                                                      \
            test_pass_++;                                                      \
            tm_printf((UB *)"[TEST] %s PASS  %s\n", (id), (desc));             \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            test_fail_++;                                                      \
            tm_printf((UB *)"[TEST] %s FAIL  %s   (%s:%d)\n", (id), (desc),    \
                      __FILE__, __LINE__);                                     \
        }                                                                      \
    } while (0)

/** Print how many checks passed and failed for subsystem @p name. */
#define TEST_SUMMARY(name)                                                     \
    tm_printf((UB *)"[TEST] %s SUMMARY %d passed, %d failed\n", (name),        \
              test_pass_, test_fail_)

/* Public function declarations. */

#endif /* TEST_REPORT_H */
