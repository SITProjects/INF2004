/**
 * @file subsystem_test.c
 * @brief On-target tests for the subsystem. Built only with: make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/<name>/logs/ and record the result in
 * testing/<name>/TEST_LOG.md. Test IDs: XXX-U01, XXX-U02, ...
 */

#include "subsystem.h"
#include "../test_report.h"

#if APP_TEST

/* Public function definitions. */

void
subsystem_test (void)
{
    TEST_INIT();

    /* Replace with real checks of this subsystem's behaviour. */
    TEST_CHECK("XXX-U01", tk_get_tid() > 0,
               "example: running in a task context");

    TEST_SUMMARY("subsystem");
}

#endif /* APP_TEST */
