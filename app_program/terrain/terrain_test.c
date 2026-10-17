/**
 * @file terrain_test.c
 * @brief On-target tests for the terrain subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/terrain/logs/ and record the result in
 * testing/terrain/TEST_LOG.md. Test IDs: TER-U01, TER-U02, ...
 *
 * Both tests need the GY-511 connected; TER-U02 also needs the car
 * standing still on level ground during power-up.
 */

#include "terrain.h"
#include "../test_report.h"
#include "terrain_monitor.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLE_WINDOW_MS 500
#define MIN_SAMPLES 25u /* Half the 50 expected at 100 Hz in 500 ms. */
#define CALIB_TIMEOUT_MS 5000
#define POLL_MS 100

/* Public function definitions. */

void
terrain_test (void)
{
    TEST_INIT();

    /* TER-U01: the sensor is set up and samples arrive at about 100 Hz. */
    terrain_data_t before = {0};
    terrain_data_t after = {0};
    terrain_status_t first = terrain_monitor_get_latest(&before);
    (void)tk_dly_tsk(SAMPLE_WINDOW_MS);
    terrain_status_t second = terrain_monitor_get_latest(&after);
    TEST_CHECK("TER-U01",
               (first != TERRAIN_ERR_NOT_INIT) &&
                   (second != TERRAIN_ERR_NOT_INIT) &&
                   ((after.sample_count - before.sample_count) >= MIN_SAMPLES),
               "IMU delivers samples at about 100 Hz");

    /* TER-U02: calibration becomes VALID within 5 s of power-up. */
    terrain_calib_info_t info = {0};
    for (INT waited = 0; waited < CALIB_TIMEOUT_MS; waited += POLL_MS)
    {
        if ((terrain_monitor_get_calibration(&info) == TERRAIN_OK) &&
            (info.state == TERRAIN_CALIB_VALID))
        {
            break;
        }
        (void)tk_dly_tsk(POLL_MS);
    }
    TEST_CHECK("TER-U02", info.state == TERRAIN_CALIB_VALID,
               "calibration VALID with the car still and level");

    TEST_SUMMARY("terrain");
}

#endif /* APP_TEST */
