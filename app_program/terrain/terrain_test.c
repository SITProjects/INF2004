/**
 * @file terrain_test.c
 * @brief On-target tests for the terrain subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/terrain/logs/ and record the result in
 * testing/terrain/TEST_LOG.md. Test IDs: TER-U01, TER-U02, ...
 *
 * All tests need the GY-511 connected; TER-U02 and TER-U04 also need the
 * car standing still on level ground during power-up. No test waits for
 * a person: every wait is bounded.
 *
 * The bench tests TER-U06 and TER-U07 are added only with
 * TERRAIN_TEST_BENCH set to 1 in terrain_config.h. They print what to do
 * and count down before each step; the console cannot be read without
 * stopping the kernel, so they never wait for a key.
 */

#include "terrain.h"
#include "../test_report.h"
#include "terrain_config.h"
#include "terrain_monitor.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLE_WINDOW_MS 500
#define MIN_SAMPLES 25u /* Half the 50 expected at 100 Hz in 500 ms. */
#define CALIB_TIMEOUT_MS 5000
#define POLL_MS 100
#define TILT_SETTLE_MS 500 /* Over three smoothing time constants. */
#define LEVEL_TOL_DEG 1.0f

#if TERRAIN_TEST_BENCH
#define COUNTDOWN_S 5
#define ONE_S_MS 1000
#define STILL_TIMEOUT_MS 10000 /* Settle period plus several attempts. */
#define FLAT_MIN_Z_MPS2 5.0f   /* Over half of gravity on Z. */
#define LIFT_MIN_MPS2 3.0f     /* Change from flat for an 18 degree lift. */
#define MILLI_PER_UNIT 1000
#endif

/* Private function declarations. */
#if TERRAIN_TEST_BENCH
static bool bench_step (char const * p_instruction,
                        terrain_calib_info_t * p_out);
static void countdown (void);
static bool measure_still (terrain_calib_info_t * p_out);
static void print_ref (char const * p_axis, float value_mps2);
static void print_milli (float value);
#endif

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

    /* TER-U04: still and level after calibration, so tilt reads 0. */
    terrain_data_t rest = {0};
    (void)tk_dly_tsk(TILT_SETTLE_MS);
    terrain_status_t status = terrain_monitor_get_latest(&rest);
    TEST_CHECK("TER-U04",
               (status == TERRAIN_OK) && (rest.pitch_deg < LEVEL_TOL_DEG) &&
                   (rest.pitch_deg > -LEVEL_TOL_DEG) &&
                   (rest.roll_deg < LEVEL_TOL_DEG) &&
                   (rest.roll_deg > -LEVEL_TOL_DEG),
               "pitch and roll within 1 degree of 0 at rest");

#if TERRAIN_TEST_BENCH
    /* TER-U06: record the level reference; replaces the old L key. */
    terrain_calib_info_t level = {0};
    bool const b_level = bench_step(
        "TER-U06: keep the car still on a floor checked to be flat", &level);
    if (b_level)
    {
        tm_printf((UB *)"terrain: paste into terrain_config.h, rebuild:\n");
        print_ref("X", level.gravity_x_mps2);
        print_ref("Y", level.gravity_y_mps2);
        print_ref("Z", level.gravity_z_mps2);
    }
    TEST_CHECK("TER-U06", b_level, "level reference recorded");

    /* TER-U07: each car axis points the right way after mounting. */
    terrain_calib_info_t flat = {0};
    terrain_calib_info_t nose = {0};
    terrain_calib_info_t side = {0};
    bool const b_flat =
        bench_step("TER-U07 1/3: leave the car flat and still", &flat);
    bool const b_nose = bench_step(
        "TER-U07 2/3: lift the nose about 30 degrees and hold still", &nose);
    bool const b_side = bench_step(
        "TER-U07 3/3: put the nose down, lift the right side about 30 "
        "degrees and hold still",
        &side);
    float const rise_y_mps2 = nose.gravity_y_mps2 - flat.gravity_y_mps2;
    float const rise_x_mps2 = side.gravity_x_mps2 - flat.gravity_x_mps2;
    TEST_CHECK("TER-U07", b_flat && (flat.gravity_z_mps2 > FLAT_MIN_Z_MPS2),
               "flat: Z points up (else check TERRAIN_CAR_Z_FROM/SIGN)");
    TEST_CHECK("TER-U07", b_flat && b_nose && (rise_y_mps2 > LIFT_MIN_MPS2),
               "nose up: Y rises (else check TERRAIN_CAR_Y_FROM/SIGN)");
    TEST_CHECK("TER-U07", b_flat && b_side && (rise_x_mps2 > LIFT_MIN_MPS2),
               "right side up: X rises (else check TERRAIN_CAR_X_FROM/SIGN)");

    /* Leave the car calibrated for whatever runs next. */
    tm_printf((UB *)"terrain: put the car back flat and still\n");
    countdown();
    (void)terrain_monitor_recalibrate();
#endif

    TEST_SUMMARY("terrain");
}

/* Private function definitions. */

#if TERRAIN_TEST_BENCH
/*
 * Print the instruction, give the person time to follow it, then take
 * one still reading and print it. Returns false if none came in time.
 */
static bool
bench_step (char const * p_instruction, terrain_calib_info_t * p_out)
{
    tm_printf((UB *)"terrain: %s\n", p_instruction);
    countdown();

    bool const b_still = measure_still(p_out);
    if (!b_still)
    {
        tm_printf((UB *)"terrain: no still reading in time\n");
        return false;
    }

    tm_printf((UB *)"terrain: X ");
    print_milli(p_out->gravity_x_mps2);
    tm_printf((UB *)" Y ");
    print_milli(p_out->gravity_y_mps2);
    tm_printf((UB *)" Z ");
    print_milli(p_out->gravity_z_mps2);
    tm_printf((UB *)" m/s^2\n");
    return true;
}

static void
countdown (void)
{
    tm_printf((UB *)"terrain: starting in");
    for (INT left = COUNTDOWN_S; left > 0; left--)
    {
        tm_printf((UB *)" %d", left);
        (void)tk_dly_tsk(ONE_S_MS);
    }
    tm_printf((UB *)"; hold still\n");
}

/*
 * Start a calibration and wait for one attempt in which the car did not
 * move. Its average is in p_out whether the result was VALID or
 * NOT_LEVEL, so this works with a level reference that is out of date.
 */
static bool
measure_still (terrain_calib_info_t * p_out)
{
    terrain_calib_info_t const empty = {0};
    *p_out = empty;
    if (terrain_monitor_recalibrate() != TERRAIN_OK)
    {
        return false;
    }

    for (INT waited = 0; waited < STILL_TIMEOUT_MS; waited += POLL_MS)
    {
        (void)tk_dly_tsk(POLL_MS);
        if ((terrain_monitor_get_calibration(p_out) == TERRAIN_OK) &&
            (p_out->attempt_count > p_out->retry_count))
        {
            return true;
        }
    }
    return false;
}

/* One line in the form used in terrain_config.h. */
static void
print_ref (char const * p_axis, float value_mps2)
{
    tm_printf((UB *)"#define TERRAIN_LEVEL_REF_%s (", p_axis);
    print_milli(value_mps2);
    tm_printf((UB *)"f)\n");
}

/* tm_printf has no %f: print with three decimals. */
static void
print_milli (float value)
{
    float const rounding = (value < 0.0f) ? -0.5f : 0.5f;
    INT milli = (INT)((value * (float)MILLI_PER_UNIT) + rounding);
    char const * p_sign = "";
    if (milli < 0)
    {
        p_sign = "-";
        milli = -milli;
    }

    tm_printf((UB *)"%s%d.%03d", p_sign, milli / MILLI_PER_UNIT,
              milli % MILLI_PER_UNIT);
}
#endif

#endif /* APP_TEST */
