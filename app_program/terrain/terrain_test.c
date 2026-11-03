/**
 * @file terrain_test.c
 * @brief On-target tests for the terrain subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/terrain/logs/ and record the result in
 * testing/terrain/TEST_LOG.md. Test IDs: TER-U01, TER-U02, ...
 *
 * TER-U12 runs the hump state machine and event queue on made-up data, and
 * TER-U15 the hump height on made-up humps; both need only the Pico. The
 * other tests need the GY-511 connected; TER-U02, TER-U04 and TER-U13
 * also need the car standing still on level ground during power-up. No
 * test waits for a person: every wait is bounded.
 *
 * The bench tests TER-U06 and TER-U07 are added only with
 * TERRAIN_TEST_BENCH set to 1 in terrain_config.h. They print what to do
 * and count down before each step; the console cannot be read without
 * stopping the kernel, so they never wait for a key.
 */

#include "terrain.h"
#include "../test_report.h"
#include "terrain_config.h"
#include "terrain_detect.h"
#include "terrain_event.h"
#include "terrain_height.h"
#include "terrain_monitor.h"
#include "terrain_process.h"

#if APP_TEST

/* Private types, constants, and macros. */
#define SAMPLE_WINDOW_MS 500
#define MIN_SAMPLES 25u /* Half the 50 expected at 100 Hz in 500 ms. */
#define CALIB_TIMEOUT_MS 5000
#define POLL_MS 100
#define TILT_SETTLE_MS 500 /* Over three smoothing time constants. */
#define LEVEL_TOL_DDEG 10  /* 1 degree. */

/* TER-U12 trace: pitch well past the 4 degree limits, in degrees. */
#define TRACE_UP_DEG 8.0f
#define TRACE_DOWN_DEG (-8.0f)
#define TRACE_FLAT_DEG 0.0f
#define TRACE_UP_DDEG 80
#define DDEG_PER_DEG 10.0f
#define TRACE_EVENTS_MAX 8u

/* TER-U15 humps, driven at 2 mm a sample (200 mm/s). */
#define HUMP_STEP_MM 2.0f
#define HUMP_TOL_MM 3 /* 40 mm humps; 80 mm ones get HUMP_TALL_TOL_MM. */
#define HUMP_TALL_TOL_MM 4
#define HUMP_LEAD_MM 100.0f
#define HUMP_TAIL_MM 400.0f
#define HUMP_TILT_ONLY_MAX_MM 60 /* Long ramps read low without distance. */

#if TERRAIN_TEST_BENCH
#define COUNTDOWN_S 5
#define ONE_S_MS 1000
#define STILL_TIMEOUT_MS 10000 /* Settle period plus several attempts. */
#define FLAT_MIN_Z_MPS2 5.0f   /* Over half of gravity on Z. */
#define LIFT_MIN_MPS2 3.0f     /* Change from flat for an 18 degree lift. */
#define MILLI_PER_UNIT 1000
#endif

/** @brief One stretch of the TER-U12 pitch trace. */
typedef struct
{
    float pitch_deg;  /**< Pitch for the whole stretch. */
    uint32_t samples; /**< Length, samples. */
} trace_part_t;

/** @brief A TER-U15 trapezoid hump, mm. */
typedef struct
{
    float ramp_mm;   /**< Length of each ramp. */
    float top_mm;    /**< Length of the flat top. */
    float height_mm; /**< Height of the top. */
} hump_shape_t;

/* Private function declarations. */
static bool hump_trace_ok (void);
static bool hump_heights_ok (void);
static int32_t drive_hump (hump_shape_t const * p_shape, bool b_known);
static float hump_ground_mm (hump_shape_t const * p_shape, float x_mm);
static float root (float square);
static bool queue_drops_oldest (void);
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
                   ((uint16_t)(terrain_monitor_data_sample_count(&after) -
                               terrain_monitor_data_sample_count(&before)) >=
                    MIN_SAMPLES),
               "IMU delivers samples at about 100 Hz");

    /* TER-U02: calibration becomes VALID within 5 s of power-up. */
    terrain_calib_info_t info = {0};
    for (INT waited = 0; waited < CALIB_TIMEOUT_MS; waited += POLL_MS)
    {
        if ((terrain_monitor_get_calibration(&info) == TERRAIN_OK) &&
            (terrain_monitor_calib_state(&info) == TERRAIN_CALIB_VALID))
        {
            break;
        }
        (void)tk_dly_tsk(POLL_MS);
    }
    TEST_CHECK("TER-U02",
               terrain_monitor_calib_state(&info) == TERRAIN_CALIB_VALID,
               "calibration VALID with the car still and level");

    /* TER-U04: still and level after calibration, so tilt reads 0. */
    terrain_data_t rest = {0};
    (void)tk_dly_tsk(TILT_SETTLE_MS);
    terrain_status_t status = terrain_monitor_get_latest(&rest);
    INT const pitch_ddeg = terrain_monitor_data_pitch_ddeg(&rest);
    INT const roll_ddeg = terrain_monitor_data_roll_ddeg(&rest);
    TEST_CHECK("TER-U04",
               (status == TERRAIN_OK) && (pitch_ddeg < LEVEL_TOL_DDEG) &&
                   (pitch_ddeg > -LEVEL_TOL_DDEG) &&
                   (roll_ddeg < LEVEL_TOL_DDEG) &&
                   (roll_ddeg > -LEVEL_TOL_DDEG),
               "pitch and roll within 1 degree of 0 at rest");

    /* TER-U12: hump machine and queue on made-up data (no IMU needed). */
    TEST_CHECK("TER-U12", hump_trace_ok(),
               "hump trace gives ASCENDING, DESCENDING, LEVEL only");
    TEST_CHECK("TER-U12", queue_drops_oldest(),
               "full event queue drops the oldest event");

    /* TER-U15: hump height on made-up humps (no IMU needed). */
    TEST_CHECK("TER-U15", hump_heights_ok(),
               "40 mm narrow and 80 mm ramped humps read their height");

    /* TER-U13: still and level, so no hump and no events. */
    terrain_data_t still = {0};
    terrain_event_t event = {0u};
    terrain_status_t const data_status = terrain_monitor_get_latest(&still);
    terrain_status_t const event_status = terrain_monitor_poll_event(&event);
    TEST_CHECK(
        "TER-U13",
        (data_status == TERRAIN_OK) &&
            (terrain_monitor_data_hump_state(&still) == TERRAIN_HUMP_LEVEL) &&
            (event_status == TERRAIN_OK) &&
            (terrain_monitor_event_type(&event) == TERRAIN_EVENT_NONE),
        "hump LEVEL and no events at rest");

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

/*
 * TER-U12: a 4-sample spike (ignored), then one hump. Only ASCENDING,
 * DESCENDING and LEVEL may come out, in that order, with the climb's
 * pitch as its value.
 */
static bool
hump_trace_ok (void)
{
    static trace_part_t const trace[] = {
        {TRACE_FLAT_DEG, 20u}, {TRACE_UP_DEG, TERRAIN_HUMP_ENTER_SAMPLES - 1u},
        {TRACE_FLAT_DEG, 10u}, {TRACE_UP_DEG, 15u},
        {TRACE_FLAT_DEG, 10u}, {TRACE_DOWN_DEG, 15u},
        {TRACE_FLAT_DEG, 20u}};
    static terrain_event_type_t const expected[] = {
        TERRAIN_EVENT_ASCENDING, TERRAIN_EVENT_DESCENDING, TERRAIN_EVENT_LEVEL};
    terrain_hump_limits_t const limits = {
        TERRAIN_HUMP_UP_DEG,        TERRAIN_HUMP_TOP_DEG,
        TERRAIN_HUMP_DOWN_DEG,      TERRAIN_HUMP_LEVEL_DEG,
        TERRAIN_HUMP_ENTER_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};

    terrain_event_queue_t queue;
    terrain_event_queue_reset(&queue);
    uint32_t word = 0u;
    uint32_t tick = 0u;
    for (uint32_t part = 0u; part < (sizeof(trace) / sizeof(trace[0])); part++)
    {
        for (uint32_t i = 0u; i < trace[part].samples; i++)
        {
            terrain_event_type_t const type =
                terrain_detect_hump_step(&word, trace[part].pitch_deg, &limits);
            uint32_t const value = terrain_process_encode_signed(
                trace[part].pitch_deg, DDEG_PER_DEG, TERRAIN_EVENT_VALUE_BITS);
            terrain_event_queue_push(&queue,
                                     terrain_event_make(type, value, tick));
            tick++;
        }
    }

    uint32_t const n = sizeof(expected) / sizeof(expected[0]);
    bool b_ok = (terrain_event_queue_count(&queue) == n) &&
                (terrain_detect_hump_state(word) == TERRAIN_HUMP_LEVEL);
    for (uint32_t i = 0u; i < n; i++)
    {
        terrain_event_t const event = terrain_event_queue_pop(&queue);
        b_ok = b_ok && (terrain_event_type(event) == expected[i]);
        if (i == 0u)
        {
            b_ok = b_ok && (terrain_event_value(event) == TRACE_UP_DDEG);
        }
    }
    return b_ok && (terrain_event_type(terrain_event_queue_pop(&queue)) ==
                    TERRAIN_EVENT_NONE);
}

/*
 * TER-U12: one event more than the queue holds drops the first one;
 * the rest come out in order, then NONE.
 */
static bool
queue_drops_oldest (void)
{
    terrain_event_queue_t queue;
    terrain_event_queue_reset(&queue);
    for (uint32_t i = 0u; i <= TERRAIN_EVENT_QUEUE_LEN; i++)
    {
        terrain_event_queue_push(&queue,
                                 terrain_event_make(TERRAIN_EVENT_LEVEL, i, i));
    }

    bool b_ok =
        (terrain_event_queue_count(&queue) == TERRAIN_EVENT_QUEUE_LEN) &&
        (terrain_event_queue_dropped(&queue) == 1u);
    for (uint32_t i = 1u; i <= TERRAIN_EVENT_QUEUE_LEN; i++)
    {
        terrain_event_t const event = terrain_event_queue_pop(&queue);
        b_ok = b_ok && (terrain_event_value(event) == (int32_t)i);
    }
    return b_ok && (terrain_event_queue_count(&queue) == 0u) &&
           (terrain_event_type(terrain_event_queue_pop(&queue)) ==
            TERRAIN_EVENT_NONE);
}

/*
 * TER-U15: a 40 mm hump narrower than the car (tilt estimate) and an
 * 80 mm one with long ramps (distance estimate) read their height within
 * HUMP_TOL_MM and HUMP_TALL_TOL_MM. Without the distance the long-ramp
 * hump reads clearly low, as documented.
 */
static bool
hump_heights_ok (void)
{
    static hump_shape_t const narrow = {20.0f, 20.0f, 40.0f};
    static hump_shape_t const ramped = {200.0f, 300.0f, 80.0f};

    int32_t const narrow_mm = drive_hump(&narrow, true);
    int32_t const ramped_mm = drive_hump(&ramped, true);
    int32_t const tilt_only_mm = drive_hump(&ramped, false);

    return (narrow_mm >= (40 - HUMP_TOL_MM)) &&
           (narrow_mm <= (40 + HUMP_TOL_MM)) &&
           (ramped_mm >= (80 - HUMP_TALL_TOL_MM)) &&
           (ramped_mm <= (80 + HUMP_TALL_TOL_MM)) && (tilt_only_mm >= 0) &&
           (tilt_only_mm < HUMP_TILT_ONLY_MAX_MM);
}

/*
 * Drive the car over one hump: the rear wheel at x, the front one a
 * wheelbase ahead along the car, pitch from the two heights through the
 * module's own pitch maths (gravity along the car's slope). As in the
 * monitor, the hump machine gets the pitch with the tilt smoothing and
 * the height the unsmoothed pitch (it smooths it itself). Returns the
 * LEVEL height, -1 if none.
 */
static int32_t
drive_hump (hump_shape_t const * p_shape, bool b_known)
{
    terrain_hump_limits_t const hump_limits = {
        TERRAIN_HUMP_UP_DEG,        TERRAIN_HUMP_TOP_DEG,
        TERRAIN_HUMP_DOWN_DEG,      TERRAIN_HUMP_LEVEL_DEG,
        TERRAIN_HUMP_ENTER_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};
    terrain_height_limits_t const height_limits = {
        (float)TERRAIN_WHEELBASE_MM, TERRAIN_HUMP_LEVEL_DEG,
        (float)TERRAIN_HEIGHT_MAX_STEP_MM,
        TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_HUMP_HEIGHT_TAU_S,
                                      TERRAIN_SAMPLE_PERIOD_S)};
    float const tilt_alpha = TERRAIN_PROCESS_LOWPASS_ALPHA(
        TERRAIN_TILT_TAU_S, TERRAIN_SAMPLE_PERIOD_S);
    float const wheelbase_mm = (float)TERRAIN_WHEELBASE_MM;
    float const end_mm = HUMP_LEAD_MM + (2.0f * p_shape->ramp_mm) +
                         p_shape->top_mm + wheelbase_mm + HUMP_TAIL_MM;

    terrain_height_t height;
    terrain_height_reset(&height);
    uint32_t word = 0u;
    float tilt_deg = 0.0f;
    int32_t result = -1;
    for (float x_mm = 0.0f; x_mm < end_mm; x_mm += HUMP_STEP_MM)
    {
        /* The front wheel is a wheelbase away along the tilted car, so a
         * little less than that ahead: a few passes find where it rests. */
        float const rear_mm = hump_ground_mm(p_shape, x_mm);
        float run_mm = wheelbase_mm;
        float rise_mm = 0.0f;
        for (uint32_t pass = 0u; pass < 3u; pass++)
        {
            rise_mm = hump_ground_mm(p_shape, x_mm + run_mm) - rear_mm;
            run_mm = root((wheelbase_mm * wheelbase_mm) - (rise_mm * rise_mm));
        }
        terrain_vec3_t const slope = {0.0f, rise_mm, run_mm};

        /* The encoders measure along the ground, not across it. */
        float const climb_mm =
            hump_ground_mm(p_shape, x_mm + HUMP_STEP_MM) - rear_mm;
        float const ground_mm =
            root((HUMP_STEP_MM * HUMP_STEP_MM) + (climb_mm * climb_mm));
        float const pitch_deg = terrain_process_pitch_deg(&slope);
        tilt_deg += tilt_alpha * (pitch_deg - tilt_deg);
        terrain_height_step(&height, terrain_detect_hump_state(word), pitch_deg,
                            ground_mm, b_known, &height_limits);
        if ((terrain_detect_hump_step(&word, tilt_deg, &hump_limits) ==
             TERRAIN_EVENT_LEVEL) &&
            (result < 0))
        {
            result = (int32_t)terrain_height_mm(&height);
        }
    }
    return result;
}

/* Ground height under x for a TER-U15 hump starting at HUMP_LEAD_MM. */
static float
hump_ground_mm (hump_shape_t const * p_shape, float x_mm)
{
    float const up_end_mm = HUMP_LEAD_MM + p_shape->ramp_mm;
    float const top_end_mm = up_end_mm + p_shape->top_mm;
    float const down_end_mm = top_end_mm + p_shape->ramp_mm;

    float ground_mm = 0.0f;
    if ((x_mm <= HUMP_LEAD_MM) || (x_mm >= down_end_mm))
    {
        ground_mm = 0.0f;
    }
    else if (x_mm < up_end_mm)
    {
        ground_mm =
            p_shape->height_mm * (x_mm - HUMP_LEAD_MM) / p_shape->ramp_mm;
    }
    else if (x_mm <= top_end_mm)
    {
        ground_mm = p_shape->height_mm;
    }
    else
    {
        ground_mm =
            p_shape->height_mm * (down_end_mm - x_mm) / p_shape->ramp_mm;
    }
    return ground_mm;
}

/*
 * Square root by Newton's method, for TER-U15's car geometry: the test
 * file must not include math.h after the kernel headers. Good to well
 * under 0.01 mm for the lengths used here; 0 for 0 or less.
 */
static float
root (float square)
{
    if (square <= 0.0f)
    {
        return 0.0f;
    }
    float value = (square > 1.0f) ? square : 1.0f;
    for (uint32_t i = 0u; i < 20u; i++)
    {
        value = 0.5f * (value + (square / value));
    }
    return value;
}

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
 * move: the state then leaves CALIBRATING. Its average is in p_out whether
 * the result was VALID or NOT_LEVEL, so this works with a level reference
 * that is out of date.
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
            (terrain_monitor_calib_state(p_out) != TERRAIN_CALIB_CALIBRATING))
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
