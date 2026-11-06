/**
 * @file terrain_test.c
 * @brief On-target tests for the terrain subsystem. Built only with:
 *        make TEST=1
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/terrain/logs/ and record the result in
 * testing/terrain/TEST_LOG.md. Test IDs: TER-U01, TER-U02, ...
 *
 * TER-U12 runs the hump state machine and event queue on made-up data,
 * TER-U15 the hump height on made-up one-wheel ramps, TER-U19 the
 * collision state machine on made-up pushes and speeds, TER-U22 the
 * turn rate and turning state machine on made-up wheel speeds and turn
 * rates, and TER-U24 the speed state machine on made-up wheel speeds;
 * they need only the Pico. The other tests need the GY-511 connected;
 * TER-U02, TER-U04 and TER-U13 also need the car standing still on level
 * ground during power-up. No test waits for a person: every wait is bounded.
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

/* TER-U12 trace: roll well past the lean limits, in degrees. The climb
 * leans right (negative roll) to show that either side counts. */
#define TRACE_UP_DEG (-8.0f)
#define TRACE_DOWN_DEG (-4.0f) /* 4 degrees below the peak. */
#define TRACE_FLAT_DEG 0.0f
#define TRACE_UP_DDEG 80 /* The lean: the size of the roll. */
#define DDEG_PER_DEG 10.0f

/* TER-U19 pushes (g) and speeds (mm/s): line-following speed, a stop,
 * pushes below the limit, between the limit and sure, and sure. */
#define CRASH_CRUISE_MM_S 120
#define CRASH_STOP_MM_S 40
#define CRASH_SOFT_G 0.8f
#define CRASH_HIT_G 1.5f
#define CRASH_BIG_G 1.8f
#define CRASH_SURE_G 2.5f
#define CRASH_HIT_DG 15u
#define CRASH_BIG_DG 18u
#define CRASH_SURE_DG 25u
#define CRASH_WAIT 3u  /* Samples before the speed drops. */
#define CRASH_LONG 20u /* Longer than the 12-sample check. */
#define CRASH_GAP 60u  /* Longer than the 50-sample cooldown. */
#define DG_PER_G 10.0f

/* TER-U22 wheel speeds (mm/s) and turn rates (deg/s): a gentle curve
 * (right 160, left 80: 41.4 deg/s left), a turn in place at motion's
 * 90 deg/s (wheels at +/-87 mm/s: 90.1 deg/s), and rates below the off
 * limit, between the limits, above the on limit and a turn in place. */
#define TURN_CURVE_LEFT_MM_S 80
#define TURN_CURVE_RIGHT_MM_S 160
#define TURN_CURVE_DPS_X10 414 /* Expected, tenths of a deg/s. */
#define TURN_SPIN_MM_S 87
#define TURN_SPIN_DPS_X10 901
#define TURN_TOL_X10 5 /* 0.5 deg/s. */
#define TURN_STILL_DPS 0.0f
#define TURN_CREEP_DPS 5.0f
#define TURN_MID_DPS 15.0f
#define TURN_SLOW_DPS (-25.0f) /* A slow right curve. */
#define TURN_FAST_DPS 90.0f
#define TURN_EVENTS_MAX 8u
#define TURN_HUMP_BITS 0x0002ABCDu /* Made-up hump fields, bits 0-17. */
#define TURN_HUMP_MASK 0x0003FFFFu

/* TER-U24 wheel speeds (mm/s): stopped, a blip, line-following speed
 * (120) reached in 20 mm/s steps, one noisy check, and slowing down. */
#define SPEED_STOP_MM_S 0.0f
#define SPEED_BLIP_MM_S 100.0f
#define SPEED_CRUISE_MM_S 120.0f
#define SPEED_NOISE_MM_S 140.0f
#define SPEED_EVENTS_MAX 8u

/* TER-U15 ramps under one wheel, driven at 1.2 mm a sample (120 mm/s,
 * the line-following speed). */
#define HUMP_STEP_MM 1.2f
#define HUMP_TOL_MM 2
#define HUMP_LEAD_MM 100.0f
#define HUMP_TAIL_MM 200.0f

#if TERRAIN_TEST_BENCH
#define COUNTDOWN_S 5
#define ONE_S_MS 1000
#define STILL_TIMEOUT_MS 10000 /* Settle period plus several attempts. */
#define FLAT_MIN_Z_MPS2 5.0f   /* Over half of gravity on Z. */
#define LIFT_MIN_MPS2 3.0f     /* Change from flat for an 18 degree lift. */
#define MILLI_PER_UNIT 1000
#endif

/** @brief One stretch of the TER-U12 roll trace. */
typedef struct
{
    float roll_deg;   /**< Roll for the whole stretch. */
    uint32_t samples; /**< Length, samples. */
} trace_part_t;

/** @brief One stretch of a TER-U19 collision trace. */
typedef struct
{
    float push_g;              /**< Horizontal push for the whole stretch. */
    terrain_hump_state_t hump; /**< Hump state for the stretch. */
    bool b_speed_known;        /**< Motion's speed is fresh. */
    int32_t speed_mm_s;        /**< Ground speed. */
    uint32_t samples;          /**< Length, samples. */
} crash_part_t;

/** @brief One stretch of a TER-U22 turn-rate trace. */
typedef struct
{
    float turn_dps;   /**< Turn rate for the whole stretch. */
    uint32_t samples; /**< Length, samples. */
} turn_part_t;

/** @brief One stretch of a TER-U24 wheel-speed trace. */
typedef struct
{
    float wheel_mm_s; /**< Wheel speed for the whole stretch. */
    uint32_t samples; /**< Length, samples. */
} speed_part_t;

/** @brief A TER-U15 trapezoid ramp under one wheel, mm. */
typedef struct
{
    float ramp_mm;   /**< Length of each ramp. */
    float top_mm;    /**< Length of the flat top. */
    float height_mm; /**< Height of the top. */
} hump_shape_t;

/* Private function declarations. */
static bool hump_trace_ok (void);
static bool hump_heights_ok (void);
static int32_t drive_hump (hump_shape_t const * p_shape);
static float hump_ground_mm (hump_shape_t const * p_shape, float x_mm);
static float root (float square);
static bool queue_drops_oldest (void);
static bool crash_ignored_ok (void);
static bool crash_confirmed_ok (void);
static bool crash_at_once_ok (void);
static uint32_t run_crash (crash_part_t const * p_parts, uint32_t n_parts,
                           uint32_t * p_value);
static bool turn_rate_ok (void);
static bool rate_near (float dps, int32_t want_x10);
static bool turn_trace_ok (void);
static bool turn_keeps_hump_ok (void);
static bool speed_trace_ok (void);
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
               "6, 11 and 20 mm one-wheel ramps read their height");

    /* TER-U19: collision machine on made-up data (no IMU needed). */
    TEST_CHECK("TER-U19", crash_ignored_ok(),
               "soft push, jolt with no stop, and pushes on a ramp ignored");
    TEST_CHECK("TER-U19", crash_confirmed_ok(),
               "push then speed halved gives one COLLISION, biggest push");
    TEST_CHECK("TER-U19", crash_at_once_ok(),
               "sure push or unknown speed at once; one event per cooldown");

    /* TER-U22: turn rate and turning machine on made-up data (no IMU or
     * motors needed). */
    TEST_CHECK("TER-U22", turn_rate_ok(),
               "wheel speeds give the turn rate, left positive");
    TEST_CHECK("TER-U22", turn_trace_ok(),
               "turn trace gives TURNING, STRAIGHT, TURNING, STRAIGHT only");
    TEST_CHECK("TER-U22", turn_keeps_hump_ok(),
               "turning and hump fields of the detector word kept apart");

    /* TER-U24: speed machine on made-up wheel speeds (no IMU or motors
     * needed). */
    TEST_CHECK("TER-U24", speed_trace_ok(),
               "speed trace gives ACCELERATING, CRUISING, DECELERATING, "
               "STATIONARY only");

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
 * TER-U12: a 4-sample spike (ignored), then one ramp: climb and stay
 * (ASCENDING, then the top), come part way down (DESCENDING), flat
 * (LEVEL). Only those three may come out, in that order, with the climb's
 * lean as ASCENDING's value.
 */
static bool
hump_trace_ok (void)
{
    static trace_part_t const trace[] = {
        {TRACE_FLAT_DEG, 20u}, {TRACE_UP_DEG, TERRAIN_HUMP_ENTER_SAMPLES - 1u},
        {TRACE_FLAT_DEG, 10u}, {TRACE_UP_DEG, 20u},
        {TRACE_DOWN_DEG, 10u}, {TRACE_FLAT_DEG, 20u}};
    static terrain_event_type_t const expected[] = {
        TERRAIN_EVENT_ASCENDING, TERRAIN_EVENT_DESCENDING, TERRAIN_EVENT_LEVEL};
    terrain_hump_limits_t const limits = {
        TERRAIN_HUMP_UP_DEG,      TERRAIN_HUMP_DROP_DEG,
        TERRAIN_HUMP_LEVEL_DEG,   TERRAIN_HUMP_ENTER_SAMPLES,
        TERRAIN_HUMP_TOP_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};

    terrain_event_queue_t queue;
    terrain_event_queue_reset(&queue);
    uint32_t word = 0u;
    uint32_t tick = 0u;
    for (uint32_t part = 0u; part < (sizeof(trace) / sizeof(trace[0])); part++)
    {
        float const lean_deg = (trace[part].roll_deg < 0.0f)
                                   ? -trace[part].roll_deg
                                   : trace[part].roll_deg;
        for (uint32_t i = 0u; i < trace[part].samples; i++)
        {
            terrain_event_type_t const type =
                terrain_detect_hump_step(&word, trace[part].roll_deg, &limits);
            uint32_t const value = terrain_process_encode_signed(
                lean_deg, DDEG_PER_DEG, TERRAIN_EVENT_VALUE_BITS);
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
 * TER-U19: no COLLISION for a push under the limit, for a push over it
 * where the car keeps going (a jolt), or for any push on a ramp.
 */
static bool
crash_ignored_ok (void)
{
    static crash_part_t const soft[] = {
        {CRASH_SOFT_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_STOP_MM_S, CRASH_LONG}};
    static crash_part_t const jolt[] = {
        {CRASH_HIT_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, CRASH_LONG},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_STOP_MM_S, CRASH_LONG}};
    static crash_part_t const ramp[] = {
        {CRASH_SURE_G, TERRAIN_HUMP_ASCENDING, true, CRASH_CRUISE_MM_S, 1u},
        {CRASH_HIT_G, TERRAIN_HUMP_DESCENDING, false, 0, 1u},
        {CRASH_HIT_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_ASCENDING, true, CRASH_STOP_MM_S, CRASH_LONG}};
    uint32_t value = 0u;

    return (run_crash(soft, sizeof(soft) / sizeof(soft[0]), &value) == 0u) &&
           (run_crash(jolt, sizeof(jolt) / sizeof(jolt[0]), &value) == 0u) &&
           (run_crash(ramp, sizeof(ramp) / sizeof(ramp[0]), &value) == 0u);
}

/*
 * TER-U19: a push over the limit, a bigger one, then the speed halves a
 * few samples later: one COLLISION carrying the bigger push.
 */
static bool
crash_confirmed_ok (void)
{
    static crash_part_t const hit[] = {
        {CRASH_HIT_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {CRASH_BIG_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, CRASH_WAIT},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_STOP_MM_S, CRASH_LONG}};
    uint32_t value = 0u;

    return (run_crash(hit, sizeof(hit) / sizeof(hit[0]), &value) == 1u) &&
           (value == CRASH_BIG_DG);
}

/*
 * TER-U19: a sure push counts with no stop; so does a push over the limit
 * while motion's speed is unknown. Two sure pushes inside the cooldown
 * give one event; one after it gives another.
 */
static bool
crash_at_once_ok (void)
{
    static crash_part_t const sure[] = {
        {CRASH_SURE_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, CRASH_LONG}};
    static crash_part_t const unknown[] = {
        {CRASH_HIT_G, TERRAIN_HUMP_LEVEL, false, CRASH_CRUISE_MM_S, 1u}};
    static crash_part_t const twice[] = {
        {CRASH_SURE_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, CRASH_LONG},
        {CRASH_SURE_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u},
        {0.0f, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, CRASH_GAP},
        {CRASH_SURE_G, TERRAIN_HUMP_LEVEL, true, CRASH_CRUISE_MM_S, 1u}};
    uint32_t sure_value = 0u;
    uint32_t unknown_value = 0u;
    uint32_t value = 0u;

    return (run_crash(sure, sizeof(sure) / sizeof(sure[0]), &sure_value) ==
            1u) &&
           (sure_value == CRASH_SURE_DG) &&
           (run_crash(unknown, sizeof(unknown) / sizeof(unknown[0]),
                      &unknown_value) == 1u) &&
           (unknown_value == CRASH_HIT_DG) &&
           (run_crash(twice, sizeof(twice) / sizeof(twice[0]), &value) == 2u);
}

/* Feed a TER-U19 trace through the collision machine from READY; count
 * the COLLISION events and keep the last one's value. */
static uint32_t
run_crash (crash_part_t const * p_parts, uint32_t n_parts, uint32_t * p_value)
{
    terrain_collision_limits_t const limits = {
        (float)TERRAIN_COLLISION_LIMIT_DG / DG_PER_G,
        (float)TERRAIN_COLLISION_SURE_DG / DG_PER_G,
        (float)TERRAIN_COLLISION_MIN_SPEED_MM_S,
        TERRAIN_COLLISION_CHECK_SAMPLES, TERRAIN_COLLISION_COOLDOWN_SAMPLES};
    uint32_t word = 0u;
    uint32_t events = 0u;

    for (uint32_t part = 0u; part < n_parts; part++)
    {
        terrain_collision_input_t const in = {
            p_parts[part].push_g, p_parts[part].hump,
            p_parts[part].b_speed_known, p_parts[part].speed_mm_s};
        for (uint32_t i = 0u; i < p_parts[part].samples; i++)
        {
            if (terrain_detect_collision_step(&word, &in, &limits, p_value) ==
                TERRAIN_EVENT_COLLISION)
            {
                events++;
            }
        }
    }
    return events;
}

/*
 * TER-U22: a gentle left curve, a turn in place either way, and no wheel
 * speed difference; a track width of 0 gives 0, not a division by zero.
 */
static bool
turn_rate_ok (void)
{
    float const track_mm = TERRAIN_TRACK_WIDTH_MM;

    return rate_near(terrain_process_turn_rate_dps(
                         TURN_CURVE_LEFT_MM_S, TURN_CURVE_RIGHT_MM_S, track_mm),
                     TURN_CURVE_DPS_X10) &&
           rate_near(terrain_process_turn_rate_dps(-TURN_SPIN_MM_S,
                                                   TURN_SPIN_MM_S, track_mm),
                     TURN_SPIN_DPS_X10) &&
           rate_near(terrain_process_turn_rate_dps(TURN_SPIN_MM_S,
                                                   -TURN_SPIN_MM_S, track_mm),
                     -TURN_SPIN_DPS_X10) &&
           rate_near(terrain_process_turn_rate_dps(TURN_CURVE_RIGHT_MM_S,
                                                   TURN_CURVE_RIGHT_MM_S,
                                                   track_mm),
                     0) &&
           rate_near(terrain_process_turn_rate_dps(TURN_CURVE_LEFT_MM_S,
                                                   TURN_CURVE_RIGHT_MM_S, 0.0f),
                     0);
}

/* A rate within TURN_TOL_X10 of want_x10 (tenths of a deg/s). */
static bool
rate_near (float dps, int32_t want_x10)
{
    float const diff_x10 = (dps * DDEG_PER_DEG) - (float)want_x10;
    return (diff_x10 <= (float)TURN_TOL_X10) &&
           (diff_x10 >= -(float)TURN_TOL_X10);
}

/*
 * TER-U22: still, a turn in place one sample too short, and a rate below
 * the on limit give nothing. A turn in place gives TURNING; a slower
 * stretch between the limits keeps it; stopping gives STRAIGHT. A slow
 * right curve above the on limit gives TURNING (either way round counts),
 * creeping below the off limit STRAIGHT. Nothing else, in that order.
 */
static bool
turn_trace_ok (void)
{
    static turn_part_t const trace[] = {
        {TURN_STILL_DPS, 20u},
        {TURN_FAST_DPS, TERRAIN_TURN_ON_SAMPLES - 1u},
        {TURN_STILL_DPS, 5u},
        {TURN_MID_DPS, 40u},
        {TURN_FAST_DPS, 20u},
        {TURN_MID_DPS, 40u},
        {TURN_STILL_DPS, TERRAIN_TURN_OFF_SAMPLES},
        {TURN_SLOW_DPS, TERRAIN_TURN_ON_SAMPLES + 5u},
        {TURN_CREEP_DPS, TERRAIN_TURN_OFF_SAMPLES}};
    static terrain_event_type_t const expected[] = {
        TERRAIN_EVENT_TURNING, TERRAIN_EVENT_STRAIGHT, TERRAIN_EVENT_TURNING,
        TERRAIN_EVENT_STRAIGHT};
    terrain_turn_limits_t const limits = {
        (float)TERRAIN_TURN_ON_DPS, (float)TERRAIN_TURN_OFF_DPS,
        TERRAIN_TURN_ON_SAMPLES, TERRAIN_TURN_OFF_SAMPLES};

    terrain_event_type_t got[TURN_EVENTS_MAX];
    uint32_t n_got = 0u;
    uint32_t word = 0u;
    for (uint32_t part = 0u; part < (sizeof(trace) / sizeof(trace[0])); part++)
    {
        for (uint32_t i = 0u; i < trace[part].samples; i++)
        {
            terrain_event_type_t const type =
                terrain_detect_turn_step(&word, trace[part].turn_dps, &limits);
            if ((type != TERRAIN_EVENT_NONE) && (n_got < TURN_EVENTS_MAX))
            {
                got[n_got] = type;
                n_got++;
            }
        }
    }

    uint32_t const n = sizeof(expected) / sizeof(expected[0]);
    bool b_ok = (n_got == n) &&
                (terrain_detect_turn_state(word) == TERRAIN_TURN_STRAIGHT);
    for (uint32_t i = 0u; b_ok && (i < n); i++)
    {
        b_ok = (got[i] == expected[i]);
    }
    return b_ok;
}

/*
 * TER-U22: the turn step leaves the hump fields alone, and restarting the
 * humps (as a recalibration does) keeps the turning state.
 */
static bool
turn_keeps_hump_ok (void)
{
    terrain_turn_limits_t const limits = {
        (float)TERRAIN_TURN_ON_DPS, (float)TERRAIN_TURN_OFF_DPS,
        TERRAIN_TURN_ON_SAMPLES, TERRAIN_TURN_OFF_SAMPLES};
    uint32_t word = TURN_HUMP_BITS;
    for (uint32_t i = 0u; i < TERRAIN_TURN_ON_SAMPLES; i++)
    {
        (void)terrain_detect_turn_step(&word, TURN_FAST_DPS, &limits);
    }
    bool const b_hump_kept =
        ((word & TURN_HUMP_MASK) == TURN_HUMP_BITS) &&
        (terrain_detect_turn_state(word) == TERRAIN_TURN_TURNING);

    word = terrain_detect_hump_clear(word);
    return b_hump_kept && ((word & TURN_HUMP_MASK) == 0u) &&
           (terrain_detect_hump_state(word) == TERRAIN_HUMP_LEVEL) &&
           (terrain_detect_turn_state(word) == TERRAIN_TURN_TURNING);
}

/*
 * TER-U24: stopped, then a one-check blip: nothing. Speeding up 20 mm/s
 * a check gives ACCELERATING after two checks; steady gives CRUISING;
 * one noisy check up, then back down, gives nothing; slowing down gives
 * DECELERATING; stopped gives STATIONARY after three checks. Nothing
 * else, in that order. Every stretch is whole windows (10 samples), so
 * each check sees one stretch.
 */
static bool
speed_trace_ok (void)
{
    static speed_part_t const trace[] = {{SPEED_STOP_MM_S, 30u},
                                         {SPEED_BLIP_MM_S, 10u},
                                         {SPEED_STOP_MM_S, 30u},
                                         {40.0f, 10u},
                                         {60.0f, 10u},
                                         {80.0f, 10u},
                                         {100.0f, 10u},
                                         {SPEED_CRUISE_MM_S, 50u},
                                         {SPEED_NOISE_MM_S, 10u},
                                         {SPEED_CRUISE_MM_S, 40u},
                                         {100.0f, 10u},
                                         {80.0f, 10u},
                                         {60.0f, 10u},
                                         {40.0f, 10u},
                                         {SPEED_STOP_MM_S, 40u}};
    static terrain_event_type_t const expected[] = {
        TERRAIN_EVENT_ACCELERATING, TERRAIN_EVENT_CRUISING,
        TERRAIN_EVENT_DECELERATING, TERRAIN_EVENT_STATIONARY};
    terrain_speed_limits_t const limits = {
        (float)TERRAIN_SPEED_STOP_MM_S, (float)TERRAIN_SPEED_CHANGE_MM_S,
        TERRAIN_SPEED_WINDOW_SAMPLES, TERRAIN_SPEED_CONFIRM_CHECKS,
        TERRAIN_SPEED_STOP_CHECKS};

    terrain_event_type_t got[SPEED_EVENTS_MAX];
    uint32_t n_got = 0u;
    uint32_t word = 0u;
    for (uint32_t part = 0u; part < (sizeof(trace) / sizeof(trace[0])); part++)
    {
        for (uint32_t i = 0u; i < trace[part].samples; i++)
        {
            terrain_event_type_t const type = terrain_detect_speed_step(
                &word, trace[part].wheel_mm_s, &limits);
            if ((type != TERRAIN_EVENT_NONE) && (n_got < SPEED_EVENTS_MAX))
            {
                got[n_got] = type;
                n_got++;
            }
        }
    }

    uint32_t const n = sizeof(expected) / sizeof(expected[0]);
    bool b_ok = (n_got == n) &&
                (terrain_detect_speed_state(word) == TERRAIN_SPEED_STATIONARY);
    for (uint32_t i = 0u; b_ok && (i < n); i++)
    {
        b_ok = (got[i] == expected[i]);
    }
    return b_ok;
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
 * TER-U15: one wheel drives over a 6, 11 and 20 mm ramp (each ramp side
 * three times as long as the ramp is high, 20 mm flat top). Each must
 * read its height within HUMP_TOL_MM.
 */
static bool
hump_heights_ok (void)
{
    static hump_shape_t const ramps[] = {
        {18.0f, 20.0f, 6.0f}, {33.0f, 20.0f, 11.0f}, {60.0f, 20.0f, 20.0f}};

    bool b_ok = true;
    for (uint32_t i = 0u; i < (sizeof(ramps) / sizeof(ramps[0])); i++)
    {
        int32_t const want_mm = (int32_t)ramps[i].height_mm;
        int32_t const got_mm = drive_hump(&ramps[i]);
        b_ok = b_ok && (got_mm >= (want_mm - HUMP_TOL_MM)) &&
               (got_mm <= (want_mm + HUMP_TOL_MM));
    }
    return b_ok;
}

/*
 * Drive one wheel over a ramp while the other stays on the floor: the
 * axle (the track width long) tilts so that the lifted wheel is h mm up,
 * a roll of asin(h / track), worked out through the module's own roll
 * maths. As in the monitor, the hump machine gets the roll with the tilt
 * smoothing and the height the unsmoothed roll (it smooths it itself).
 * Returns the LEVEL height, -1 if none.
 */
static int32_t
drive_hump (hump_shape_t const * p_shape)
{
    terrain_hump_limits_t const hump_limits = {
        TERRAIN_HUMP_UP_DEG,      TERRAIN_HUMP_DROP_DEG,
        TERRAIN_HUMP_LEVEL_DEG,   TERRAIN_HUMP_ENTER_SAMPLES,
        TERRAIN_HUMP_TOP_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};
    terrain_height_limits_t const height_limits = {
        TERRAIN_TRACK_WIDTH_MM, TERRAIN_HUMP_LEVEL_DEG,
        TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_HUMP_HEIGHT_TAU_S,
                                      TERRAIN_SAMPLE_PERIOD_S)};
    float const tilt_alpha = TERRAIN_PROCESS_LOWPASS_ALPHA(
        TERRAIN_TILT_TAU_S, TERRAIN_SAMPLE_PERIOD_S);
    float const track_mm = TERRAIN_TRACK_WIDTH_MM;
    float const end_mm = HUMP_LEAD_MM + (2.0f * p_shape->ramp_mm) +
                         p_shape->top_mm + HUMP_TAIL_MM;

    terrain_height_t height;
    terrain_height_reset(&height);
    uint32_t word = 0u;
    float tilt_deg = 0.0f;
    int32_t result = -1;
    for (float x_mm = 0.0f; x_mm < end_mm; x_mm += HUMP_STEP_MM)
    {
        /* Gravity across the tilted axle: lift over the rest. */
        float const lift_mm = hump_ground_mm(p_shape, x_mm);
        terrain_vec3_t const across = {
            lift_mm, 0.0f, root((track_mm * track_mm) - (lift_mm * lift_mm))};
        float const roll_deg = terrain_process_roll_deg(&across);
        tilt_deg += tilt_alpha * (roll_deg - tilt_deg);
        terrain_height_step(&height, terrain_detect_hump_state(word), roll_deg,
                            &height_limits);
        if ((terrain_detect_hump_step(&word, tilt_deg, &hump_limits) ==
             TERRAIN_EVENT_LEVEL) &&
            (result < 0))
        {
            result = (int32_t)terrain_height_mm(&height);
        }
    }
    return result;
}

/* Ground height under x for a TER-U15 ramp starting at HUMP_LEAD_MM. */
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
 * Square root by Newton's method, for TER-U15's axle geometry: the test
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
