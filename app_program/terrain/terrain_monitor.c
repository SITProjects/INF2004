/**
 * @file terrain_monitor.c
 * @brief IMU motion and terrain monitoring: coordinator, calibration and
 *        events.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_monitor.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "drv_imu.h"
#include "drv_imu_bus.h"
#include "terrain_config.h"
#include "terrain_detect.h"
#include "terrain_event.h"
#include "terrain_fault.h"
#include "terrain_height.h"
#include "terrain_hint.h"
#include "terrain_platform.h"
#include "terrain_process.h"
#include "terrain_summary.h"

/* Private types, constants, and macros. */

/*
 * Packed layouts: masks and shifts, bit 0 first, little-endian words.
 * Signed fields are two's complement, so an all-zero word reads as 0.
 *
 * Snapshot (terrain_data_t), five words:
 *   word 0: pitch 10 | roll 10 | turn rate 9 | calib state 2 |
 *           pitch held 1
 *   word 1: roll held 1 | hump state 2 | spare 1 | turning state 1 |
 *           speed state 2 | highest hump 8 | time 16 | spare 1
 *   word 2: sample count 16 | overrun 6 | errors 6 | sensor resets 4
 *   word 3: accel X 12 | accel Y 12 | spare 8
 *   word 4: accel Z 12 | spare 20
 * Later-phase fields are reserved and stay 0 (their resting value).
 */
#define W_TILT 0u
#define PITCH_SHIFT 0u
#define ROLL_SHIFT 10u
#define ANGLE_BITS 10u
#define TURN_RATE_SHIFT 20u
#define TURN_RATE_BITS 9u
#define CALIB_SHIFT 29u
#define CALIB_BITS 2u
#define PITCH_HELD_SHIFT 31u

#define W_STATE 1u
#define ROLL_HELD_SHIFT 0u
#define HUMP_SHIFT 1u
#define HUMP_BITS 2u
#define TURN_STATE_SHIFT 4u
#define SPEED_STATE_SHIFT 5u
#define SPEED_STATE_BITS 2u
#define HIGHEST_SHIFT 7u
#define HIGHEST_BITS 8u
#define TIME_SHIFT 15u
#define TIME_BITS 16u

#define W_COUNTS 2u
#define SAMPLES_SHIFT 0u
#define SAMPLES_BITS 16u
#define OVERRUN_SHIFT 16u
#define OVERRUN_BITS 6u
#define ERRORS_SHIFT 22u
#define ERRORS_BITS 6u
#define RESETS_SHIFT 28u
#define RESETS_BITS 4u

#define W_ACCEL_XY 3u
#define W_ACCEL_Z 4u
#define ACCEL_X_SHIFT 0u
#define ACCEL_Y_SHIFT 12u
#define ACCEL_Z_SHIFT 0u
#define ACCEL_BITS 12u

#define FLAG_BITS 1u

/* Calibration details word: attempts 7 | retries 7 | spread 8 | tilt 8 |
 * state 2. The state bits are filled by the getter from the snapshot. */
#define ATTEMPTS_SHIFT 0u
#define RETRIES_SHIFT 7u
#define ATTEMPT_BITS 7u
#define SPREAD_SHIFT 14u
#define SPREAD_BITS 8u
#define CALIB_TILT_SHIFT 22u
#define CALIB_TILT_BITS 8u
#define INFO_STATE_SHIFT 30u

/* Private task state word: calib state 2 | settle 6 | fail streak 4. */
#define OWN_CALIB_SHIFT 0u
#define OWN_SETTLE_SHIFT 2u
#define OWN_SETTLE_BITS 6u
#define OWN_FAIL_SHIFT 8u
#define OWN_FAIL_BITS 4u

/* Units of the packed fields. */
#define DDEG_PER_DEG 10.0f
#define DG_PER_G 10.0f
#define DPS_PER_STEP 1.0f
#define WHEELS 2.0f
#define MG_PER_STEP 4
#define STEPS_PER_MPS2 (1000.0f / (TERRAIN_GRAVITY_MPS2 * (float)MG_PER_STEP))
#define US_PER_TICK 10000u

_Static_assert(sizeof(terrain_data_t) == 20u, "snapshot must be 5 words");
_Static_assert(sizeof(terrain_calib_info_t) == 16u,
               "calibration details must be 1 word + 3 floats");
_Static_assert(TERRAIN_CALIB_VALID < (1 << CALIB_BITS),
               "calibration state must fit its field");
_Static_assert(TERRAIN_CALIB_SETTLE_SAMPLES < (1u << OWN_SETTLE_BITS),
               "settle period must fit its field");
_Static_assert(TERRAIN_FAULT_RETRY_UPDATES < (1u << OWN_FAIL_BITS),
               "fault limit must fit its field");
_Static_assert(TERRAIN_CALIB_SAMPLES <= UINT8_MAX,
               "calibration samples must fit the 8-bit count");
_Static_assert(TERRAIN_HUMP_DESCENDING < (1u << HUMP_BITS),
               "hump state must fit its field");
_Static_assert(TERRAIN_TURN_TURNING < (1u << FLAG_BITS),
               "turning state must fit its field");
_Static_assert(TERRAIN_SPEED_DECELERATING < (1u << SPEED_STATE_BITS),
               "speed state must fit its field");
_Static_assert((SPEED_STATE_SHIFT + SPEED_STATE_BITS) == HIGHEST_SHIFT,
               "speed state must end where the highest hump starts");
_Static_assert(TERRAIN_HEIGHT_MAX_MM == ((1u << HIGHEST_BITS) - 1u),
               "highest hump must fit its field");
_Static_assert(sizeof(terrain_event_queue_t) ==
                   ((TERRAIN_EVENT_QUEUE_LEN + 1u) * sizeof(uint32_t)),
               "event queue must be one word per event plus one");
_Static_assert(sizeof(terrain_summary_t) == 12u, "summary must be 3 words");

/* Static data. */

/* Mounting of the board on the car, from terrain_config.h. */
static terrain_axis_map_t const axis_map = {
    {TERRAIN_CAR_X_FROM, TERRAIN_CAR_Y_FROM, TERRAIN_CAR_Z_FROM},
    {TERRAIN_CAR_X_SIGN, TERRAIN_CAR_Y_SIGN, TERRAIN_CAR_Z_SIGN}};

static float const board_gain[TERRAIN_AXES] = {
    TERRAIN_GAIN_BOARD_X, TERRAIN_GAIN_BOARD_Y, TERRAIN_GAIN_BOARD_Z};

static terrain_vec3_t const level_ref = {
    TERRAIN_LEVEL_REF_X, TERRAIN_LEVEL_REF_Y, TERRAIN_LEVEL_REF_Z};

static terrain_calib_limits_t const calib_limits = {
    TERRAIN_CALIB_MAX_SPREAD_MPS2, TERRAIN_CALIB_MAX_GRAVITY_ERR_MPS2,
    TERRAIN_LEVEL_MAX_DEG};

static terrain_hump_limits_t const hump_limits = {
    TERRAIN_HUMP_UP_DEG,      TERRAIN_HUMP_DROP_DEG,
    TERRAIN_HUMP_LEVEL_DEG,   TERRAIN_HUMP_ENTER_SAMPLES,
    TERRAIN_HUMP_TOP_SAMPLES, TERRAIN_HUMP_LEVEL_SAMPLES};

static terrain_stopped_limits_t const stopped_limits = {
    TERRAIN_COLLISION_STOP_FROM_MM_S, TERRAIN_COLLISION_STOP_WINDOW_SAMPLES,
    TERRAIN_COLLISION_STOP_GRACE_SAMPLES};
static terrain_collision_limits_t const collision_limits = {
    (float)TERRAIN_COLLISION_LIMIT_DG / DG_PER_G,
    (float)TERRAIN_COLLISION_SURE_DG / DG_PER_G,
    (float)TERRAIN_COLLISION_MIN_SPEED_MM_S, TERRAIN_COLLISION_CHECK_SAMPLES,
    TERRAIN_COLLISION_COOLDOWN_SAMPLES};

static terrain_turn_limits_t const turn_limits = {
    (float)TERRAIN_TURN_ON_DPS, (float)TERRAIN_TURN_OFF_DPS,
    TERRAIN_TURN_ON_SAMPLES, TERRAIN_TURN_OFF_SAMPLES};

static terrain_speed_limits_t const speed_limits = {
    (float)TERRAIN_SPEED_STOP_MM_S, (float)TERRAIN_SPEED_CHANGE_MM_S,
    TERRAIN_SPEED_WINDOW_SAMPLES, TERRAIN_SPEED_CONFIRM_CHECKS,
    TERRAIN_SPEED_STOP_CHECKS};

static terrain_height_limits_t const height_limits = {
    TERRAIN_TRACK_WIDTH_MM, TERRAIN_HUMP_LEVEL_DEG,
    TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_HUMP_HEIGHT_TAU_S,
                                  TERRAIN_SAMPLE_PERIOD_S)};

/*
 * Shared with other tasks; access only under terrain_platform_lock().
 * Only the caller of terrain_monitor_update() writes latest and calib_info;
 * other tasks write only b_recal_request. events is changed by both: the
 * update adds events and terrain_monitor_poll_event() removes them. Only
 * the update writes summary.
 */
static terrain_data_t latest;
static terrain_calib_info_t calib_info;
static bool b_recal_request = false;
static terrain_event_queue_t events;
static terrain_summary_t summary;

static bool b_ready = false;

/* Owned by the caller of terrain_monitor_update(). */
static uint32_t own_state = 0u; /* OWN_* fields. */
static terrain_calib_acc_t calib_acc;
static terrain_vec3_t bias;
static terrain_tilt_t tilt;
static uint32_t detect_word = 0u;  /* terrain_detect.h layout. */
static int32_t hump_start_mm = 0;  /* Where the latest climb began. */
static uint32_t collide_word = 0u; /* terrain_detect.h layout. */
static terrain_stopped_t stopped;
static uint32_t speed_word = 0u; /* terrain_detect.h layout. */
static terrain_height_t height;

/* Worked out when building, so it lives in flash. */
static float const tilt_alpha =
    TERRAIN_PROCESS_LOWPASS_ALPHA(TERRAIN_TILT_TAU_S, TERRAIN_SAMPLE_PERIOD_S);

/* Private function declarations. */
static terrain_status_t setup_sensor (void);
static terrain_status_t read_samples (imu_accel_raw_t * p_buf,
                                      uint32_t * p_count, bool * p_overrun);
static bool take_recal_request (void);
static void calib_start (void);
static void calib_add (terrain_vec3_t const * p_sample);
static void handle_fault (void);
static void detect_sample (terrain_vec3_t const * p_accel,
                           terrain_hint_t const * p_hint, uint32_t now_10ms);
static void detect_hump (terrain_vec3_t const * p_accel,
                         terrain_hint_t const * p_hint, uint32_t now_10ms);
static void detect_collision (terrain_vec3_t const * p_accel,
                              terrain_hint_t const * p_hint, uint32_t now_10ms);
static void detect_turn (float turn_dps, uint32_t now_10ms);
static void detect_speed (float wheel_mm_s, uint32_t now_10ms);
static void queue_event (terrain_event_type_t type, uint32_t value,
                         uint32_t now_10ms);
static void show_calibrating (terrain_data_t * p_data);
static void clear_calib_info (terrain_calib_info_t * p_info);
static void publish (terrain_vec3_t const * p_accel, float turn_dps,
                     uint32_t new_samples, bool b_overrun, uint32_t now_10ms);
static void publish_calib (terrain_calib_out_t const * p_out, bool b_moved);
static void store_latest (terrain_data_t const * p_data);
static uint32_t own_get (uint32_t shift, uint32_t width);
static void own_put (uint32_t shift, uint32_t width, uint32_t value);
static terrain_calib_state_t own_calib_state (void);
static uint32_t data_get (terrain_data_t const * p_data, uint32_t word,
                          uint32_t shift, uint32_t width);
static void data_put (terrain_data_t * p_data, uint32_t word, uint32_t shift,
                      uint32_t width, uint32_t code);
static int16_t data_accel_mg (terrain_data_t const * p_data, uint32_t word,
                              uint32_t shift);

/* Public function definitions. */

terrain_status_t
terrain_monitor_init (void)
{
    b_ready = false;

    terrain_status_t status = setup_sensor();
    if (status != TERRAIN_OK)
    {
        terrain_fault_set();
        return status;
    }
    terrain_fault_clear();

    terrain_data_t const empty_data = {0};
    uint32_t saved = terrain_platform_lock();
    latest = empty_data;
    show_calibrating(&latest);
    clear_calib_info(&calib_info);
    b_recal_request = false;
    terrain_event_queue_reset(&events);
    terrain_summary_reset(&summary);
    terrain_platform_unlock(saved);

    own_put(OWN_FAIL_SHIFT, OWN_FAIL_BITS, 0u);
    terrain_height_reset(&height);
    /* calib_start() keeps the turning and speed states: not here. */
    detect_word = 0u;
    speed_word = 0u;
    calib_start();

    b_ready = true;
    return TERRAIN_OK;
}

terrain_status_t
terrain_monitor_update (void)
{
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    if (take_recal_request())
    {
        calib_start();
    }

    /* Static keeps ~200 bytes off the stack; one caller only. */
    static imu_accel_raw_t buf[IMU_ACCEL_FIFO_DEPTH];
    uint32_t count = 0u;
    bool b_overrun = false;

    terrain_status_t status = read_samples(buf, &count, &b_overrun);
    if (status != TERRAIN_OK)
    {
        handle_fault();
        return status;
    }
    own_put(OWN_FAIL_SHIFT, OWN_FAIL_BITS, 0u);
    terrain_fault_clear();

    if (count == 0u)
    {
        return TERRAIN_OK;
    }

    float scale = imu_accel_scale_mps2();
    terrain_vec3_t accel = {0.0f, 0.0f, 0.0f};

    /* One time for the whole batch: its samples are within 10 ms. */
    uint32_t const now_10ms =
        (uint32_t)(terrain_platform_time_us() / US_PER_TICK);

    /* Speed changes fake pitch and turns fake roll: hold them then. */
    terrain_hint_t hint;
    terrain_hint_read(&hint);
    bool const b_hold_pitch = hint.b_speed_known && hint.b_speed_changing;
    bool const b_hold_roll = hint.b_turn_known && hint.b_turning;

    /* From the wheels, so it needs no calibration; 0 without odometry. */
    float const turn_dps =
        hint.b_distance_known
            ? terrain_process_turn_rate_dps(hint.left_mm_s, hint.right_mm_s,
                                            TERRAIN_TRACK_WIDTH_MM)
            : 0.0f;

    /* Wheel speed, from the sizes so a turn in place counts as moving;
     * 0 (stopped) without odometry. */
    float const wheel_mm_s =
        hint.b_distance_known
            ? (fabsf((float)hint.left_mm_s) + fabsf((float)hint.right_mm_s)) /
                  WHEELS
            : 0.0f;

    for (uint32_t i = 0u; i < count; i++)
    {
        int16_t const board[TERRAIN_AXES] = {buf[i].x, buf[i].y, buf[i].z};
        terrain_vec3_t sample =
            terrain_process_map_axes(&axis_map, board_gain, board, scale);

        if (own_calib_state() == TERRAIN_CALIB_CALIBRATING)
        {
            uint32_t const settle_left =
                own_get(OWN_SETTLE_SHIFT, OWN_SETTLE_BITS);
            if (settle_left > 0u)
            {
                own_put(OWN_SETTLE_SHIFT, OWN_SETTLE_BITS, settle_left - 1u);
            }
            else
            {
                calib_add(&sample);
            }
        }

        /* Not "else": the sample that completes calibration counts too. */
        if (own_calib_state() == TERRAIN_CALIB_VALID)
        {
            accel = terrain_process_correct(&sample, &bias);
            terrain_process_tilt_update(&tilt, &accel, tilt_alpha, b_hold_pitch,
                                        b_hold_roll);
            detect_sample(&accel, &hint, now_10ms);
        }

        /* Every calibration state: the wheels need no level reference. */
        detect_turn(turn_dps, now_10ms);
        detect_speed(wheel_mm_s, now_10ms);
    }

    publish(&accel, turn_dps, count, b_overrun, now_10ms);
    return TERRAIN_OK;
}

terrain_status_t
terrain_monitor_get_latest (terrain_data_t * p_out)
{
    if (p_out == NULL)
    {
        return TERRAIN_ERR_PARAM;
    }
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    uint32_t saved = terrain_platform_lock();
    *p_out = latest;
    bool const b_pending = b_recal_request;
    terrain_platform_unlock(saved);

    /* The old results stand until the next update resets them: hide them. */
    if (b_pending)
    {
        show_calibrating(p_out);
    }

    if (terrain_monitor_data_calib_state(p_out) != TERRAIN_CALIB_VALID)
    {
        return TERRAIN_ERR_NOT_CALIBRATED;
    }
    return TERRAIN_OK;
}

bool
terrain_monitor_on_hump (void)
{
    terrain_data_t data = {{0u}};
    (void)terrain_monitor_get_latest(&data);
    return terrain_monitor_data_hump_state(&data) != TERRAIN_HUMP_LEVEL;
}

terrain_status_t
terrain_monitor_get_calibration (terrain_calib_info_t * p_out)
{
    if (p_out == NULL)
    {
        return TERRAIN_ERR_PARAM;
    }
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    uint32_t saved = terrain_platform_lock();
    *p_out = calib_info;
    uint32_t const state = data_get(&latest, W_TILT, CALIB_SHIFT, CALIB_BITS);
    bool const b_pending = b_recal_request;
    terrain_platform_unlock(saved);

    /* The state is kept once, in the snapshot. */
    p_out->word = terrain_process_field_put(p_out->word, INFO_STATE_SHIFT,
                                            CALIB_BITS, state);
    if (b_pending)
    {
        clear_calib_info(p_out);
    }
    return TERRAIN_OK;
}

terrain_status_t
terrain_monitor_recalibrate (void)
{
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    /*
     * Only flag the request: terrain_monitor_update() does the work in its own
     * task, so the sensor and the shared copies have one user. Until then the
     * getters hide the old results, so no caller sees the old VALID state.
     */
    uint32_t saved = terrain_platform_lock();
    b_recal_request = true;
    terrain_platform_unlock(saved);
    return TERRAIN_OK;
}

terrain_status_t
terrain_monitor_poll_event (terrain_event_t * p_out)
{
    if (p_out == NULL)
    {
        return TERRAIN_ERR_PARAM;
    }
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    uint32_t saved = terrain_platform_lock();
    *p_out = terrain_event_queue_pop(&events);
    terrain_platform_unlock(saved);
    return TERRAIN_OK;
}

uint8_t
terrain_monitor_events_dropped (void)
{
    if (!b_ready)
    {
        return 0u;
    }

    uint32_t saved = terrain_platform_lock();
    uint32_t const dropped = terrain_event_queue_dropped(&events);
    terrain_platform_unlock(saved);
    return (uint8_t)dropped;
}

terrain_status_t
terrain_monitor_get_summary (terrain_summary_t * p_out)
{
    if (p_out == NULL)
    {
        return TERRAIN_ERR_PARAM;
    }
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    uint32_t saved = terrain_platform_lock();
    *p_out = summary;
    terrain_platform_unlock(saved);
    return TERRAIN_OK;
}

uint8_t
terrain_monitor_summary_hump_count (terrain_summary_t const * p_summary)
{
    return terrain_summary_hump_count(p_summary);
}

uint8_t
terrain_monitor_summary_highest_mm (terrain_summary_t const * p_summary)
{
    return terrain_summary_highest_mm(p_summary);
}

uint8_t
terrain_monitor_summary_hump_mm (terrain_summary_t const * p_summary,
                                 uint32_t index)
{
    return terrain_summary_hump_mm(p_summary, index);
}

uint8_t
terrain_monitor_summary_collisions (terrain_summary_t const * p_summary)
{
    return terrain_summary_collisions(p_summary);
}

uint8_t
terrain_monitor_summary_turns (terrain_summary_t const * p_summary)
{
    return terrain_summary_turns(p_summary);
}

terrain_status_t
terrain_monitor_summary_pack (terrain_summary_t const * p_summary,
                              uint8_t * p_bytes)
{
    return terrain_summary_pack(p_summary, p_bytes) ? TERRAIN_OK
                                                    : TERRAIN_ERR_PARAM;
}

terrain_event_type_t
terrain_monitor_event_type (terrain_event_t const * p_event)
{
    if (p_event == NULL)
    {
        return TERRAIN_EVENT_NONE;
    }
    return terrain_event_type(*p_event);
}

int16_t
terrain_monitor_event_value (terrain_event_t const * p_event)
{
    if (p_event == NULL)
    {
        return 0;
    }
    return (int16_t)terrain_event_value(*p_event);
}

uint16_t
terrain_monitor_event_time_10ms (terrain_event_t const * p_event)
{
    if (p_event == NULL)
    {
        return 0u;
    }
    return terrain_event_time_10ms(*p_event);
}

terrain_calib_state_t
terrain_monitor_data_calib_state (terrain_data_t const * p_data)
{
    return (terrain_calib_state_t)data_get(p_data, W_TILT, CALIB_SHIFT,
                                           CALIB_BITS);
}

terrain_hump_state_t
terrain_monitor_data_hump_state (terrain_data_t const * p_data)
{
    return (terrain_hump_state_t)data_get(p_data, W_STATE, HUMP_SHIFT,
                                          HUMP_BITS);
}

uint8_t
terrain_monitor_data_highest_hump_mm (terrain_data_t const * p_data)
{
    return (uint8_t)data_get(p_data, W_STATE, HIGHEST_SHIFT, HIGHEST_BITS);
}

int16_t
terrain_monitor_data_turn_rate_dps (terrain_data_t const * p_data)
{
    uint32_t const code =
        data_get(p_data, W_TILT, TURN_RATE_SHIFT, TURN_RATE_BITS);
    return (int16_t)terrain_process_decode_signed(code, TURN_RATE_BITS);
}

terrain_turn_state_t
terrain_monitor_data_turn_state (terrain_data_t const * p_data)
{
    return (terrain_turn_state_t)data_get(p_data, W_STATE, TURN_STATE_SHIFT,
                                          FLAG_BITS);
}

terrain_speed_state_t
terrain_monitor_data_speed_state (terrain_data_t const * p_data)
{
    return (terrain_speed_state_t)data_get(p_data, W_STATE, SPEED_STATE_SHIFT,
                                           SPEED_STATE_BITS);
}

int16_t
terrain_monitor_data_pitch_ddeg (terrain_data_t const * p_data)
{
    uint32_t const code = data_get(p_data, W_TILT, PITCH_SHIFT, ANGLE_BITS);
    return (int16_t)terrain_process_decode_signed(code, ANGLE_BITS);
}

int16_t
terrain_monitor_data_roll_ddeg (terrain_data_t const * p_data)
{
    uint32_t const code = data_get(p_data, W_TILT, ROLL_SHIFT, ANGLE_BITS);
    return (int16_t)terrain_process_decode_signed(code, ANGLE_BITS);
}

bool
terrain_monitor_data_is_pitch_held (terrain_data_t const * p_data)
{
    return data_get(p_data, W_TILT, PITCH_HELD_SHIFT, FLAG_BITS) != 0u;
}

bool
terrain_monitor_data_is_roll_held (terrain_data_t const * p_data)
{
    return data_get(p_data, W_STATE, ROLL_HELD_SHIFT, FLAG_BITS) != 0u;
}

int16_t
terrain_monitor_data_accel_x_mg (terrain_data_t const * p_data)
{
    return data_accel_mg(p_data, W_ACCEL_XY, ACCEL_X_SHIFT);
}

int16_t
terrain_monitor_data_accel_y_mg (terrain_data_t const * p_data)
{
    return data_accel_mg(p_data, W_ACCEL_XY, ACCEL_Y_SHIFT);
}

int16_t
terrain_monitor_data_accel_z_mg (terrain_data_t const * p_data)
{
    return data_accel_mg(p_data, W_ACCEL_Z, ACCEL_Z_SHIFT);
}

uint16_t
terrain_monitor_data_time_10ms (terrain_data_t const * p_data)
{
    return (uint16_t)data_get(p_data, W_STATE, TIME_SHIFT, TIME_BITS);
}

uint16_t
terrain_monitor_data_sample_count (terrain_data_t const * p_data)
{
    return (uint16_t)data_get(p_data, W_COUNTS, SAMPLES_SHIFT, SAMPLES_BITS);
}

uint8_t
terrain_monitor_data_overrun_count (terrain_data_t const * p_data)
{
    return (uint8_t)data_get(p_data, W_COUNTS, OVERRUN_SHIFT, OVERRUN_BITS);
}

uint8_t
terrain_monitor_data_error_count (terrain_data_t const * p_data)
{
    return (uint8_t)data_get(p_data, W_COUNTS, ERRORS_SHIFT, ERRORS_BITS);
}

uint8_t
terrain_monitor_data_reset_count (terrain_data_t const * p_data)
{
    return (uint8_t)data_get(p_data, W_COUNTS, RESETS_SHIFT, RESETS_BITS);
}

terrain_calib_state_t
terrain_monitor_calib_state (terrain_calib_info_t const * p_info)
{
    if (p_info == NULL)
    {
        return TERRAIN_CALIB_CALIBRATING;
    }
    return (terrain_calib_state_t)terrain_process_field_get(
        p_info->word, INFO_STATE_SHIFT, CALIB_BITS);
}

uint8_t
terrain_monitor_calib_attempts (terrain_calib_info_t const * p_info)
{
    if (p_info == NULL)
    {
        return 0u;
    }
    return (uint8_t)terrain_process_field_get(p_info->word, ATTEMPTS_SHIFT,
                                              ATTEMPT_BITS);
}

uint8_t
terrain_monitor_calib_retries (terrain_calib_info_t const * p_info)
{
    if (p_info == NULL)
    {
        return 0u;
    }
    return (uint8_t)terrain_process_field_get(p_info->word, RETRIES_SHIFT,
                                              ATTEMPT_BITS);
}

uint16_t
terrain_monitor_calib_spread_mg (terrain_calib_info_t const * p_info)
{
    if (p_info == NULL)
    {
        return 0u;
    }
    uint32_t const code =
        terrain_process_field_get(p_info->word, SPREAD_SHIFT, SPREAD_BITS);
    return (uint16_t)(code * (uint32_t)MG_PER_STEP);
}

uint16_t
terrain_monitor_calib_tilt_ddeg (terrain_calib_info_t const * p_info)
{
    if (p_info == NULL)
    {
        return 0u;
    }
    return (uint16_t)terrain_process_field_get(p_info->word, CALIB_TILT_SHIFT,
                                               CALIB_TILT_BITS);
}

char const *
terrain_monitor_status_text (terrain_status_t status)
{
    switch (status)
    {
        case TERRAIN_OK:
            return "OK";
        case TERRAIN_ERR_NOT_INIT:
            return "terrain_monitor_init() has not succeeded yet";
        case TERRAIN_ERR_BUS_SETUP:
            return "cannot open the kernel I2C device: check "
                   "DEVCNF_USE_IIC in config/config_device.h";
        case TERRAIN_ERR_NOT_FOUND:
            return "accelerometer not found: check the cable, Grove port "
                   "and 3.3 V power";
        case TERRAIN_ERR_SENSOR:
            return "accelerometer setup or read failed";
        case TERRAIN_ERR_PARAM:
            return "invalid argument";
        case TERRAIN_ERR_NOT_CALIBRATED:
            return "not calibrated yet";
        default:
            return "unknown status";
    }
}

char const *
terrain_monitor_calib_text (terrain_calib_state_t state)
{
    switch (state)
    {
        case TERRAIN_CALIB_CALIBRATING:
            return "CALIBRATING";
        case TERRAIN_CALIB_NOT_LEVEL:
            return "NOT_LEVEL";
        case TERRAIN_CALIB_VALID:
            return "VALID";
        default:
            return "UNKNOWN";
    }
}

char const *
terrain_monitor_hump_text (terrain_hump_state_t state)
{
    switch (state)
    {
        case TERRAIN_HUMP_LEVEL:
            return "LEVEL";
        case TERRAIN_HUMP_ASCENDING:
            return "ASCENDING";
        case TERRAIN_HUMP_TOP:
            return "TOP";
        case TERRAIN_HUMP_DESCENDING:
            return "DESCENDING";
        default:
            return "UNKNOWN";
    }
}

char const *
terrain_monitor_turn_text (terrain_turn_state_t state)
{
    switch (state)
    {
        case TERRAIN_TURN_STRAIGHT:
            return "STRAIGHT";
        case TERRAIN_TURN_TURNING:
            return "TURNING";
        default:
            return "UNKNOWN";
    }
}

char const *
terrain_monitor_speed_text (terrain_speed_state_t state)
{
    switch (state)
    {
        case TERRAIN_SPEED_STATIONARY:
            return "STATIONARY";
        case TERRAIN_SPEED_ACCELERATING:
            return "ACCELERATING";
        case TERRAIN_SPEED_CRUISING:
            return "CRUISING";
        case TERRAIN_SPEED_DECELERATING:
            return "DECELERATING";
        default:
            return "UNKNOWN";
    }
}

char const *
terrain_monitor_event_text (terrain_event_type_t type)
{
    switch (type)
    {
        case TERRAIN_EVENT_NONE:
            return "NONE";
        case TERRAIN_EVENT_ASCENDING:
            return "ASCENDING";
        case TERRAIN_EVENT_DESCENDING:
            return "DESCENDING";
        case TERRAIN_EVENT_LEVEL:
            return "LEVEL";
        case TERRAIN_EVENT_COLLISION:
            return "COLLISION";
        case TERRAIN_EVENT_STATIONARY:
            return "STATIONARY";
        case TERRAIN_EVENT_ACCELERATING:
            return "ACCELERATING";
        case TERRAIN_EVENT_CRUISING:
            return "CRUISING";
        case TERRAIN_EVENT_DECELERATING:
            return "DECELERATING";
        case TERRAIN_EVENT_TURNING:
            return "TURNING";
        case TERRAIN_EVENT_STRAIGHT:
            return "STRAIGHT";
        default:
            return "UNKNOWN";
    }
}

/* Private function definitions. */

/* Open the I2C bus and set up the accelerometer from terrain_config.h. */
static terrain_status_t
setup_sensor (void)
{
    if (!imu_bus_init())
    {
        return TERRAIN_ERR_BUS_SETUP;
    }

    imu_status_t status =
        imu_accel_init(TERRAIN_ACCEL_ADDR, TERRAIN_ACCEL_ODR,
                       TERRAIN_ACCEL_RANGE, (TERRAIN_ACCEL_USE_FIFO != 0));
    if (status == IMU_ERR_NOT_FOUND)
    {
        return TERRAIN_ERR_NOT_FOUND;
    }
    if (status != IMU_OK)
    {
        return TERRAIN_ERR_SENSOR;
    }
    return TERRAIN_OK;
}

static terrain_status_t
read_samples (imu_accel_raw_t * p_buf, uint32_t * p_count, bool * p_overrun)
{
    imu_status_t status = IMU_OK;

#if TERRAIN_ACCEL_USE_FIFO
    status =
        imu_accel_read_fifo(p_buf, IMU_ACCEL_FIFO_DEPTH, p_count, p_overrun);
#else
    *p_overrun = false;
    status = imu_accel_read(&p_buf[0]);
    *p_count = (status == IMU_OK) ? 1u : 0u;
#endif

    return (status == IMU_OK) ? TERRAIN_OK : TERRAIN_ERR_SENSOR;
}

/*
 * Return and clear a request made by terrain_monitor_recalibrate(). The
 * shared copies are reset in the same locked section, so a getter never
 * finds the request gone while the old results are still there.
 */
static bool
take_recal_request (void)
{
    uint32_t saved = terrain_platform_lock();
    bool b_request = b_recal_request;
    if (b_request)
    {
        b_recal_request = false;
        show_calibrating(&latest);
        clear_calib_info(&calib_info);
    }
    terrain_platform_unlock(saved);
    return b_request;
}

/* Forget the current calibration and start a new run of attempts. */
static void
calib_start (void)
{
    terrain_vec3_t const zero = {0.0f, 0.0f, 0.0f};

    own_put(OWN_CALIB_SHIFT, CALIB_BITS, (uint32_t)TERRAIN_CALIB_CALIBRATING);
    own_put(OWN_SETTLE_SHIFT, OWN_SETTLE_BITS, TERRAIN_CALIB_SETTLE_SAMPLES);
    bias = zero;
    terrain_process_calib_reset(&calib_acc);
    terrain_process_tilt_reset(&tilt);

    /* A hump or collision check cut short just ends: no event, queued
     * events are kept. Turning and the speed state come from the wheels,
     * not this calibration, so they carry on. */
    detect_word = terrain_detect_hump_clear(detect_word);
    collide_word = 0u;
    stopped = (terrain_stopped_t){0};
    terrain_height_reset(&height);
}

/* Collect one sample; after a full attempt, judge it. */
static void
calib_add (terrain_vec3_t const * p_sample)
{
    terrain_process_calib_add(&calib_acc, p_sample);
    if (calib_acc.count < TERRAIN_CALIB_SAMPLES)
    {
        return;
    }

    terrain_calib_out_t out;
    terrain_calib_result_t result = terrain_process_calib_finish(
        &calib_acc, &level_ref, &calib_limits, &out);
    terrain_process_calib_reset(&calib_acc);

    bool b_moved = false;
    switch (result)
    {
        case TERRAIN_CALIB_RESULT_OK:
            bias = out.bias;
            own_put(OWN_CALIB_SHIFT, CALIB_BITS, (uint32_t)TERRAIN_CALIB_VALID);
            break;
        case TERRAIN_CALIB_RESULT_NOT_LEVEL:
            own_put(OWN_CALIB_SHIFT, CALIB_BITS,
                    (uint32_t)TERRAIN_CALIB_NOT_LEVEL);
            break;
        default:
            /* Moved: stay CALIBRATING and try again, without limit. */
            b_moved = true;
            break;
    }

    publish_calib(&out, b_moved);
}

/*
 * A read failed. Samples collected so far no longer form one continuous
 * still period, so a calibration attempt starts again. After repeated
 * failures the bus and sensor are set up again; the bias is kept.
 */
static void
handle_fault (void)
{
    if (own_calib_state() == TERRAIN_CALIB_CALIBRATING)
    {
        terrain_process_calib_reset(&calib_acc);
    }

    bool b_reset = false;
    uint32_t streak = own_get(OWN_FAIL_SHIFT, OWN_FAIL_BITS) + 1u;
    if (streak >= TERRAIN_FAULT_RETRY_UPDATES)
    {
        /* A real fault, not a glitch: raise it until a read succeeds. */
        terrain_fault_set();
        streak = 0u;
        b_reset = (setup_sensor() == TERRAIN_OK);
    }
    own_put(OWN_FAIL_SHIFT, OWN_FAIL_BITS, streak);

    /* Only this task writes latest, so it can read it without the lock. */
    terrain_data_t next = latest;
    uint32_t count = data_get(&next, W_COUNTS, ERRORS_SHIFT, ERRORS_BITS);
    data_put(&next, W_COUNTS, ERRORS_SHIFT, ERRORS_BITS,
             terrain_process_count_add_sat(count, 1u, ERRORS_BITS));
    if (b_reset)
    {
        count = data_get(&next, W_COUNTS, RESETS_SHIFT, RESETS_BITS);
        data_put(&next, W_COUNTS, RESETS_SHIFT, RESETS_BITS,
                 terrain_process_count_add_sat(count, 1u, RESETS_BITS));
    }
    store_latest(&next);
}

/* Run the detectors on the newest sample: humps first, because the
 * collision check needs this sample's hump state. */
static void
detect_sample (terrain_vec3_t const * p_accel, terrain_hint_t const * p_hint,
               uint32_t now_10ms)
{
    terrain_hump_sample_t const sample = terrain_detect_hump_sample(
        p_hint->b_following, (tilt.flags & TERRAIN_TILT_ROLL_HELD) != 0u,
        p_hint->speed_mm_s, TERRAIN_HUMP_MIN_SPEED_MM_S);
    if (TERRAIN_HUMP_SAMPLE_DROP == sample)
    {
        detect_word = terrain_detect_hump_clear(detect_word);
        terrain_height_clear_hump(&height);
    }
    else if (TERRAIN_HUMP_SAMPLE_USE == sample)
    {
        detect_hump(p_accel, p_hint, now_10ms);
    }
    else
    {
        /* Paused. */
    }
    detect_collision(p_accel, p_hint, now_10ms);
}

/*
 * Run the hump height and the hump state machine on the newest sample and
 * queue the event. The machine uses the smoothed roll; the height its own
 * lighter smoothing of this sample's roll.
 */
static void
detect_hump (terrain_vec3_t const * p_accel, terrain_hint_t const * p_hint,
             uint32_t now_10ms)
{
    /* Height first: it needs the state before this sample's step. */
    terrain_height_step(&height, terrain_detect_hump_state(detect_word),
                        terrain_process_roll_deg(p_accel), &height_limits);

    terrain_event_type_t const type =
        terrain_detect_hump_step(&detect_word, tilt.roll_deg, &hump_limits);
    if (type == TERRAIN_EVENT_NONE)
    {
        return;
    }

    if (type == TERRAIN_EVENT_ASCENDING)
    {
        hump_start_mm = p_hint->distance_mm;
    }

    /* ASCENDING and DESCENDING carry the lean; LEVEL the hump height. */
    uint32_t value = 0u;
    if ((type == TERRAIN_EVENT_ASCENDING) || (type == TERRAIN_EVENT_DESCENDING))
    {
        value = terrain_process_encode_signed(
            fabsf(tilt.roll_deg), DDEG_PER_DEG, TERRAIN_EVENT_VALUE_BITS);
    }
    else if (type == TERRAIN_EVENT_LEVEL)
    {
        value = terrain_height_mm(&height);
        terrain_height_clear_hump(&height);
        if (!terrain_detect_hump_lasted(
                hump_start_mm, p_hint->distance_mm,
                terrain_detect_hump_min_mm(
                    p_hint->speed_mm_s, TERRAIN_HUMP_SLOW_MM_S,
                    TERRAIN_HUMP_SLOW_MIN_MM, TERRAIN_HUMP_MIN_MM)))
        {
            return; /* The lean of a steering correction: not counted. */
        }
    }
    else
    {
        /* The hump step sends no other kinds. */
    }
    queue_event(type, value, now_10ms);
}

/*
 * Run the turning state machine on one sample and queue TURNING or
 * STRAIGHT (value 0). The rate is the batch's: motion updates every
 * 20 ms, so the samples of one batch share it.
 */
static void
detect_turn (float turn_dps, uint32_t now_10ms)
{
    terrain_event_type_t const type =
        terrain_detect_turn_step(&detect_word, turn_dps, &turn_limits);
    if (type != TERRAIN_EVENT_NONE)
    {
        queue_event(type, 0u, now_10ms);
    }
}

/*
 * Run the speed state machine on one sample and queue STATIONARY,
 * ACCELERATING, CRUISING or DECELERATING (value 0). The speed is the
 * batch's, as for detect_turn().
 */
static void
detect_speed (float wheel_mm_s, uint32_t now_10ms)
{
    terrain_event_type_t const type =
        terrain_detect_speed_step(&speed_word, wheel_mm_s, &speed_limits);
    if (type != TERRAIN_EVENT_NONE)
    {
        queue_event(type, 0u, now_10ms);
    }
}

/*
 * Run the collision state machine on the newest sample: the horizontal
 * push of the unsmoothed acceleration (a hit lasts about 50 ms, which the
 * tilt smoothing would flatten) and motion's ground speed.
 */
static void
detect_collision (terrain_vec3_t const * p_accel, terrain_hint_t const * p_hint,
                  uint32_t now_10ms)
{
    float const push_g =
        sqrtf((p_accel->x * p_accel->x) + (p_accel->y * p_accel->y)) /
        TERRAIN_GRAVITY_MPS2;
    bool const b_stopped_dead = terrain_detect_stopped_dead(
        &stopped, (int32_t)p_hint->speed_mm_s, push_g,
        p_hint->b_distance_known && p_hint->b_following,
        terrain_detect_hump_state(detect_word), &stopped_limits);
    terrain_collision_input_t const in = {
        terrain_detect_collision_push(
            push_g, p_hint->b_distance_known ? (int32_t)p_hint->speed_mm_s : 0,
            fabsf(tilt.roll_deg), TERRAIN_COLLISION_DRIVING_MM_S,
            TERRAIN_COLLISION_MAX_LEAN_DEG),
        terrain_detect_hump_state(detect_word),
        p_hint->b_distance_known,
        (int32_t)p_hint->speed_mm_s,
        b_stopped_dead,
        stopped.peak_g};
    uint32_t value = 0u;

    if (terrain_detect_collision_step(&collide_word, &in, &collision_limits,
                                      &value) == TERRAIN_EVENT_COLLISION)
    {
        queue_event(TERRAIN_EVENT_COLLISION, value, now_10ms);
    }
}

/*
 * Add an event to the queue and count it in the run summary; the
 * snapshot's highest hump follows the summary's. The lock is held only
 * for that.
 */
static void
queue_event (terrain_event_type_t type, uint32_t value, uint32_t now_10ms)
{
    terrain_event_t const event = terrain_event_make(type, value, now_10ms);

    uint32_t saved = terrain_platform_lock();
    terrain_event_queue_push(&events, event);
    terrain_summary_add(&summary, type, value);
    data_put(&latest, W_STATE, HIGHEST_SHIFT, HIGHEST_BITS,
             terrain_summary_highest_mm(&summary));
    terrain_platform_unlock(saved);
}

/*
 * What the shared snapshot shows while calibrating: no readings; counters,
 * time and the highest hump (a fact about the run) kept, and the turn
 * rate, turning state and speed state (from the wheels, not the
 * calibration). Used for
 * the real reset and by the getters while a request is pending, so both
 * always agree.
 */
static void
show_calibrating (terrain_data_t * p_data)
{
    /* All-zero fields read as CALIBRATING, LEVEL, 0 and not held. */
    uint32_t const time = data_get(p_data, W_STATE, TIME_SHIFT, TIME_BITS);
    uint32_t const highest =
        data_get(p_data, W_STATE, HIGHEST_SHIFT, HIGHEST_BITS);
    uint32_t const turn_rate =
        data_get(p_data, W_TILT, TURN_RATE_SHIFT, TURN_RATE_BITS);
    uint32_t const turning =
        data_get(p_data, W_STATE, TURN_STATE_SHIFT, FLAG_BITS);
    uint32_t const speed =
        data_get(p_data, W_STATE, SPEED_STATE_SHIFT, SPEED_STATE_BITS);
    p_data->word[W_TILT] = 0u;
    p_data->word[W_STATE] = 0u;
    data_put(p_data, W_TILT, TURN_RATE_SHIFT, TURN_RATE_BITS, turn_rate);
    data_put(p_data, W_STATE, TIME_SHIFT, TIME_BITS, time);
    data_put(p_data, W_STATE, HIGHEST_SHIFT, HIGHEST_BITS, highest);
    data_put(p_data, W_STATE, TURN_STATE_SHIFT, FLAG_BITS, turning);
    data_put(p_data, W_STATE, SPEED_STATE_SHIFT, SPEED_STATE_BITS, speed);
    p_data->word[W_ACCEL_XY] = 0u;
    p_data->word[W_ACCEL_Z] = 0u;
}

/* Calibration details at the start of a run of attempts: all zero, which
 * reads as CALIBRATING. */
static void
clear_calib_info (terrain_calib_info_t * p_info)
{
    terrain_calib_info_t const empty_info = {0};
    *p_info = empty_info;
}

/*
 * Pack the results into a local copy, then publish it whole under the
 * lock. A pending recalibration is hidden by the getters.
 */
static void
publish (terrain_vec3_t const * p_accel, float turn_dps, uint32_t new_samples,
         bool b_overrun, uint32_t now_10ms)
{
    terrain_calib_state_t const state = own_calib_state();
    bool const b_valid = (state == TERRAIN_CALIB_VALID);
    terrain_vec3_t const zero = {0.0f, 0.0f, 0.0f};
    terrain_vec3_t const * p_shown = b_valid ? p_accel : &zero;

    /* Only this task writes latest, so it can read it without the lock. */
    terrain_data_t next = latest;

    data_put(&next, W_TILT, PITCH_SHIFT, ANGLE_BITS,
             terrain_process_encode_signed(b_valid ? tilt.pitch_deg : 0.0f,
                                           DDEG_PER_DEG, ANGLE_BITS));
    data_put(&next, W_TILT, ROLL_SHIFT, ANGLE_BITS,
             terrain_process_encode_signed(b_valid ? tilt.roll_deg : 0.0f,
                                           DDEG_PER_DEG, ANGLE_BITS));
    data_put(&next, W_TILT, CALIB_SHIFT, CALIB_BITS, (uint32_t)state);
    data_put(&next, W_TILT, PITCH_HELD_SHIFT, FLAG_BITS,
             (b_valid && ((tilt.flags & TERRAIN_TILT_PITCH_HELD) != 0u)) ? 1u
                                                                         : 0u);
    data_put(&next, W_STATE, ROLL_HELD_SHIFT, FLAG_BITS,
             (b_valid && ((tilt.flags & TERRAIN_TILT_ROLL_HELD) != 0u)) ? 1u
                                                                        : 0u);
    data_put(&next, W_STATE, HUMP_SHIFT, HUMP_BITS,
             b_valid ? (uint32_t)terrain_detect_hump_state(detect_word) : 0u);

    /* Turning comes from the wheels: given in every calibration state. */
    data_put(
        &next, W_TILT, TURN_RATE_SHIFT, TURN_RATE_BITS,
        terrain_process_encode_signed(turn_dps, DPS_PER_STEP, TURN_RATE_BITS));
    data_put(&next, W_STATE, TURN_STATE_SHIFT, FLAG_BITS,
             (uint32_t)terrain_detect_turn_state(detect_word));
    data_put(&next, W_STATE, SPEED_STATE_SHIFT, SPEED_STATE_BITS,
             (uint32_t)terrain_detect_speed_state(speed_word));

    /* 10 ms ticks, wrapping: readers only use differences. */
    data_put(&next, W_STATE, TIME_SHIFT, TIME_BITS, now_10ms);

    uint32_t count = data_get(&next, W_COUNTS, SAMPLES_SHIFT, SAMPLES_BITS);
    data_put(&next, W_COUNTS, SAMPLES_SHIFT, SAMPLES_BITS,
             terrain_process_count_add_wrap(count, new_samples, SAMPLES_BITS));
    if (b_overrun)
    {
        count = data_get(&next, W_COUNTS, OVERRUN_SHIFT, OVERRUN_BITS);
        data_put(&next, W_COUNTS, OVERRUN_SHIFT, OVERRUN_BITS,
                 terrain_process_count_add_sat(count, 1u, OVERRUN_BITS));
    }

    data_put(
        &next, W_ACCEL_XY, ACCEL_X_SHIFT, ACCEL_BITS,
        terrain_process_encode_signed(p_shown->x, STEPS_PER_MPS2, ACCEL_BITS));
    data_put(
        &next, W_ACCEL_XY, ACCEL_Y_SHIFT, ACCEL_BITS,
        terrain_process_encode_signed(p_shown->y, STEPS_PER_MPS2, ACCEL_BITS));
    data_put(
        &next, W_ACCEL_Z, ACCEL_Z_SHIFT, ACCEL_BITS,
        terrain_process_encode_signed(p_shown->z, STEPS_PER_MPS2, ACCEL_BITS));

    store_latest(&next);
}

/*
 * Publish one attempt's measurements with the state it led to, under one
 * lock. Spread and tilt round up, so they never read better than they were.
 */
static void
publish_calib (terrain_calib_out_t const * p_out, bool b_moved)
{
    /* Only this task writes these, so it can read them without the lock. */
    terrain_calib_info_t next_info = calib_info;
    terrain_data_t next_data = latest;

    uint32_t word = next_info.word;
    uint32_t count =
        terrain_process_field_get(word, ATTEMPTS_SHIFT, ATTEMPT_BITS);
    word = terrain_process_field_put(
        word, ATTEMPTS_SHIFT, ATTEMPT_BITS,
        terrain_process_count_add_sat(count, 1u, ATTEMPT_BITS));
    if (b_moved)
    {
        count = terrain_process_field_get(word, RETRIES_SHIFT, ATTEMPT_BITS);
        word = terrain_process_field_put(
            word, RETRIES_SHIFT, ATTEMPT_BITS,
            terrain_process_count_add_sat(count, 1u, ATTEMPT_BITS));
    }
    word = terrain_process_field_put(
        word, SPREAD_SHIFT, SPREAD_BITS,
        terrain_process_encode_unsigned(p_out->spread_mps2, STEPS_PER_MPS2,
                                        SPREAD_BITS, true));
    word = terrain_process_field_put(
        word, CALIB_TILT_SHIFT, CALIB_TILT_BITS,
        terrain_process_encode_unsigned(p_out->tilt_deg, DDEG_PER_DEG,
                                        CALIB_TILT_BITS, true));
    next_info.word = word;
    next_info.gravity_x_mps2 = p_out->mean.x;
    next_info.gravity_y_mps2 = p_out->mean.y;
    next_info.gravity_z_mps2 = p_out->mean.z;

    data_put(&next_data, W_TILT, CALIB_SHIFT, CALIB_BITS,
             (uint32_t)own_calib_state());

    uint32_t saved = terrain_platform_lock();
    calib_info = next_info;
    latest = next_data;
    terrain_platform_unlock(saved);
}

static void
store_latest (terrain_data_t const * p_data)
{
    uint32_t saved = terrain_platform_lock();
    latest = *p_data;
    terrain_platform_unlock(saved);
}

static uint32_t
own_get (uint32_t shift, uint32_t width)
{
    return terrain_process_field_get(own_state, shift, width);
}

static void
own_put (uint32_t shift, uint32_t width, uint32_t value)
{
    own_state = terrain_process_field_put(own_state, shift, width, value);
}

static terrain_calib_state_t
own_calib_state (void)
{
    return (terrain_calib_state_t)own_get(OWN_CALIB_SHIFT, CALIB_BITS);
}

/* One field of a snapshot; 0 if p_data is NULL. */
static uint32_t
data_get (terrain_data_t const * p_data, uint32_t word, uint32_t shift,
          uint32_t width)
{
    if (p_data == NULL)
    {
        return 0u;
    }
    return terrain_process_field_get(p_data->word[word], shift, width);
}

static void
data_put (terrain_data_t * p_data, uint32_t word, uint32_t shift,
          uint32_t width, uint32_t code)
{
    p_data->word[word] =
        terrain_process_field_put(p_data->word[word], shift, width, code);
}

static int16_t
data_accel_mg (terrain_data_t const * p_data, uint32_t word, uint32_t shift)
{
    uint32_t const code = data_get(p_data, word, shift, ACCEL_BITS);
    return (int16_t)(terrain_process_decode_signed(code, ACCEL_BITS) *
                     MG_PER_STEP);
}
