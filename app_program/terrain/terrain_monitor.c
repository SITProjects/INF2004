/**
 * @file terrain_monitor.c
 * @brief IMU motion and terrain monitoring: coordinator and calibration.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_monitor.h"
#include <stdbool.h>
#include <stddef.h>
#include "drv_imu.h"
#include "drv_imu_bus.h"
#include "terrain_config.h"
#include "terrain_hint.h"
#include "terrain_platform.h"
#include "terrain_process.h"

/* Private types, constants, and macros. */

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

/* Shared with other tasks; access only under terrain_platform_lock(). */
static terrain_data_t latest;
static terrain_calib_info_t calib_info;
static bool b_recal_request = false;

static bool b_ready = false;

/* Owned by the caller of terrain_monitor_update(). */
static terrain_calib_state_t calib_state = TERRAIN_CALIB_CALIBRATING;
static terrain_calib_acc_t calib_acc;
static terrain_vec3_t bias;
static uint32_t fail_streak = 0u;
static terrain_tilt_t tilt;
static float tilt_alpha = 1.0f;

/* Private function declarations. */
static terrain_status_t setup_sensor (void);
static terrain_status_t read_samples (imu_accel_raw_t * p_buf,
                                      uint32_t * p_count, bool * p_overrun);
static bool take_recal_request (void);
static void calib_start (void);
static void calib_add (terrain_vec3_t const * p_sample);
static void handle_fault (void);
static terrain_calib_state_t shared_state (void);
static void publish (terrain_vec3_t const * p_accel, uint32_t new_samples,
                     bool b_overrun);
static void publish_calib (terrain_calib_out_t const * p_out);

/* Public function definitions. */

terrain_status_t
terrain_monitor_init (void)
{
    b_ready = false;

    terrain_status_t status = setup_sensor();
    if (status != TERRAIN_OK)
    {
        return status;
    }

    terrain_data_t const empty_data = {0};
    terrain_calib_info_t const empty_info = {0};
    uint32_t saved = terrain_platform_lock();
    latest = empty_data;
    calib_info = empty_info;
    b_recal_request = false;
    terrain_platform_unlock(saved);

    fail_streak = 0u;
    tilt_alpha = terrain_process_lowpass_alpha(TERRAIN_TILT_TAU_S,
                                               TERRAIN_SAMPLE_PERIOD_S);
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
    fail_streak = 0u;

    if (count == 0u)
    {
        return TERRAIN_OK;
    }

    float scale = imu_accel_scale_mps2();
    terrain_vec3_t accel = {0.0f, 0.0f, 0.0f};

    /* Speed changes fake pitch and turns fake roll: hold them then. */
    terrain_hint_t hint;
    terrain_hint_read(&hint);
    bool const b_hold_pitch = hint.b_speed_known && hint.b_speed_changing;
    bool const b_hold_roll = hint.b_turn_known && hint.b_turning;

    for (uint32_t i = 0u; i < count; i++)
    {
        int16_t const board[TERRAIN_AXES] = {buf[i].x, buf[i].y, buf[i].z};
        terrain_vec3_t sample =
            terrain_process_map_axes(&axis_map, board_gain, board, scale);

        if (calib_state == TERRAIN_CALIB_CALIBRATING)
        {
            calib_add(&sample);
        }

        /* Not "else": the sample that completes calibration counts too. */
        if (calib_state == TERRAIN_CALIB_VALID)
        {
            accel = terrain_process_correct(&sample, &bias);
            terrain_process_tilt_update(&tilt, &accel, tilt_alpha, b_hold_pitch,
                                        b_hold_roll);
        }
    }

    publish(&accel, count, b_overrun);
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
    terrain_platform_unlock(saved);

    if (p_out->calib_state != TERRAIN_CALIB_VALID)
    {
        return TERRAIN_ERR_NOT_CALIBRATED;
    }
    return TERRAIN_OK;
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
    terrain_platform_unlock(saved);
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
     * task, so the sensor is never used by two tasks at once. The shared
     * copies change now so no caller sees the old VALID state.
     */
    terrain_calib_info_t const empty_info = {0};

    uint32_t saved = terrain_platform_lock();
    b_recal_request = true;
    latest.calib_state = TERRAIN_CALIB_CALIBRATING;
    latest.ax_mps2 = 0.0f;
    latest.ay_mps2 = 0.0f;
    latest.az_mps2 = 0.0f;
    latest.pitch_deg = 0.0f;
    latest.roll_deg = 0.0f;
    latest.b_pitch_held = false;
    latest.b_roll_held = false;
    calib_info = empty_info;
    calib_info.state = TERRAIN_CALIB_CALIBRATING;
    terrain_platform_unlock(saved);
    return TERRAIN_OK;
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

/* Return and clear a request made by terrain_monitor_recalibrate(). */
static bool
take_recal_request (void)
{
    uint32_t saved = terrain_platform_lock();
    bool b_request = b_recal_request;
    b_recal_request = false;
    terrain_platform_unlock(saved);
    return b_request;
}

/* Forget the current calibration and start a new run of attempts. */
static void
calib_start (void)
{
    terrain_vec3_t const zero = {0.0f, 0.0f, 0.0f};
    terrain_calib_info_t const empty_info = {0};

    calib_state = TERRAIN_CALIB_CALIBRATING;
    bias = zero;
    terrain_process_calib_reset(&calib_acc);
    terrain_process_tilt_reset(&tilt);

    uint32_t saved = terrain_platform_lock();
    calib_info = empty_info;
    calib_info.state = shared_state();
    latest.calib_state = shared_state();
    terrain_platform_unlock(saved);
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
            calib_state = TERRAIN_CALIB_VALID;
            break;
        case TERRAIN_CALIB_RESULT_NOT_LEVEL:
            calib_state = TERRAIN_CALIB_NOT_LEVEL;
            break;
        default:
            /* Moved: stay CALIBRATING and try again, without limit. */
            b_moved = true;
            break;
    }

    uint32_t saved = terrain_platform_lock();
    publish_calib(&out);
    calib_info.attempt_count++;
    if (b_moved)
    {
        calib_info.retry_count++;
    }
    terrain_platform_unlock(saved);
}

/*
 * A read failed. Samples collected so far no longer form one continuous
 * still period, so a calibration attempt starts again. After repeated
 * failures the bus and sensor are set up again; the bias is kept.
 */
static void
handle_fault (void)
{
    if (calib_state == TERRAIN_CALIB_CALIBRATING)
    {
        terrain_process_calib_reset(&calib_acc);
    }

    bool b_reset = false;
    fail_streak++;
    if (fail_streak >= TERRAIN_FAULT_RETRY_UPDATES)
    {
        fail_streak = 0u;
        b_reset = (setup_sensor() == TERRAIN_OK);
    }

    uint32_t saved = terrain_platform_lock();
    latest.error_count++;
    if (b_reset)
    {
        latest.sensor_reset_count++;
    }
    terrain_platform_unlock(saved);
}

/*
 * State to show other tasks. A pending recalibration request already
 * counts as CALIBRATING. Call with the lock held.
 */
static terrain_calib_state_t
shared_state (void)
{
    return b_recal_request ? TERRAIN_CALIB_CALIBRATING : calib_state;
}

static void
publish (terrain_vec3_t const * p_accel, uint32_t new_samples, bool b_overrun)
{
    uint64_t now_us = terrain_platform_time_us();

    uint32_t saved = terrain_platform_lock();
    terrain_calib_state_t state = shared_state();
    bool b_valid = (state == TERRAIN_CALIB_VALID);
    latest.ax_mps2 = b_valid ? p_accel->x : 0.0f;
    latest.ay_mps2 = b_valid ? p_accel->y : 0.0f;
    latest.az_mps2 = b_valid ? p_accel->z : 0.0f;
    latest.pitch_deg = b_valid ? tilt.pitch_deg : 0.0f;
    latest.roll_deg = b_valid ? tilt.roll_deg : 0.0f;
    latest.b_pitch_held = b_valid && tilt.b_pitch_held;
    latest.b_roll_held = b_valid && tilt.b_roll_held;
    latest.calib_state = state;
    latest.timestamp_us = now_us;
    latest.sample_count += new_samples;
    if (b_overrun)
    {
        latest.overrun_count++;
    }
    terrain_platform_unlock(saved);
}

/* Copy one attempt's measurements to the shared copy. Lock held. */
static void
publish_calib (terrain_calib_out_t const * p_out)
{
    calib_info.state = shared_state();
    calib_info.gravity_x_mps2 = p_out->mean.x;
    calib_info.gravity_y_mps2 = p_out->mean.y;
    calib_info.gravity_z_mps2 = p_out->mean.z;
    calib_info.spread_mps2 = p_out->spread_mps2;
    calib_info.tilt_deg = p_out->tilt_deg;
    calib_info.bias_x_mps2 = bias.x;
    calib_info.bias_y_mps2 = bias.y;
    calib_info.bias_z_mps2 = bias.z;
    latest.calib_state = calib_info.state;
}
