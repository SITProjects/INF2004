/**
 * @file terrain.c
 * @brief IMU motion and terrain monitoring: coordinator and calibration.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain.h"
#include <stdbool.h>
#include <stddef.h>
#include "imu.h"
#include "imu_bus.h"
#include "terrain_config.h"
#include "terrain_platform.h"
#include "terrain_process.h"

/* Private types, constants, and macros. */

/* Static data. */

/* Mounting of the board on the car, from terrain_config.h. */
static terrain_axis_map_t const axis_map = {
    {TERRAIN_CAR_X_FROM, TERRAIN_CAR_Y_FROM, TERRAIN_CAR_Z_FROM},
    {TERRAIN_CAR_X_SIGN, TERRAIN_CAR_Y_SIGN, TERRAIN_CAR_Z_SIGN}};

static terrain_data_t latest;
static bool b_ready = false;

/* Private function declarations. */
static terrain_status_t read_samples (imu_accel_raw_t * p_buf,
                                      uint32_t * p_count, bool * p_overrun);
static void publish (terrain_vec3_t const * p_accel, uint32_t new_samples,
                     bool b_overrun);
static void count_error (void);

/* Public function definitions. */

terrain_status_t
terrain_init (void)
{
    b_ready = false;

    if (!imu_bus_init(TERRAIN_I2C_UNIT, TERRAIN_I2C_SDA_PIN,
                      TERRAIN_I2C_SCL_PIN, TERRAIN_I2C_BAUD_HZ,
                      TERRAIN_I2C_TIMEOUT_US))
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

    terrain_data_t const empty = {0};
    uint32_t saved = terrain_platform_lock();
    latest = empty;
    terrain_platform_unlock(saved);

    b_ready = true;
    return TERRAIN_OK;
}

terrain_status_t
terrain_update (void)
{
    if (!b_ready)
    {
        return TERRAIN_ERR_NOT_INIT;
    }

    /* Static keeps ~200 bytes off the stack; one caller only. */
    static imu_accel_raw_t buf[IMU_ACCEL_FIFO_DEPTH];
    uint32_t count = 0u;
    bool b_overrun = false;

    terrain_status_t status = read_samples(buf, &count, &b_overrun);
    if (status != TERRAIN_OK)
    {
        count_error();
        return status;
    }
    if (count == 0u)
    {
        return TERRAIN_OK;
    }

    float scale = imu_accel_scale_mps2();
    terrain_vec3_t accel = {0.0f, 0.0f, 0.0f};

    for (uint32_t i = 0u; i < count; i++)
    {
        int16_t const board[TERRAIN_AXES] = {buf[i].x, buf[i].y, buf[i].z};
        accel = terrain_process_map_axes(&axis_map, board, scale);
    }

    publish(&accel, count, b_overrun);
    return TERRAIN_OK;
}

terrain_status_t
terrain_get_latest (terrain_data_t * p_out)
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
    return TERRAIN_OK;
}

char const *
terrain_status_text (terrain_status_t status)
{
    switch (status)
    {
        case TERRAIN_OK:
            return "OK";
        case TERRAIN_ERR_NOT_INIT:
            return "terrain_init() has not succeeded yet";
        case TERRAIN_ERR_BUS_SETUP:
            return "I2C setup failed: check the unit and pins in "
                   "terrain_config.h";
        case TERRAIN_ERR_NOT_FOUND:
            return "accelerometer not found: check the cable, Grove port "
                   "and 3.3 V power";
        case TERRAIN_ERR_SENSOR:
            return "accelerometer setup or read failed";
        case TERRAIN_ERR_PARAM:
            return "invalid argument";
        default:
            return "unknown status";
    }
}

/* Private function definitions. */

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

static void
publish (terrain_vec3_t const * p_accel, uint32_t new_samples, bool b_overrun)
{
    uint64_t now_us = terrain_platform_time_us();

    uint32_t saved = terrain_platform_lock();
    latest.ax_mps2 = p_accel->x;
    latest.ay_mps2 = p_accel->y;
    latest.az_mps2 = p_accel->z;
    latest.timestamp_us = now_us;
    latest.sample_count += new_samples;
    if (b_overrun)
    {
        latest.overrun_count++;
    }
    terrain_platform_unlock(saved);
}

static void
count_error (void)
{
    uint32_t saved = terrain_platform_lock();
    latest.error_count++;
    terrain_platform_unlock(saved);
}
