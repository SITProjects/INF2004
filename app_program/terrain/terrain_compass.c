/**
 * @file terrain_compass.c
 * @brief GY-511 magnetometer as a compass.
 *
 * With the car flat, the Earth's horizontal field points north. Seen from
 * a car turned ψ counter-clockwise from north, it reads H sin ψ on the
 * car's X (right) axis and H cos ψ on Y (forward), so ψ = atan2(X, Y).
 * The board axes are mapped to the car's with the accelerometer's
 * TERRAIN_CAR_* settings: the LSM303DLHC's two sensors share their axes.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "terrain_compass.h"
#include <math.h>
#include <stddef.h>
#include "../robot_types.h"
#include "drv_imu.h"
#include "terrain_config.h"
#include "terrain_platform.h"

/* Private types, constants, and macros. */
#define BOARD_AXES 3u
#define DDEG_PER_TURN 3600
#define DDEG_PER_RAD (1800.0f / 3.14159265f)
#define HALF 0.5f
/* Updates between set-up attempts after a failure: 1 s at 10 ms. */
#define RETRY_UPDATES 100u

/* Static data. */

/* Shared with other tasks; access only under terrain_platform_lock(). */
static uint32_t latest[ROBOT_COMPASS_WORDS];

/* Terrain task only. */
static bool b_ready = false;
static uint32_t retry_wait = 0u;
static terrain_compass_range_t range;
static bool b_range_started = false;

/* Private function declarations. */
static int16_t car_axis (int16_t const * p_board, int32_t from, int32_t sign);
static int16_t heading_ddeg (int16_t x, int16_t y);
static void track_range (int16_t x, int16_t y);
static void publish (terrain_compass_t const * p_reading);
static terrain_compass_t unpack (void);

/* Public function definitions. */

terrain_status_t
terrain_compass_init (void)
{
    imu_status_t const status =
        imu_mag_init(TERRAIN_MAG_ADDR, TERRAIN_MAG_ODR, TERRAIN_MAG_GAIN);
    b_ready = (IMU_OK == status);
    retry_wait = RETRY_UPDATES;
    if (IMU_OK == status)
    {
        return TERRAIN_OK;
    }
    return (IMU_ERR_NOT_FOUND == status) ? TERRAIN_ERR_NOT_FOUND
                                         : TERRAIN_ERR_SENSOR;
}

void
terrain_compass_update (void)
{
    if (!b_ready)
    {
        if (retry_wait > 0u)
        {
            retry_wait--;
        }
        else
        {
            (void)terrain_compass_init();
        }
        return;
    }

    imu_mag_raw_t raw = {0, 0, 0};
    terrain_compass_t reading = unpack(); /* Only this task writes it. */
    if (imu_mag_read(&raw) != IMU_OK)
    {
        b_ready = false;
        retry_wait = RETRY_UPDATES;
        reading.b_valid = false;
        publish(&reading);
        return;
    }

    reading.samples = (uint8_t)(reading.samples + 1u);
    if ((IMU_MAG_OVERFLOW == raw.x) || (IMU_MAG_OVERFLOW == raw.y) ||
        (IMU_MAG_OVERFLOW == raw.z))
    {
        /* A magnet too close, or the range too small. */
        reading.b_valid = false;
        publish(&reading);
        return;
    }

    int16_t const board[BOARD_AXES] = {raw.x, raw.y, raw.z};
    reading.x = car_axis(board, TERRAIN_CAR_X_FROM, TERRAIN_CAR_X_SIGN);
    reading.y = car_axis(board, TERRAIN_CAR_Y_FROM, TERRAIN_CAR_Y_SIGN);
    reading.z = car_axis(board, TERRAIN_CAR_Z_FROM, TERRAIN_CAR_Z_SIGN);
    reading.heading_ddeg = heading_ddeg(reading.x, reading.y);
    reading.b_valid = true;
    track_range(reading.x, reading.y);
    publish(&reading);
}

bool
terrain_compass_get (terrain_compass_t * p_out)
{
    if (NULL == p_out)
    {
        return false;
    }
    uint32_t const saved = terrain_platform_lock();
    *p_out = unpack();
    terrain_platform_unlock(saved);
    return p_out->b_valid;
}

void
terrain_compass_get_range (terrain_compass_range_t * p_out)
{
    if (p_out != NULL)
    {
        *p_out = range;
    }
}

/* Private function definitions. */

static int16_t
car_axis (int16_t const * p_board, int32_t from, int32_t sign)
{
    return (int16_t)(p_board[from] * sign);
}

/* Counter-clockwise from magnetic north, 0 to 3599 tenths of a degree. */
static int16_t
heading_ddeg (int16_t x, int16_t y)
{
    float const east = (float)(x - (int16_t)TERRAIN_MAG_OFFSET_X);
    float const north = (float)(y - (int16_t)TERRAIN_MAG_OFFSET_Y);
    int32_t ddeg = (int32_t)floorf((atan2f(east, north) * DDEG_PER_RAD) + HALF);
    if (ddeg < 0)
    {
        ddeg += DDEG_PER_TURN;
    }
    if (ddeg >= DDEG_PER_TURN)
    {
        ddeg -= DDEG_PER_TURN;
    }
    return (int16_t)ddeg;
}

static void
track_range (int16_t x, int16_t y)
{
    if (!b_range_started)
    {
        range.min_x = x;
        range.max_x = x;
        range.min_y = y;
        range.max_y = y;
        b_range_started = true;
        return;
    }
    range.min_x = (x < range.min_x) ? x : range.min_x;
    range.max_x = (x > range.max_x) ? x : range.max_x;
    range.min_y = (y < range.min_y) ? y : range.min_y;
    range.max_y = (y > range.max_y) ? y : range.max_y;
}

static void
publish (terrain_compass_t const * p_reading)
{
    uint32_t first = robot_field_put(
        0u, ROBOT_COMPASS_X_SHIFT, ROBOT_COMPASS_AXIS_BITS,
        robot_field_from_signed(p_reading->x, ROBOT_COMPASS_AXIS_BITS));
    first = robot_field_put(
        first, ROBOT_COMPASS_Y_SHIFT, ROBOT_COMPASS_AXIS_BITS,
        robot_field_from_signed(p_reading->y, ROBOT_COMPASS_AXIS_BITS));
    first = robot_field_put(first, ROBOT_COMPASS_SAMPLES_SHIFT,
                            ROBOT_COMPASS_SAMPLES_BITS, p_reading->samples);
    uint32_t second = robot_field_put(
        p_reading->b_valid ? ROBOT_COMPASS_VALID : 0u, ROBOT_COMPASS_Z_SHIFT,
        ROBOT_COMPASS_AXIS_BITS,
        robot_field_from_signed(p_reading->z, ROBOT_COMPASS_AXIS_BITS));
    second = robot_field_put(second, ROBOT_COMPASS_HEADING_SHIFT,
                             ROBOT_COMPASS_HEADING_BITS,
                             (uint32_t)p_reading->heading_ddeg);

    uint32_t const saved = terrain_platform_lock();
    latest[0] = first;
    latest[1] = second;
    terrain_platform_unlock(saved);
}

/* The latest reading, decoded. */
static terrain_compass_t
unpack (void)
{
    terrain_compass_t out;

    out.b_valid = (latest[1] & ROBOT_COMPASS_VALID) != 0u;
    out.x = (int16_t)robot_field_to_signed(
        robot_field_get(latest[0], ROBOT_COMPASS_X_SHIFT,
                        ROBOT_COMPASS_AXIS_BITS),
        ROBOT_COMPASS_AXIS_BITS);
    out.y = (int16_t)robot_field_to_signed(
        robot_field_get(latest[0], ROBOT_COMPASS_Y_SHIFT,
                        ROBOT_COMPASS_AXIS_BITS),
        ROBOT_COMPASS_AXIS_BITS);
    out.z = (int16_t)robot_field_to_signed(
        robot_field_get(latest[1], ROBOT_COMPASS_Z_SHIFT,
                        ROBOT_COMPASS_AXIS_BITS),
        ROBOT_COMPASS_AXIS_BITS);
    out.heading_ddeg = (int16_t)robot_field_get(
        latest[1], ROBOT_COMPASS_HEADING_SHIFT, ROBOT_COMPASS_HEADING_BITS);
    out.samples = (uint8_t)robot_field_get(
        latest[0], ROBOT_COMPASS_SAMPLES_SHIFT, ROBOT_COMPASS_SAMPLES_BITS);
    return out;
}
