/**
 * @file terrain_compass.h
 * @brief GY-511 magnetometer as a compass: the car's heading from the
 *        Earth's field, independent of the wheels.
 *
 * The terrain task sets it up after the accelerometer and reads it on
 * every update; any task may read the latest result. The heading turns
 * the same way as motion's odometry heading (+ is left), so the two can
 * be compared: a wheel that slips turns the odometry but not the compass.
 * The car's own magnets add a fixed offset that must be measured once
 * with the calibration spin (TER-U28) and set as TERRAIN_MAG_OFFSET_X and
 * _Y. Until then, and near steel or motor currents, the heading is only
 * as good as the field it sees: use it for comparison, not yet for
 * steering.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_COMPASS_H
#define TERRAIN_COMPASS_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "terrain_monitor.h"

/* Public types, constants, and macros. */

/** @brief The latest compass reading. */
typedef struct
{
    bool b_valid;         /**< A sample was read and no axis overflowed. */
    int16_t x;            /**< Car X (right), counts, offset not removed. */
    int16_t y;            /**< Car Y (forward), counts, offset not
                               removed. */
    int16_t z;            /**< Car Z (up), counts. */
    int16_t heading_ddeg; /**< Tenths of a degree, 0 to 3599,
                               counter-clockwise from magnetic north (+ is
                               left), TERRAIN_MAG_OFFSET_* removed. */
    uint16_t samples;     /**< Samples read since start-up; wraps. */
} terrain_compass_t;

/**
 * @brief Field seen on X and Y since start-up, for the calibration spin:
 *        the offsets are the centres, (min + max) / 2.
 */
typedef struct
{
    int16_t min_x; /**< Smallest car X, counts. */
    int16_t max_x; /**< Largest car X, counts. */
    int16_t min_y; /**< Smallest car Y, counts. */
    int16_t max_y; /**< Largest car Y, counts. */
} terrain_compass_range_t;

/* Public function declarations. */

/**
 * @brief Set up the magnetometer.
 * @return TERRAIN_OK, or TERRAIN_ERR_NOT_FOUND / _SENSOR / _BUS_SETUP; on
 *         failure terrain_compass_update() retries every second.
 * Call from the terrain task only, after terrain_monitor_init() has
 * succeeded (it opens the bus). The accelerometer works without it.
 */
terrain_status_t terrain_compass_init (void);

/**
 * @brief Read one sample and publish it.
 * Call from the terrain task only, on every update (10 ms). A failed read
 * marks the reading invalid and sets the sensor up again a second later.
 */
void terrain_compass_update (void);

/**
 * @brief Copy the latest reading.
 * @param p_out Receives the reading; all zero before the first sample.
 * @return True if @p p_out is valid.
 * Any task; takes the terrain platform lock briefly, never waits.
 */
bool terrain_compass_get (terrain_compass_t * p_out);

/**
 * @brief Copy the X and Y range seen since start-up.
 * @param p_out Receives the range; all zero before the first sample.
 * Terrain task only.
 */
void terrain_compass_get_range (terrain_compass_range_t * p_out);

#endif /* TERRAIN_COMPASS_H */
