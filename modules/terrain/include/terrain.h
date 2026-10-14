/**
 * @file terrain.h
 * @brief IMU motion and terrain monitoring: public interface.
 *
 * Phase 1: initialise the GY-511 and read acceleration in the car's
 * axes. Tilt, motion state, bump height and events (terrain_poll_event)
 * are added in Phases 3 to 4.
 *
 * Call terrain_init() once, then terrain_update() regularly (about
 * every 10 ms) from one place only. terrain_get_latest() may be called
 * from anywhere, including other tasks.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_H
#define TERRAIN_H

/* Public includes. */
#include <stdint.h>

/* Public types, constants, and macros. */

/** @brief Result of a terrain module call. */
typedef enum
{
    TERRAIN_OK = 0,
    TERRAIN_ERR_NOT_INIT,  /**< terrain_init() has not succeeded yet. */
    TERRAIN_ERR_BUS_SETUP, /**< Invalid I2C unit or pins in config. */
    TERRAIN_ERR_NOT_FOUND, /**< Accelerometer did not answer. */
    TERRAIN_ERR_SENSOR,    /**< Sensor setup or a read failed. */
    TERRAIN_ERR_PARAM      /**< Invalid argument. */
} terrain_status_t;

/**
 * @brief Snapshot of the module's latest results.
 * Acceleration is in the car's axes: X right, Y forward, Z up. A car
 * sitting flat reads about +9.81 on Z, because the sensor feels the
 * ground pushing up against gravity.
 */
typedef struct
{
    float ax_mps2;          /**< Acceleration along X, m/s^2. */
    float ay_mps2;          /**< Acceleration along Y, m/s^2. */
    float az_mps2;          /**< Acceleration along Z, m/s^2. */
    uint64_t timestamp_us;  /**< When the newest sample was read. */
    uint32_t sample_count;  /**< Samples processed since init. */
    uint32_t overrun_count; /**< Times the sensor buffer overflowed. */
    uint32_t error_count;   /**< Failed sensor reads since init. */
} terrain_data_t;

/* Public function declarations. */

/**
 * @brief Set up the I2C bus and the accelerometer from terrain_config.h.
 * @return TERRAIN_OK, or the reason it failed; see terrain_status_text().
 * Safe to call again to retry, for example after plugging the sensor in.
 */
terrain_status_t terrain_init (void);

/**
 * @brief Read and process every new sample from the sensor.
 * @return TERRAIN_OK, TERRAIN_ERR_NOT_INIT or TERRAIN_ERR_SENSOR.
 * Call about every 10 ms from a single task or loop. Never waits for
 * data: if no new sample is ready it returns immediately. Each I2C
 * transfer is bounded by TERRAIN_I2C_TIMEOUT_US.
 */
terrain_status_t terrain_update (void);

/**
 * @brief Copy the latest results.
 * @param p_out Receives the snapshot.
 * @return TERRAIN_OK, TERRAIN_ERR_NOT_INIT or TERRAIN_ERR_PARAM.
 * Safe to call from other tasks while terrain_update() runs.
 */
terrain_status_t terrain_get_latest (terrain_data_t * p_out);

/**
 * @brief Human-readable description of a status, for error messages.
 * @param status Any terrain_status_t value.
 * @return A constant string; never NULL.
 */
char const * terrain_status_text (terrain_status_t status);

#endif /* TERRAIN_H */
