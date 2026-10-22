/**
 * @file terrain_monitor.h
 * @brief IMU motion and terrain monitoring: public interface.
 *
 * Phase 1: initialise the GY-511 and read acceleration in the car's
 * axes. Phase 2: calibrate at every start, then correct every reading.
 * Phase 3: pitch and roll. Events, turn rate, collisions, motion state
 * and hump height follow in later phases.
 *
 * Call terrain_monitor_init() once, then terrain_monitor_update()
 * regularly (about every 10 ms) from one place only. Calibration then takes
 * TERRAIN_CALIB_SAMPLES samples (2 s) with the car standing still on
 * level ground; an attempt in which the car moved is repeated. Readings
 * are given out only once the calibration state is TERRAIN_CALIB_VALID.
 *
 * terrain_monitor_get_latest(), terrain_monitor_get_calibration() and
 * terrain_monitor_recalibrate() may be called from anywhere, including other
 * tasks.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_MONITOR_H
#define TERRAIN_MONITOR_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** @brief Result of a terrain module call. */
typedef enum
{
    TERRAIN_OK = 0,
    TERRAIN_ERR_NOT_INIT,      /**< terrain_monitor_init() not succeeded. */
    TERRAIN_ERR_BUS_SETUP,     /**< Kernel I2C device did not open. */
    TERRAIN_ERR_NOT_FOUND,     /**< Accelerometer did not answer. */
    TERRAIN_ERR_SENSOR,        /**< Sensor setup or a read failed. */
    TERRAIN_ERR_PARAM,         /**< Invalid argument. */
    TERRAIN_ERR_NOT_CALIBRATED /**< Calibration state is not VALID. */
} terrain_status_t;

/** @brief Calibration state. */
typedef enum
{
    TERRAIN_CALIB_CALIBRATING = 0, /**< Measuring; keep the car still. */
    TERRAIN_CALIB_NOT_LEVEL, /**< Ground not level; move the car and reset. */
    TERRAIN_CALIB_VALID      /**< Calibrated; readings are corrected. */
} terrain_calib_state_t;

/**
 * @brief Snapshot of the module's latest results.
 * Acceleration is in the car's axes: X right, Y forward, Z up, with
 * gains and calibration bias applied. A car sitting still on level
 * ground reads 0, 0, +9.81, because the sensor feels the ground pushing
 * up against gravity.
 *
 * Pitch and roll are smoothed over TERRAIN_TILT_TAU_S. While the car
 * speeds up or brakes, pitch keeps its last value and b_pitch_held is
 * true; while it turns, the same goes for roll. Those hints come from
 * other subsystems; until they exist, nothing is held.
 *
 * The acceleration and tilt fields are 0 (and nothing is held) unless
 * calib_state is TERRAIN_CALIB_VALID.
 */
typedef struct
{
    float ax_mps2;               /**< Acceleration along X, m/s^2. */
    float ay_mps2;               /**< Acceleration along Y, m/s^2. */
    float az_mps2;               /**< Acceleration along Z, m/s^2. */
    float pitch_deg;             /**< Nose up (+) or down (-), degrees. */
    float roll_deg;              /**< Right side up (+) or down, degrees. */
    bool b_pitch_held;           /**< Pitch held: speeding up/braking. */
    bool b_roll_held;            /**< Roll held: the car is turning. */
    uint64_t timestamp_us;       /**< When the newest sample was read. */
    uint32_t sample_count;       /**< Samples processed since init. */
    uint32_t overrun_count;      /**< Times the sensor buffer overflowed. */
    uint32_t error_count;        /**< Failed sensor reads since init. */
    uint32_t sensor_reset_count; /**< Sensor set up again after faults. */
    terrain_calib_state_t calib_state; /**< Current calibration state. */
} terrain_data_t;

/**
 * @brief Details of the current calibration, for status displays and
 *        for recording the level reference.
 * Vectors are in the car's axes, m/s^2. The gravity_* fields are the
 * last attempt's average with gains applied but no bias removed: on
 * level ground they are the values for TERRAIN_LEVEL_REF_*.
 */
typedef struct
{
    terrain_calib_state_t state; /**< Current calibration state. */
    uint32_t attempt_count;      /**< Attempts finished in this run. */
    uint32_t retry_count;        /**< Attempts rejected as moving. */
    float gravity_x_mps2;        /**< Last attempt's average, X. */
    float gravity_y_mps2;        /**< Last attempt's average, Y. */
    float gravity_z_mps2;        /**< Last attempt's average, Z. */
    float spread_mps2;           /**< Last attempt's largest (max - min). */
    float tilt_deg;              /**< Last attempt's angle from level ref. */
    float bias_x_mps2;           /**< Bias in use, X; 0 unless VALID. */
    float bias_y_mps2;           /**< Bias in use, Y; 0 unless VALID. */
    float bias_z_mps2;           /**< Bias in use, Z; 0 unless VALID. */
} terrain_calib_info_t;

/* Public function declarations. */

/**
 * @brief Set up the I2C bus and the accelerometer from terrain_config.h,
 *        and start calibrating.
 * @return TERRAIN_OK, or the reason it failed; see
 *         terrain_monitor_status_text().
 * Does not wait for calibration: terrain_monitor_update() collects the
 * samples. Clears all counters. Safe to call again to retry, for example
 * after plugging the sensor in, but only from the task that calls
 * terrain_monitor_update(); use terrain_monitor_recalibrate() from other
 * tasks.
 */
terrain_status_t terrain_monitor_init (void);

/**
 * @brief Read and process every new sample from the sensor.
 * @return TERRAIN_OK, TERRAIN_ERR_NOT_INIT or TERRAIN_ERR_SENSOR.
 * Call about every 10 ms from a single task or loop. Never waits for
 * data: if no new sample is ready it returns immediately. Each I2C
 * transfer is bounded by TERRAIN_I2C_TIMEOUT_US.
 *
 * While calibrating, samples are collected instead of given out. An
 * attempt in which the car moved is retried without limit. Once VALID,
 * every sample also updates pitch and roll. After
 * TERRAIN_FAULT_RETRY_UPDATES failed reads in a row, the bus and sensor
 * are set up again; the calibration is kept.
 */
terrain_status_t terrain_monitor_update (void);

/**
 * @brief Copy the latest results.
 * @param p_out Receives the snapshot.
 * @return TERRAIN_OK; TERRAIN_ERR_NOT_CALIBRATED if the calibration
 *         state is not VALID, in which case the snapshot is still copied
 *         but its acceleration fields are 0; TERRAIN_ERR_NOT_INIT or
 *         TERRAIN_ERR_PARAM.
 * Safe to call from other tasks while terrain_monitor_update() runs.
 */
terrain_status_t terrain_monitor_get_latest (terrain_data_t * p_out);

/**
 * @brief Copy the calibration details.
 * @param p_out Receives the details.
 * @return TERRAIN_OK, TERRAIN_ERR_NOT_INIT or TERRAIN_ERR_PARAM.
 * Safe to call from other tasks while terrain_monitor_update() runs.
 */
terrain_status_t terrain_monitor_get_calibration (terrain_calib_info_t * p_out);

/**
 * @brief Throw away the current calibration and measure again.
 * @return TERRAIN_OK or TERRAIN_ERR_NOT_INIT.
 * The state becomes TERRAIN_CALIB_CALIBRATING at once; the following
 * terrain_monitor_update() calls do the measuring. Does not touch the sensor,
 * so it is safe to call from other tasks, for example on a mission
 * reset. The car must stand still on level ground for 2 s afterwards.
 */
terrain_status_t terrain_monitor_recalibrate (void);

/**
 * @brief Human-readable description of a status, for error messages.
 * @param status Any terrain_status_t value.
 * @return A constant string; never NULL.
 */
char const * terrain_monitor_status_text (terrain_status_t status);

/**
 * @brief Name of a calibration state, for printing.
 * @param state Any terrain_calib_state_t value.
 * @return A constant string such as "VALID"; never NULL.
 */
char const * terrain_monitor_calib_text (terrain_calib_state_t state);

#endif /* TERRAIN_MONITOR_H */
