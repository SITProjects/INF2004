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
 * regularly (about every 10 ms) from one place only. Calibration then skips
 * TERRAIN_CALIB_SETTLE_SAMPLES samples (0.5 s) and takes
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

/* Words in a packed snapshot. */
#define TERRAIN_DATA_WORDS 5u

/**
 * @brief Snapshot of the module's latest results, packed: every field
 *        uses only the bits its range needs. Read it only through the
 *        terrain_monitor_data_*() functions; the layout is private.
 *
 * Acceleration is in the car's axes: X right, Y forward, Z up, with
 * gains and calibration bias applied, in 4 mg steps from -8192 to
 * +8188 mg. A car sitting still on level ground reads 0, 0, +1000 mg,
 * because the sensor feels the ground pushing up against gravity.
 *
 * Pitch and roll are smoothed over TERRAIN_TILT_TAU_S, in tenths of a
 * degree from -51.2 to +51.1 degrees (clamped). While the car speeds up or
 * brakes, pitch keeps its last value and is marked held; while it turns,
 * the same goes for roll. Those hints come from other subsystems; until
 * they exist, nothing is held.
 *
 * The acceleration and tilt fields are 0 (and nothing is held) unless the
 * calibration state is TERRAIN_CALIB_VALID. An all-zero snapshot reads as
 * CALIBRATING with everything 0.
 */
typedef struct
{
    uint32_t word[TERRAIN_DATA_WORDS]; /**< Packed fields. */
} terrain_data_t;

/**
 * @brief Details of the current calibration, for status displays and
 *        for recording the level reference. Counts, spread, tilt and the
 *        state are packed into word; read them through the
 *        terrain_monitor_calib_*() functions.
 *
 * Vectors are in the car's axes, m/s^2. The gravity_* fields are the
 * last attempt's average with gains applied but no bias removed: on
 * level ground they are the values for TERRAIN_LEVEL_REF_*. They stay
 * float because the level reference is made from them. While VALID, the
 * bias in use is gravity - (0, 0, 9.80665); otherwise it is 0.
 */
typedef struct
{
    uint32_t word;        /**< Packed counts, spread, tilt and state. */
    float gravity_x_mps2; /**< Last attempt's average, X. */
    float gravity_y_mps2; /**< Last attempt's average, Y. */
    float gravity_z_mps2; /**< Last attempt's average, Z. */
} terrain_calib_info_t;

/* Public function declarations. */

/**
 * @brief Set up the I2C bus and the accelerometer from terrain_config.h,
 *        and start calibrating.
 * @return TERRAIN_OK, or the reason it failed; see
 *         terrain_monitor_status_text().
 * Does not wait for calibration: terrain_monitor_update() collects the
 * samples. Clears all counters. Raises the IMU fault (terrain_fault.h) if
 * setup fails and clears it if setup succeeds. Safe to call again to retry, for example
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
 * While calibrating, samples are collected instead of given out, after
 * the first TERRAIN_CALIB_SETTLE_SAMPLES are skipped. An
 * attempt in which the car moved is retried without limit. Once VALID,
 * every sample also updates pitch and roll. After
 * TERRAIN_FAULT_RETRY_UPDATES failed reads in a row, the IMU fault is
 * raised and the bus and sensor are set up again; the calibration is kept.
 * The next successful read clears the fault; a shorter run of failures
 * does not raise it.
 */
terrain_status_t terrain_monitor_update (void);

/**
 * @brief Copy the latest results.
 * @param p_out Receives the packed snapshot; read it with the
 *        terrain_monitor_data_*() functions.
 * @return TERRAIN_OK; TERRAIN_ERR_NOT_CALIBRATED if the calibration
 *         state is not VALID, in which case the snapshot is still copied
 *         but its acceleration and tilt fields are 0; TERRAIN_ERR_NOT_INIT
 *         or TERRAIN_ERR_PARAM.
 * Safe to call from other tasks while terrain_monitor_update() runs.
 */
terrain_status_t terrain_monitor_get_latest (terrain_data_t * p_out);

/**
 * @brief Copy the calibration details.
 * @param p_out Receives the details; read the packed part with the
 *        terrain_monitor_calib_*() functions.
 * @return TERRAIN_OK, TERRAIN_ERR_NOT_INIT or TERRAIN_ERR_PARAM.
 * Safe to call from other tasks while terrain_monitor_update() runs.
 */
terrain_status_t terrain_monitor_get_calibration (terrain_calib_info_t * p_out);

/**
 * @brief Throw away the current calibration and measure again.
 * @return TERRAIN_OK or TERRAIN_ERR_NOT_INIT.
 * Only records the request: the next terrain_monitor_update() resets the
 * results and the following ones do the measuring. Until that reset the
 * getters already report TERRAIN_CALIB_CALIBRATING, with zero readings and
 * empty calibration details, so no caller sees the old results. Does not
 * touch the sensor, so it is safe to call from other tasks, for example on
 * a mission reset. The car must stand still on level ground for 2.5 s
 * afterwards.
 */
terrain_status_t terrain_monitor_recalibrate (void);

/**
 * @brief Calibration state in a snapshot.
 * @param p_data Snapshot from terrain_monitor_get_latest().
 * @return The state; TERRAIN_CALIB_CALIBRATING if @p p_data is NULL.
 * The data read functions only read their argument: any task may call
 * them on its own copy.
 */
terrain_calib_state_t
terrain_monitor_data_calib_state (terrain_data_t const * p_data);

/**
 * @brief Pitch in a snapshot.
 * @param p_data Snapshot.
 * @return Tenths of a degree, -512 to 511; nose up is positive. 0 if
 *         @p p_data is NULL.
 */
int16_t terrain_monitor_data_pitch_ddeg (terrain_data_t const * p_data);

/**
 * @brief Roll in a snapshot.
 * @param p_data Snapshot.
 * @return Tenths of a degree, -512 to 511; right side up is positive. 0 if
 *         @p p_data is NULL.
 */
int16_t terrain_monitor_data_roll_ddeg (terrain_data_t const * p_data);

/**
 * @brief Whether pitch is held in a snapshot (the car speeds up or brakes).
 * @param p_data Snapshot.
 * @return True if held; false if @p p_data is NULL.
 */
bool terrain_monitor_data_is_pitch_held (terrain_data_t const * p_data);

/**
 * @brief Whether roll is held in a snapshot (the car is turning).
 * @param p_data Snapshot.
 * @return True if held; false if @p p_data is NULL.
 */
bool terrain_monitor_data_is_roll_held (terrain_data_t const * p_data);

/**
 * @brief Acceleration along X (right) in a snapshot.
 * @param p_data Snapshot.
 * @return Milli-g, -8192 to 8188 in steps of 4; 0 if @p p_data is NULL.
 */
int16_t terrain_monitor_data_accel_x_mg (terrain_data_t const * p_data);

/**
 * @brief Acceleration along Y (forward) in a snapshot.
 * @param p_data Snapshot.
 * @return Milli-g, -8192 to 8188 in steps of 4; 0 if @p p_data is NULL.
 */
int16_t terrain_monitor_data_accel_y_mg (terrain_data_t const * p_data);

/**
 * @brief Acceleration along Z (up) in a snapshot.
 * @param p_data Snapshot.
 * @return Milli-g, -8192 to 8188 in steps of 4; 0 if @p p_data is NULL.
 */
int16_t terrain_monitor_data_accel_z_mg (terrain_data_t const * p_data);

/**
 * @brief When the snapshot's newest sample was read.
 * @param p_data Snapshot.
 * @return Time in 10 ms ticks, wrapping round after 655.35 s: use only
 *         16-bit unsigned differences. 0 if @p p_data is NULL.
 */
uint16_t terrain_monitor_data_time_10ms (terrain_data_t const * p_data);

/**
 * @brief Samples processed since init.
 * @param p_data Snapshot.
 * @return Count that wraps round at 65536: use only 16-bit unsigned
 *         differences. 0 if @p p_data is NULL.
 */
uint16_t terrain_monitor_data_sample_count (terrain_data_t const * p_data);

/**
 * @brief Times the sensor buffer overflowed.
 * @param p_data Snapshot.
 * @return 0 to 63; stops at 63. 0 if @p p_data is NULL.
 */
uint8_t terrain_monitor_data_overrun_count (terrain_data_t const * p_data);

/**
 * @brief Failed sensor reads since init.
 * @param p_data Snapshot.
 * @return 0 to 63; stops at 63. 0 if @p p_data is NULL.
 */
uint8_t terrain_monitor_data_error_count (terrain_data_t const * p_data);

/**
 * @brief Times the sensor was set up again after faults.
 * @param p_data Snapshot.
 * @return 0 to 15; stops at 15. 0 if @p p_data is NULL.
 */
uint8_t terrain_monitor_data_reset_count (terrain_data_t const * p_data);

/**
 * @brief Calibration state in calibration details.
 * @param p_info Details from terrain_monitor_get_calibration().
 * @return The state; TERRAIN_CALIB_CALIBRATING if @p p_info is NULL.
 * The calibration read functions only read their argument: any task may
 * call them on its own copy.
 */
terrain_calib_state_t
terrain_monitor_calib_state (terrain_calib_info_t const * p_info);

/**
 * @brief Attempts finished in this run of calibration.
 * @param p_info Calibration details.
 * @return 0 to 127; stops at 127. 0 if @p p_info is NULL.
 */
uint8_t terrain_monitor_calib_attempts (terrain_calib_info_t const * p_info);

/**
 * @brief Attempts rejected because the car moved.
 * @param p_info Calibration details.
 * @return 0 to 127; stops at 127. 0 if @p p_info is NULL.
 */
uint8_t terrain_monitor_calib_retries (terrain_calib_info_t const * p_info);

/**
 * @brief Last attempt's largest (max - min) on any axis.
 * @param p_info Calibration details.
 * @return Milli-g, 0 to 1020 in steps of 4, rounded up so it never reads
 *         better than it was; stops at 1020. 0 if @p p_info is NULL.
 */
uint16_t terrain_monitor_calib_spread_mg (terrain_calib_info_t const * p_info);

/**
 * @brief Last attempt's angle from the level reference.
 * @param p_info Calibration details.
 * @return Tenths of a degree, 0 to 255, rounded up so it never reads
 *         better than it was; stops at 25.5 degrees. 0 if @p p_info is NULL.
 */
uint16_t terrain_monitor_calib_tilt_ddeg (terrain_calib_info_t const * p_info);

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
