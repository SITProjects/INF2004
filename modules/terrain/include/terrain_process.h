/**
 * @file terrain_process.h
 * @brief IMU processing: axis mapping, calibration, tilt, motion state,
 *        impacts, bumps.
 *
 * Pure C with no Pico SDK or kernel calls, so the same file can run on
 * the car or be tested on a PC. Phase 1 adds axis mapping and Phase 2
 * calibration; detection algorithms follow from Phase 3.
 *
 * Internal to the terrain module; other modules should use terrain.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_PROCESS_H
#define TERRAIN_PROCESS_H

/* Public includes. */
#include <stdint.h>

/* Public types, constants, and macros. */

/* Number of axes in a 3-D vector. */
#define TERRAIN_AXES 3u

/* Standard gravity, m/s^2. A level, still car reads this on Z. */
#define TERRAIN_GRAVITY_MPS2 9.80665f

/** @brief A 3-D vector in the car's axes: X right, Y forward, Z up. */
typedef struct
{
    float x;
    float y;
    float z;
} terrain_vec3_t;

/**
 * @brief How the sensor board is mounted on the car.
 * For car axis i (0 = X, 1 = Y, 2 = Z), from[i] is the board axis it is
 * read from (0, 1 or 2) and sign[i] is 1 or -1.
 */
typedef struct
{
    uint8_t from[TERRAIN_AXES];
    int8_t sign[TERRAIN_AXES];
} terrain_axis_map_t;

/**
 * @brief Running totals for one calibration attempt.
 * Fixed size: samples are summed, not stored.
 */
typedef struct
{
    terrain_vec3_t sum; /**< Sum of all samples, m/s^2. */
    terrain_vec3_t min; /**< Smallest value seen on each axis. */
    terrain_vec3_t max; /**< Largest value seen on each axis. */
    uint32_t count;     /**< Samples added since the last reset. */
} terrain_calib_acc_t;

/** @brief Limits a calibration attempt must meet. */
typedef struct
{
    float max_spread_mps2;      /**< Largest (max - min) on any axis. */
    float max_gravity_err_mps2; /**< Largest size difference from ref. */
    float max_tilt_deg;         /**< Largest angle from the reference. */
} terrain_calib_limits_t;

/** @brief Outcome of a calibration attempt. */
typedef enum
{
    TERRAIN_CALIB_RESULT_OK = 0,    /**< Still and level; bias is usable. */
    TERRAIN_CALIB_RESULT_MOVED,     /**< Too much movement; try again. */
    TERRAIN_CALIB_RESULT_NOT_LEVEL, /**< Still, but tilted too far. */
    TERRAIN_CALIB_RESULT_INVALID    /**< NULL argument or no samples. */
} terrain_calib_result_t;

/** @brief Measurements from a calibration attempt. */
typedef struct
{
    terrain_vec3_t mean; /**< Average sample, m/s^2. */
    terrain_vec3_t bias; /**< Subtract from samples so level reads 0,0,g. */
    float spread_mps2;   /**< Largest (max - min) of any axis. */
    float tilt_deg;      /**< Angle between mean and level reference. */
} terrain_calib_out_t;

/* Public function declarations. */

/**
 * @brief Convert one sensor sample from board axes and counts to car
 *        axes and physical units, applying the chip's per-axis gains.
 * @param p_map Mounting; each from[] entry must be 0, 1 or 2.
 * @param gain Gain per BOARD axis: gain[0] = X, [1] = Y, [2] = Z.
 * @param board Sample in board axes: board[0] = X, [1] = Y, [2] = Z.
 * @param scale Units per count, e.g. m/s^2 per count.
 * @return The sample in car axes, multiplied by @p scale and the gain of
 *         its board axis. All zero if @p p_map, @p gain or @p board is
 *         NULL or a from[] entry is out of range.
 */
terrain_vec3_t terrain_process_map_axes (terrain_axis_map_t const * p_map,
                                         float const gain[TERRAIN_AXES],
                                         int16_t const board[TERRAIN_AXES],
                                         float scale);

/**
 * @brief Empty the running totals to start a new calibration attempt.
 * @param p_acc Totals to clear; ignored if NULL.
 */
void terrain_process_calib_reset (terrain_calib_acc_t * p_acc);

/**
 * @brief Add one sample to a calibration attempt.
 * @param p_acc Running totals.
 * @param p_sample Sample in car axes, m/s^2, gains applied.
 * Does nothing if either argument is NULL.
 */
void terrain_process_calib_add (terrain_calib_acc_t * p_acc,
                                terrain_vec3_t const * p_sample);

/**
 * @brief Judge a finished calibration attempt.
 * @param p_acc Running totals; must hold at least one sample.
 * @param p_level_ref Average reading of the car on level ground, m/s^2.
 * @param p_limits Limits the attempt must meet.
 * @param p_out Receives the measurements whatever the result, so they
 *        can be printed; all zero for TERRAIN_CALIB_RESULT_INVALID.
 * @return MOVED if any axis spread, or the difference between the size
 *         of the mean and of @p p_level_ref, is over its limit;
 *         otherwise NOT_LEVEL if the tilt is over its limit; otherwise
 *         OK. INVALID if an argument is NULL or there are no samples.
 */
terrain_calib_result_t terrain_process_calib_finish (
    terrain_calib_acc_t const * p_acc, terrain_vec3_t const * p_level_ref,
    terrain_calib_limits_t const * p_limits, terrain_calib_out_t * p_out);

/**
 * @brief Angle between two directions.
 * @param p_a First vector.
 * @param p_b Second vector.
 * @return Angle in degrees, 0 to 180. 0 if either argument is NULL or
 *         either vector is all zero.
 */
float terrain_process_angle_deg (terrain_vec3_t const * p_a,
                                 terrain_vec3_t const * p_b);

/**
 * @brief Remove the calibration bias from a sample.
 * @param p_sample Sample in car axes, m/s^2, gains applied.
 * @param p_bias Bias from a successful calibration.
 * @return @p p_sample minus @p p_bias; all zero if either is NULL.
 */
terrain_vec3_t terrain_process_correct (terrain_vec3_t const * p_sample,
                                        terrain_vec3_t const * p_bias);

#endif /* TERRAIN_PROCESS_H */
