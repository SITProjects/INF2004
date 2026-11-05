/**
 * @file terrain_process.h
 * @brief IMU processing: axis mapping, calibration, tilt, motion state,
 *        collisions, humps.
 *
 * Pure C with no Pico SDK or kernel calls, so the same file can run on
 * the car or be tested on a PC. Phase 1 adds axis mapping, Phase 2
 * calibration and Phase 3 tilt; detection algorithms follow in later
 * phases.
 *
 * Internal to the terrain module; other modules should use terrain_monitor.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_PROCESS_H
#define TERRAIN_PROCESS_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/* Number of axes in a 3-D vector. */
#define TERRAIN_AXES 3u

/* Standard gravity, m/s^2. A level, still car reads this on Z. */
#define TERRAIN_GRAVITY_MPS2 9.80665f

/*
 * Filter weight for a smoothing time constant tau_s and sample period
 * period_s, both positive. A macro so a constant weight can be worked out
 * when building; terrain_process_lowpass_alpha() uses the same formula.
 */
#define TERRAIN_PROCESS_LOWPASS_ALPHA(tau_s, period_s)                         \
    ((period_s) / ((tau_s) + (period_s)))

/* Widest field the encoders handle, bits. */
#define TERRAIN_FIELD_MAX_BITS 16u

/* terrain_tilt_t flags: masks and shifts, bit 0 first. Bits 4..31 spare. */
#define TERRAIN_TILT_PITCH_PRIMED 0x01u /**< Pitch filter has a sample. */
#define TERRAIN_TILT_ROLL_PRIMED 0x02u  /**< Roll filter has a sample. */
#define TERRAIN_TILT_PITCH_HELD 0x04u   /**< Pitch held by the last update. */
#define TERRAIN_TILT_ROLL_HELD 0x08u    /**< Roll held by the last update. */

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
    uint8_t count;      /**< Samples added since the last reset. */
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

/**
 * @brief First-order low-pass filter on a 3-D vector.
 * Each sample moves the value a fraction alpha of the way towards it.
 * Whether it has had a sample yet is kept by the owner (see
 * terrain_tilt_t), so the flags of several filters share one word.
 */
typedef struct
{
    terrain_vec3_t value; /**< Smoothed vector. */
} terrain_lowpass_t;

/**
 * @brief Tilt of the car, in degrees.
 * Pitch and roll have separate filters so that each can be held on its
 * own: speeding up or braking fakes pitch, turning fakes roll. Filters
 * and angles stay float: rounding them every sample would make the
 * filters drift.
 */
typedef struct
{
    terrain_lowpass_t pitch_filter; /**< Smoothed acceleration for pitch. */
    terrain_lowpass_t roll_filter;  /**< Smoothed acceleration for roll. */
    float pitch_deg;                /**< Nose up is positive. */
    float roll_deg;                 /**< Right side up is positive. */
    uint32_t flags;                 /**< TERRAIN_TILT_* bits. */
} terrain_tilt_t;

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
 * Holds at most UINT8_MAX samples; later ones are ignored. Does nothing
 * if either argument is NULL.
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

/**
 * @brief Filter weight for a smoothing time constant.
 * @param tau_s Time constant, s: the filter covers about two thirds of a
 *        step in this time and 95 % in three times it.
 * @param period_s Time between samples, s.
 * @return Weight for terrain_process_lowpass_step(), 0 to 1. 1 (no
 *         smoothing) if @p tau_s is not positive; 0 if @p period_s is not
 *         positive.
 */
float terrain_process_lowpass_alpha (float tau_s, float period_s);

/**
 * @brief Empty a filter; the next sample seeds it.
 * @param p_filter Filter to clear; ignored if NULL.
 */
void terrain_process_lowpass_reset (terrain_lowpass_t * p_filter);

/**
 * @brief Add one sample to a filter.
 * @param p_filter Filter to update.
 * @param p_sample New sample.
 * @param alpha Weight from terrain_process_lowpass_alpha().
 * @param b_first True for the first sample after a reset: it is taken as
 *        it is, so the output does not creep up from zero.
 * Does nothing if either pointer is NULL.
 */
void terrain_process_lowpass_step (terrain_lowpass_t * p_filter,
                                   terrain_vec3_t const * p_sample, float alpha,
                                   bool b_first);

/**
 * @brief Pitch from an acceleration in car axes.
 * @param p_accel Acceleration, m/s^2, with only gravity acting.
 * @return Degrees, -90 to 90; nose up is positive. Unaffected by roll.
 *         0 if @p p_accel is NULL.
 */
float terrain_process_pitch_deg (terrain_vec3_t const * p_accel);

/**
 * @brief Roll from an acceleration in car axes.
 * @param p_accel Acceleration, m/s^2, with only gravity acting.
 * @return Degrees, -180 to 180; right side up is positive. Pairs with
 *         terrain_process_pitch_deg(). 0 if @p p_accel is NULL.
 */
float terrain_process_roll_deg (terrain_vec3_t const * p_accel);

/**
 * @brief Turn rate of the car's body from its two wheel speeds.
 * @param left_mm_s Left wheel speed, mm/s; + is forward.
 * @param right_mm_s Right wheel speed, mm/s; + is forward.
 * @param track_mm Track width: centre of one drive wheel to the other, mm.
 * @return Degrees per second, (right - left) / track in radians; turning
 *         left (right wheel faster) is positive, forward or reversing
 *         alike. Not rounded or clamped. 0 if @p track_mm is not positive.
 */
float terrain_process_turn_rate_dps (int16_t left_mm_s, int16_t right_mm_s,
                                     float track_mm);

/**
 * @brief Clear the tilt: angles 0, nothing held, filters empty.
 * @param p_tilt Tilt to clear; ignored if NULL.
 */
void terrain_process_tilt_reset (terrain_tilt_t * p_tilt);

/**
 * @brief Update pitch and roll from one corrected sample.
 * @param p_tilt Tilt to update.
 * @param p_accel Corrected sample in car axes, m/s^2.
 * @param alpha Weight from terrain_process_lowpass_alpha().
 * @param b_hold_pitch True while the car speeds up or brakes: pitch and
 *        its filter keep their values.
 * @param b_hold_roll True while the car turns: roll and its filter keep
 *        their values.
 * Pausing a held filter avoids a jump when the hold ends. Does nothing
 * if either pointer is NULL.
 */
void terrain_process_tilt_update (terrain_tilt_t * p_tilt,
                                  terrain_vec3_t const * p_accel, float alpha,
                                  bool b_hold_pitch, bool b_hold_roll);

/**
 * @brief Write a field into a packed word.
 * @param word Word to change.
 * @param shift Position of the field's lowest bit, 0 to 31.
 * @param width Field width in bits, 1 to 31; shift + width at most 32.
 * @param code Value to store; bits above @p width are dropped.
 * @return @p word with the field replaced; unchanged if @p shift or
 *         @p width is out of range.
 */
uint32_t terrain_process_field_put (uint32_t word, uint32_t shift,
                                    uint32_t width, uint32_t code);

/**
 * @brief Read a field from a packed word.
 * @param word Packed word.
 * @param shift Position of the field's lowest bit, 0 to 31.
 * @param width Field width in bits, 1 to 31; shift + width at most 32.
 * @return The field's code; 0 if @p shift or @p width is out of range.
 */
uint32_t terrain_process_field_get (uint32_t word, uint32_t shift,
                                    uint32_t width);

/**
 * @brief Code for a signed value in a field of @p width bits, stored as
 *        two's complement so that code 0 means 0.
 * @param value Value in its own unit, e.g. degrees.
 * @param steps_per_unit Steps per unit, e.g. 10 for tenths of a degree.
 * @param width Field width, 2 to TERRAIN_FIELD_MAX_BITS.
 * @return Rounded to the nearest step and clamped to
 *         -2^(width-1) .. 2^(width-1) - 1 steps, never wrapped. 0 for a
 *         NaN or a width out of range.
 */
uint32_t terrain_process_encode_signed (float value, float steps_per_unit,
                                        uint32_t width);

/**
 * @brief Value of a code made by terrain_process_encode_signed().
 * @param code Field code; bits above @p width are ignored.
 * @param width Field width, 2 to TERRAIN_FIELD_MAX_BITS.
 * @return The value in steps, e.g. tenths of a degree; 0 for a width out
 *         of range.
 */
int32_t terrain_process_decode_signed (uint32_t code, uint32_t width);

/**
 * @brief Code for a value that cannot be negative.
 * @param value Value in its own unit; negative values give 0.
 * @param steps_per_unit Steps per unit.
 * @param width Field width, 1 to TERRAIN_FIELD_MAX_BITS.
 * @param b_round_up True to round up, so the code never shows less than
 *        the value (for "how bad" values); false for the nearest step.
 * @return Clamped to 0 .. 2^width - 1 steps. 0 for a NaN or a width out
 *         of range.
 */
uint32_t terrain_process_encode_unsigned (float value, float steps_per_unit,
                                          uint32_t width, bool b_round_up);

/**
 * @brief Add to a counter that stops at its largest value.
 * @param count Current count.
 * @param add Amount to add.
 * @param width Counter width, 1 to 31.
 * @return min(count + add, 2^width - 1); @p count if @p width is out of
 *         range.
 */
uint32_t terrain_process_count_add_sat (uint32_t count, uint32_t add,
                                        uint32_t width);

/**
 * @brief Add to a counter that wraps round to 0, for counts used only as
 *        differences (compare them with 16-bit unsigned subtraction).
 * @param count Current count.
 * @param add Amount to add.
 * @param width Counter width, 1 to 31.
 * @return (count + add) modulo 2^width; @p count if @p width is out of
 *         range.
 */
uint32_t terrain_process_count_add_wrap (uint32_t count, uint32_t add,
                                         uint32_t width);

#endif /* TERRAIN_PROCESS_H */
