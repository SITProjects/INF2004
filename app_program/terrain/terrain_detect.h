/**
 * @file terrain_detect.h
 * @brief Terrain detectors: the hump, turning and speed state machines,
 *        and the lift and overturn check.
 *
 * Pure C with no kernel calls, so it can be tested on a PC. Each step
 * takes its machine's word and one sample and returns the event it found,
 * if any. Waits are counted in samples, so no clock is needed.
 *
 * The course's humps are ramps under one drive wheel: the car leans
 * sideways while that wheel is up, and its nose rises only half as much.
 * So the machine follows the lean, the size of the roll: either wheel
 * counts the same, and the lean never goes negative, so coming down is a
 * drop from the highest lean of the hump (its peak), not a dip.
 *
 * Detector word, masks and shifts, bit 0 first; all zero is the resting
 * state (LEVEL, STRAIGHT, counters 0, no peak):
 *   hump state 2 | climb counter 4 | other counter 4 | peak lean 8 |
 *   turning state 1 | turn counter 5 | spare 8
 * The peak lean is in tenths of a degree, 0 to 25.5 degrees (stops
 * there): about a 46 mm lift of one wheel on the car's track, far above
 * any ramp it can climb. The turn counter counts samples, 0 to 31. Each
 * step changes only its own machine's fields; the spare bits are kept as
 * they are.
 *
 * Turning is found from the turn rate (from motion's wheel speeds), not
 * the accelerometer: see terrain_detect_turn_step().
 *
 * Lifting and overturning are found from the mean of the last 100 ms of
 * samples against the level reading from calibration: a lift pushes up
 * harder than driving does, and an overturned car reads gravity from the
 * side or below; see terrain_detect_lift_step().
 *
 * The speed state is found from the wheel speed (from motion's wheel
 * speeds), not the accelerometer: see terrain_detect_speed_step().
 *
 * Speed word, masks and shifts, bit 0 first; all zero is STATIONARY with
 * nothing counted:
 *   speed state 2 | window count 4 | candidate 2 | confirm count 2 |
 *   speed at the last check 8 | spare 14
 * The window count is in samples (0 to 15); the confirm count in checks
 * (0 to 3); the speed in 2 mm/s steps, 0 to 510 mm/s (stops there; motion
 * drives at most 500 mm/s).
 *
 * Internal to the terrain module; other modules should use
 * terrain_monitor.h.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_DETECT_H
#define TERRAIN_DETECT_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "terrain_monitor.h"
#include "terrain_process.h"

/* Public types, constants, and macros. */

/* Largest sample count the 4-bit hump counters hold. */
#define TERRAIN_DETECT_COUNT_MAX 15u

/** @brief Limits of the hump state machine, on the lean (size of roll). */
typedef struct
{
    float up_deg;          /**< Lean above this is climbing. */
    float drop_deg;        /**< Lean this far below the hump's peak is
                                coming down. */
    float level_deg;       /**< Lean below this is flat. */
    uint8_t enter_samples; /**< Samples in a row to start a climb or
                                descent, 1 to TERRAIN_DETECT_COUNT_MAX. */
    uint8_t top_samples;   /**< Samples in a row with no new peak to reach
                                the top, 1 to TERRAIN_DETECT_COUNT_MAX. */
    uint8_t level_samples; /**< Flat samples in a row to end a descent, 1 to
                                TERRAIN_DETECT_COUNT_MAX. */
} terrain_hump_limits_t;

/* Samples averaged by the lift check: 100 ms at 100 Hz. */
#define TERRAIN_DETECT_LIFT_WINDOW 10u

/** @brief Limits of the lift and overturn check. */
typedef struct
{
    float up_ratio;           /**< Upward acceleration, as a share of the
                                   level reading, above this is a lift. */
    float over_deg;           /**< Tilt from level above this ... */
    uint8_t over_samples;     /**< ... for this many samples in a row is
                                   overturned; 0 counts as 1. */
    uint8_t cooldown_samples; /**< Samples after an event before the next;
                                   0 counts as 1. */
} terrain_lift_limits_t;

/**
 * @brief Memory of the lift check; starts zeroed.
 *
 * The window keeps the last samples in cm/s^2, 16 bits per axis (the
 * sensor's +/-8 g needs 14). Word, bit 0 first: next slot 4 | samples
 * held 4 | over-tilt run 8 | cooldown 8 | overturn reported 1 | spare 7.
 */
typedef struct
{
    int16_t window[TERRAIN_DETECT_LIFT_WINDOW][3];
    uint32_t word;
} terrain_lift_t;

/* Largest sample count the 5-bit turn counter holds. */
#define TERRAIN_DETECT_TURN_COUNT_MAX 31u

/** @brief Limits of the turning state machine, on the size of the turn
 *         rate. */
typedef struct
{
    float on_dps;        /**< Faster than this is turning, deg/s. */
    float off_dps;       /**< Slower than this is straight, deg/s; below
                              on_dps. */
    uint8_t on_samples;  /**< Samples in a row to start a turn, 1 to
                              TERRAIN_DETECT_TURN_COUNT_MAX. */
    uint8_t off_samples; /**< Samples in a row to end a turn, 1 to
                              TERRAIN_DETECT_TURN_COUNT_MAX. */
} terrain_turn_limits_t;

/* Largest window the 4-bit window count holds, samples. */
#define TERRAIN_DETECT_WINDOW_MAX 15u
/* Largest confirm count the 2-bit field holds, checks. */
#define TERRAIN_DETECT_CONFIRM_MAX 3u

/** @brief Limits of the speed state machine. */
typedef struct
{
    float stop_mm_s;        /**< Slower than this is stopped, mm/s. */
    float change_mm_s;      /**< A change of at least this between two
                                checks is speeding up or slowing down,
                                mm/s. */
    uint8_t window;         /**< Samples between checks, 1 to
                                TERRAIN_DETECT_WINDOW_MAX. */
    uint8_t confirm_checks; /**< Checks in a row to change state, 1 to
                                 TERRAIN_DETECT_CONFIRM_MAX. */
    uint8_t stop_checks;    /**< As confirm_checks, for STATIONARY. */
} terrain_speed_limits_t;

/* Public function declarations. */

/**
 * @brief Run the hump state machine on one sample.
 * @param p_word Detector word; start from 0.
 * @param roll_deg Roll of this sample, degrees; either sign. Only its
 *        size, the lean, is used.
 * @param p_limits Limits; sample counts of 0 count as 1, and counts over
 *        TERRAIN_DETECT_COUNT_MAX can never be reached.
 * @return The event the sample caused: ASCENDING, DESCENDING, LEVEL or
 *         NONE. NONE, with nothing changed, if a pointer is NULL.
 *
 * LEVEL -> ASCENDING (event) after enter_samples above up_deg; the peak
 * starts there.
 * ASCENDING -> TOP (no event) after top_samples with no new peak.
 * TOP -> DESCENDING (event) after enter_samples at least drop_deg below
 * the peak.
 * DESCENDING -> LEVEL (event) after level_samples below level_deg; the
 * peak goes back to 0.
 * While ASCENDING or TOP, a higher lean raises the peak (in TOP it also
 * restarts the descent count). A counter restarts when its condition
 * breaks; both restart on a change of state.
 */
terrain_event_type_t
terrain_detect_hump_step (uint32_t * p_word, float roll_deg,
                          terrain_hump_limits_t const * p_limits);

/**
 * @brief Hump state in a detector word.
 * @param word Detector word.
 * @return The state.
 */
terrain_hump_state_t terrain_detect_hump_state (uint32_t word);

/**
 * @brief Restart the hump state machine: LEVEL, counters 0, no peak.
 * @param word Detector word.
 * @return @p word with only the hump fields cleared; the turning state
 *         and the spare bits are kept.
 */
uint32_t terrain_detect_hump_clear (uint32_t word);

/**
 * @brief Whether a hump was long enough to count.
 * @param start_mm Odometry distance as the climb began.
 * @param end_mm Odometry distance when level again.
 * @param min_mm TERRAIN_HUMP_MIN_MM.
 * @return True if it counts. Distances may wrap; reversing never counts.
 */
bool terrain_detect_hump_lasted (int32_t start_mm, int32_t end_mm,
                                 int32_t min_mm);

/**
 * @brief The travel a hump needs to count.
 * @param speed_mm_s Ground speed as the hump ended.
 * @param slow_mm_s TERRAIN_HUMP_SLOW_MM_S.
 * @param slow_min_mm TERRAIN_HUMP_SLOW_MIN_MM.
 * @param min_mm TERRAIN_HUMP_MIN_MM.
 * @return @p slow_min_mm for a car that slowed for it, else @p min_mm.
 */
int32_t terrain_detect_hump_min_mm (int32_t speed_mm_s, int32_t slow_mm_s,
                                    int32_t slow_min_mm, int32_t min_mm);

/** @brief What a sample means to the hump machine. */
typedef enum
{
    TERRAIN_HUMP_SAMPLE_USE,   /**< Step the machine. */
    TERRAIN_HUMP_SAMPLE_PAUSE, /**< Skip it, keeping any hump so far. */
    TERRAIN_HUMP_SAMPLE_DROP   /**< Drop any hump so far. */
} terrain_hump_sample_t;

/**
 * @brief Whether a sample can say anything about a hump.
 * @param b_following Following the line: moves, pivots and hands fake a lean.
 * @param b_roll_held The roll is an old value, not news.
 * @param speed_mm_s Ground speed; + is forward.
 * @param min_speed_mm_s TERRAIN_HUMP_MIN_SPEED_MM_S.
 * @return What to do with it. Too slow only pauses: the car brakes for a hump,
 *         almost to a stop, and that must not end the hump.
 */
terrain_hump_sample_t terrain_detect_hump_sample (bool b_following,
                                                  bool b_roll_held,
                                                  int32_t speed_mm_s,
                                                  int32_t min_speed_mm_s);

/**
 * @brief Run the turning state machine on one sample.
 * @param p_word Detector word; start from 0.
 * @param turn_dps Turn rate for this sample, deg/s; either sign. Only its
 *        size is used.
 * @param p_limits Limits; sample counts of 0 count as 1, and counts over
 *        TERRAIN_DETECT_TURN_COUNT_MAX can never be reached.
 * @return The event the sample caused: TURNING, STRAIGHT or NONE. NONE,
 *         with nothing changed, if a pointer is NULL.
 *
 * STRAIGHT -> TURNING (event) after on_samples above on_dps.
 * TURNING -> STRAIGHT (event) after off_samples below off_dps.
 * The counter restarts when its condition breaks and on a change of
 * state. A rate between the two limits keeps the state, so a slow part of
 * a turn does not end it. A NaN rate counts as neither.
 */
terrain_event_type_t
terrain_detect_turn_step (uint32_t * p_word, float turn_dps,
                          terrain_turn_limits_t const * p_limits);

/**
 * @brief Turning state in a detector word.
 * @param word Detector word.
 * @return The state.
 */
terrain_turn_state_t terrain_detect_turn_state (uint32_t word);

/**
 * @brief Run the speed state machine on one sample.
 * @param p_word Speed word; start from 0.
 * @param wheel_mm_s Wheel speed for this sample, mm/s: the average of the
 *        sizes of the two wheel speeds, so 0 or more (a negative value
 *        counts as 0). Use 0 when it is unknown.
 * @param p_limits Limits; counts of 0 count as 1, and counts over the
 *        field limits are cut to them.
 * @return The event the sample caused: STATIONARY, ACCELERATING,
 *         CRUISING, DECELERATING or NONE. NONE, with nothing changed, if
 *         a pointer is NULL.
 *
 * Every window samples the speed is checked against the speed at the
 * previous check, and gives one answer: STATIONARY below stop_mm_s;
 * otherwise ACCELERATING for a rise of at least change_mm_s,
 * DECELERATING for a fall of at least change_mm_s, CRUISING for less.
 * The state changes (event) once the same answer, different from the
 * state, comes confirm_checks checks in a row (stop_checks for
 * STATIONARY). An answer equal to the state, or a different answer,
 * restarts the count. A NaN speed counts as 0.
 */
terrain_event_type_t
terrain_detect_speed_step (uint32_t * p_word, float wheel_mm_s,
                           terrain_speed_limits_t const * p_limits);

/**
 * @brief Speed state in a speed word.
 * @param word Speed word.
 * @return The state.
 */
terrain_speed_state_t terrain_detect_speed_state (uint32_t word);

/**
 * @brief Run the lift and overturn check on one sample.
 * @param p_state State, zeroed to start; zero it again after a new
 *        calibration.
 * @param p_sample Sample in car axes, m/s^2, gains applied, before the
 *        bias.
 * @param p_level Level reading from calibration (the bias plus 0, 0, g),
 *        m/s^2.
 * @param p_limits Limits.
 * @param p_value Receives the event value: for LIFTED the upward
 *        acceleration in hundredths of the level reading, for OVERTURNED
 *        the tilt in degrees. Unchanged otherwise.
 * @return LIFTED, OVERTURNED or NONE. NONE, with nothing changed, if a
 *         pointer is NULL or the level reading is zero.
 *
 * Both use the mean of the last TERRAIN_DETECT_LIFT_WINDOW samples, and
 * nothing is decided until the window is full. LIFTED when the mean's part
 * along the level reading is more than up_ratio of it. OVERTURNED when the
 * angle between the mean and the level reading has been more than
 * over_deg for over_samples samples in a row; once, until the angle falls
 * back. After either, none for cooldown_samples samples.
 */
terrain_event_type_t terrain_detect_lift_step (
    terrain_lift_t * p_state, terrain_vec3_t const * p_sample,
    terrain_vec3_t const * p_level, terrain_lift_limits_t const * p_limits,
    uint32_t * p_value);

#endif /* TERRAIN_DETECT_H */
