/**
 * @file terrain_detect.h
 * @brief Terrain detectors: the hump, collision, turning and speed state
 *        machines.
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
 * Collisions are found from the horizontal push (forward/back and
 * sideways): a wall shoves the car back or sideways, a wheel landing
 * mostly pushes up. A push that might be a jolt is confirmed by the
 * car's speed dropping; see terrain_detect_collision_step().
 *
 * Collision word, masks and shifts, bit 0 first; all zero is READY:
 *   cooldown 6 | check countdown 4 | speed at the push 8 |
 *   biggest push 7 | spare 7
 * Cooldown and countdown are in samples (0 to 63, 0 to 15); the speed in
 * 4 mm/s steps, 0 to 1020 mm/s (stops there); the push in 0.1 g steps, 0
 * to 12.7 g (stops there; two axes of +/-8 g give at most 11.3 g).
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

/* Largest check countdown and cooldown the collision word holds. */
#define TERRAIN_DETECT_CHECK_MAX 15u
#define TERRAIN_DETECT_COOLDOWN_MAX 63u

/* Largest push the collision word and event value hold, 0.1 g. */
#define TERRAIN_DETECT_PUSH_MAX_DG 127u

/** @brief Limits of the collision state machine. */
typedef struct
{
    float limit_g;            /**< Push at least this may be a collision:
                                   check the speed. */
    float sure_g;             /**< Push at least this is a collision at
                                   once; at or above limit_g. */
    float min_speed_mm_s;     /**< Below this speed there is nothing to
                                   drop from: the push alone decides. */
    uint8_t check_samples;    /**< Samples to wait for the speed to halve,
                                   1 to TERRAIN_DETECT_CHECK_MAX. */
    uint8_t cooldown_samples; /**< Samples after a collision before the
                                   next one, 1 to
                                   TERRAIN_DETECT_COOLDOWN_MAX. */
} terrain_collision_limits_t;

/** @brief One sample's input to the collision state machine. */
typedef struct
{
    float push_g;              /**< Horizontal push, g: the size of the
                                       forward/back and sideways parts of
                                       the corrected acceleration. */
    terrain_hump_state_t hump; /**< Hump state after this sample. */
    bool b_speed_known;        /**< speed_mm_s is valid. */
    int32_t speed_mm_s;        /**< Car speed, mm/s; either sign. */
    bool b_stopped_dead;       /**< terrain_detect_stopped_dead() said so. */
    float peak_g;              /**< Its recent biggest push, g, reported as
                                    the size of that hit. */
} terrain_collision_input_t;

/** @brief Limits of terrain_detect_stopped_dead(). */
typedef struct
{
    int32_t from_mm_s;       /**< Driving at this or faster can stop dead. */
    uint32_t window_samples; /**< Longest fall to half speed, samples. */
    uint32_t grace_samples;  /**< A lean this new may be the hit itself. */
} terrain_stopped_limits_t;

/** @brief Memory of terrain_detect_stopped_dead(); starts zeroed. */
typedef struct
{
    int32_t kept_mm_s;  /* Speed last driven at; 0 when none to fall from. */
    uint32_t age;       /* Samples since then. */
    uint32_t off_level; /* Samples since the hump state left LEVEL. */
    float peak_g;       /* Biggest recent push, fading, g. */
} terrain_stopped_t;

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
 * @brief The push to judge: none unless driving forward and not lifted.
 * @param push_g Horizontal push, in g.
 * @param speed_mm_s Ground speed; + is forward.
 * @param lean_deg Size of the roll, in degrees.
 * @param driving_mm_s TERRAIN_COLLISION_DRIVING_MM_S.
 * @param max_lean_deg TERRAIN_COLLISION_MAX_LEAN_DEG.
 * @return @p push_g, or 0.
 * @note A hit under way still counts as the car slows through the limit: its
 *       check needs only the speed.
 */
float terrain_detect_collision_push (float push_g, int32_t speed_mm_s,
                                     float lean_deg, int32_t driving_mm_s,
                                     float max_lean_deg);

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
 * @brief Run the collision state machine on one sample.
 * @param p_word Collision word; start from 0.
 * @param p_in This sample's push, hump state and speed.
 * @param p_limits Limits; sample counts of 0 count as 1, and counts over
 *        the field limits are cut to them.
 * @param p_value Receives the event value when COLLISION is returned:
 *        the biggest push of the collision, 0.1 g, 0 to
 *        TERRAIN_DETECT_PUSH_MAX_DG. Unchanged otherwise.
 * @return COLLISION or NONE. NONE, with nothing changed, if a pointer is
 *         NULL.
 *
 * Three states, read from the two countdowns:
 * READY -> COLLISION at once (to COOLDOWN) for a push of at least sure_g,
 * or at least limit_g when the speed is unknown or below min_speed_mm_s.
 * READY -> CHECKING (no event) for a push of at least limit_g otherwise;
 * the speed then is kept.
 * CHECKING -> COLLISION (to COOLDOWN) once the speed is at most half the
 * kept speed, the speed becomes unknown, or a push reaches sure_g, within
 * check_samples samples; the biggest push so far is the value.
 * CHECKING -> READY (no event) when check_samples pass first (a jolt: the
 * car kept going), or the hump state leaves LEVEL.
 * COOLDOWN -> READY after cooldown_samples samples: one hit, one event.
 * Pushes count only in hump state LEVEL, so ramps never do.
 */
terrain_event_type_t terrain_detect_collision_step (
    uint32_t * p_word, terrain_collision_input_t const * p_in,
    terrain_collision_limits_t const * p_limits, uint32_t * p_value);

/**
 * @brief Whether the wheels have just fallen to half speed with nothing asking
 *        them to: the mark of a hit at line-following speed.
 *
 * At 120 mm/s a hit is a small push, inside what driving and the hump give, so
 * the push cannot tell it. Everything that slows the car on purpose either
 * leaves following (a stop, a pivot) or starts with the hump state leaving
 * LEVEL and takes longer than the grace to halve the speed.
 *
 * @param p_state State, zeroed to start. Call once per sample.
 * @param speed_mm_s Car speed, mean of both wheels, mm/s.
 * @param push_g This sample's horizontal push, g.
 * @param b_driving Following the line, with speed_mm_s valid.
 * @param hump Hump state after this sample.
 * @param p_limits Limits.
 * @return True once for each fall.
 */
bool terrain_detect_stopped_dead (terrain_stopped_t * p_state,
                                  int32_t speed_mm_s, float push_g,
                                  bool b_driving, terrain_hump_state_t hump,
                                  terrain_stopped_limits_t const * p_limits);

#endif /* TERRAIN_DETECT_H */
