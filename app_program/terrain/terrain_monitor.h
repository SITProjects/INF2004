/**
 * @file terrain_monitor.h
 * @brief IMU motion and terrain monitoring: public interface.
 *
 * Phase 1: initialise the GY-511 and read acceleration in the car's
 * axes. Phase 2: calibrate at every start, then correct every reading.
 * Phase 3: pitch and roll. Phase 4: hump state and an event queue. Phase
 * 5: turn rate and turning state, from motion's wheel speeds. Phase 7:
 * collisions. Phase 9: hump height. Motion state follows in a later
 * phase.
 *
 * Call terrain_monitor_init() once, then terrain_monitor_update()
 * regularly (about every 10 ms) from one place only. Calibration then skips
 * TERRAIN_CALIB_SETTLE_SAMPLES samples (0.5 s) and takes
 * TERRAIN_CALIB_SAMPLES samples (2 s) with the car standing still on
 * level ground; an attempt in which the car moved is repeated. Readings
 * are given out only once the calibration state is TERRAIN_CALIB_VALID.
 *
 * terrain_monitor_get_latest(), terrain_monitor_get_calibration(),
 * terrain_monitor_recalibrate(), terrain_monitor_poll_event() and
 * terrain_monitor_events_dropped() may be called from anywhere, including
 * other tasks.
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
 * @brief Where the car is on a hump, judged from the lean (size of roll):
 *        the course's humps are ramps under one drive wheel. Resting
 *        value 0.
 */
typedef enum
{
    TERRAIN_HUMP_LEVEL = 0, /**< Flat, no hump in progress. */
    TERRAIN_HUMP_ASCENDING, /**< Leaning more: a wheel is climbing. */
    TERRAIN_HUMP_TOP,       /**< On the top, not coming down yet. */
    TERRAIN_HUMP_DESCENDING /**< Leaning less: the wheel is coming down. */
} terrain_hump_state_t;

/**
 * @brief Whether the car is turning, judged from the turn rate (how fast
 *        its body rotates, from motion's wheel speeds). Resting value 0.
 */
typedef enum
{
    TERRAIN_TURN_STRAIGHT = 0, /**< Not rotating, or only slowly. */
    TERRAIN_TURN_TURNING       /**< Rotating faster than the turn limit. */
} terrain_turn_state_t;

/**
 * @brief How the wheels' speed is changing, judged from motion's two
 *        wheel speeds. Resting value 0. The order matches the event codes
 *        TERRAIN_EVENT_STATIONARY to TERRAIN_EVENT_DECELERATING.
 */
typedef enum
{
    TERRAIN_SPEED_STATIONARY = 0, /**< Wheels stopped, or nearly. */
    TERRAIN_SPEED_ACCELERATING,   /**< Wheels speeding up. */
    TERRAIN_SPEED_CRUISING,       /**< Wheels at a steady speed. */
    TERRAIN_SPEED_DECELERATING    /**< Wheels slowing down. */
} terrain_speed_state_t;

/**
 * @brief Kind of event. The codes are fixed, because events are passed to
 *        other subsystems; new kinds take the next free code.
 */
typedef enum
{
    TERRAIN_EVENT_NONE = 0,     /**< No event: the queue was empty. */
    TERRAIN_EVENT_ASCENDING,    /**< A climb started. */
    TERRAIN_EVENT_DESCENDING,   /**< A descent (or a dip) started. */
    TERRAIN_EVENT_LEVEL,        /**< Flat again after a descent. */
    TERRAIN_EVENT_COLLISION,    /**< The car hit something. */
    TERRAIN_EVENT_STATIONARY,   /**< The wheels stopped. */
    TERRAIN_EVENT_ACCELERATING, /**< The wheels started speeding up. */
    TERRAIN_EVENT_CRUISING,     /**< The wheels reached a steady speed. */
    TERRAIN_EVENT_DECELERATING, /**< The wheels started slowing down. */
    TERRAIN_EVENT_TURNING,      /**< The car started turning. */
    TERRAIN_EVENT_STRAIGHT      /**< The car stopped turning. */
} terrain_event_type_t;

/* Number of event codes in use; codes from here up to 15 are spare. */
#define TERRAIN_EVENT_TYPES 11u

/**
 * @brief One event, packed into one word: type 4 bits, value 12 bits,
 *        time 16 bits. Read it only through the terrain_monitor_event_*()
 *        functions; the layout is private. An all-zero event is NONE.
 */
typedef struct
{
    uint32_t word; /**< Packed type, value and time. */
} terrain_event_t;

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
 * The hump state follows the lean (size of roll) through LEVEL,
 * ASCENDING, TOP and DESCENDING; see terrain_monitor_poll_event() for the
 * events it sends. While roll is held, the hump state and its waits pause
 * too.
 *
 * The highest hump is the largest LEVEL event height since init, in mm
 * (see terrain_monitor_poll_event()). It is kept through a recalibration.
 *
 * The turn rate is how fast the car's body rotates, from motion's two
 * wheel speeds, in whole degrees per second from -256 to +255 (clamped);
 * turning left is positive. The turning state is TURNING while the car
 * rotates faster than TERRAIN_TURN_ON_DPS and STRAIGHT otherwise (see
 * terrain_monitor_poll_event()). Both come from the wheels, not the
 * accelerometer, so they are given in every calibration state. While
 * motion's odometry is missing or stale the turn rate reads 0.
 *
 * The speed state says whether the wheels are stopped, speeding up, at a
 * steady speed or slowing down (see terrain_monitor_poll_event()). It
 * also comes from the wheels, so it is given in every calibration state;
 * while motion's odometry is missing or stale the wheels count as
 * stopped.
 *
 * The acceleration, tilt and hump state fields are 0 (and nothing is
 * held) unless the calibration state is TERRAIN_CALIB_VALID. An all-zero
 * snapshot reads as CALIBRATING, LEVEL, STRAIGHT, STATIONARY, with
 * everything 0.
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
 * @brief Take the oldest event from the queue.
 * @param p_out Receives the event; read it with the
 *        terrain_monitor_event_*() functions.
 * @return TERRAIN_OK, also when the queue is empty: the event is then
 *         TERRAIN_EVENT_NONE. TERRAIN_ERR_NOT_INIT or TERRAIN_ERR_PARAM,
 *         with @p p_out unchanged.
 *
 * Never waits. A poll removes the event, so only one task should read
 * events. Calling it until it gives NONE empties the queue.
 *
 * Events, while the calibration state is VALID. The course's humps are
 * ramps under one drive wheel, so they are found from the lean, the size
 * of the roll; either wheel counts the same:
 * - ASCENDING: the lean stayed above TERRAIN_HUMP_UP_DEG for
 *   TERRAIN_HUMP_ENTER_SAMPLES samples. Value: lean then, tenths of a
 *   degree (0 or more).
 * - DESCENDING: on the top (no new peak for TERRAIN_HUMP_TOP_SAMPLES),
 *   the lean stayed at least TERRAIN_HUMP_DROP_DEG below the hump's
 *   highest lean for TERRAIN_HUMP_ENTER_SAMPLES samples. Value: lean
 *   then, tenths of a degree (0 or more).
 * - LEVEL: after a descent, the lean stayed below TERRAIN_HUMP_LEVEL_DEG
 *   for TERRAIN_HUMP_LEVEL_SAMPLES samples. Value: the hump's height, mm,
 *   0 to 255 (stops at 255).
 *
 * The height is TERRAIN_TRACK_WIDTH_MM x sin(lean), kept at its highest
 * from the start of the climb: how far the wheel on the ramp was lifted.
 *
 * While roll is held (the car is turning), no hump events are found and
 * the height pauses: the held value is not a new reading.
 *
 * - COLLISION: in hump state LEVEL only (never on a ramp), a horizontal
 *   push (forward/back and sideways) of at least
 *   TERRAIN_COLLISION_LIMIT_DG, after which motion's ground speed
 *   dropped to half or less within TERRAIN_COLLISION_CHECK_SAMPLES. It
 *   counts at once for a push of at least TERRAIN_COLLISION_SURE_DG, or
 *   when the speed is unknown or below TERRAIN_COLLISION_MIN_SPEED_MM_S.
 *   Then none for TERRAIN_COLLISION_COOLDOWN_SAMPLES. Value: the biggest
 *   push of the hit, tenths of a g, 0 to 127 (stops at 12.7 g).
 *
 * In every calibration state (the turn rate comes from motion's wheel
 * speeds, not the accelerometer):
 * - TURNING: the size of the turn rate stayed above TERRAIN_TURN_ON_DPS
 *   for TERRAIN_TURN_ON_SAMPLES samples, either way round. Value: 0.
 * - STRAIGHT: after TURNING, the size of the turn rate stayed below
 *   TERRAIN_TURN_OFF_DPS for TERRAIN_TURN_OFF_SAMPLES samples. Value: 0.
 * Turns in place and line-following curves faster than the limit count;
 * each separate turn of an obstacle detour counts as its own turn.
 *
 * Also in every calibration state, from the wheel speed: the average of
 * the sizes of motion's two wheel speeds, so a turn in place counts as
 * moving. Every TERRAIN_SPEED_WINDOW_SAMPLES samples it is compared with
 * the previous check. The state changes, with an event of the same name
 * (value 0), once the same answer comes TERRAIN_SPEED_CONFIRM_CHECKS
 * checks in a row (TERRAIN_SPEED_STOP_CHECKS for STATIONARY):
 * - STATIONARY: below TERRAIN_SPEED_STOP_MM_S (or no odometry).
 * - ACCELERATING: up by at least TERRAIN_SPEED_CHANGE_MM_S since the
 *   previous check.
 * - DECELERATING: down by at least TERRAIN_SPEED_CHANGE_MM_S.
 * - CRUISING: moving, and changed by less than that.
 *
 * The queue holds TERRAIN_EVENT_QUEUE_LEN events; when it is full the
 * oldest is dropped and counted (terrain_monitor_events_dropped()). A
 * recalibration resets the hump state to LEVEL and the collision check to
 * ready without an event, keeps the turning state, and keeps
 * the events already queued. Only terrain_monitor_init() empties it.
 * Safe to call from other tasks while terrain_monitor_update() runs.
 */
terrain_status_t terrain_monitor_poll_event (terrain_event_t * p_out);

/**
 * @brief Events dropped because the queue was full, since init.
 * @return 0 to 255; stops at 255. 0 before init.
 * Safe to call from other tasks.
 */
uint8_t terrain_monitor_events_dropped (void);

/**
 * @brief Kind of an event.
 * @param p_event Event from terrain_monitor_poll_event().
 * @return The kind; TERRAIN_EVENT_NONE if @p p_event is NULL.
 * The event read functions only read their argument: any task may call
 * them on its own copy.
 */
terrain_event_type_t
terrain_monitor_event_type (terrain_event_t const * p_event);

/**
 * @brief Value of an event, in the unit of its kind (see
 *        terrain_monitor_poll_event()).
 * @param p_event Event.
 * @return For ASCENDING and DESCENDING, tenths of a degree, -2048 to
 *         2047; for other kinds 0 to 4095. 0 for NONE or if @p p_event
 *         is NULL.
 */
int16_t terrain_monitor_event_value (terrain_event_t const * p_event);

/**
 * @brief When an event happened.
 * @param p_event Event.
 * @return Time in 10 ms ticks, on the same clock as
 *         terrain_monitor_data_time_10ms(), wrapping round after
 *         655.35 s: use only 16-bit unsigned differences. Events found
 *         in one update share its time. 0 if @p p_event is NULL.
 */
uint16_t terrain_monitor_event_time_10ms (terrain_event_t const * p_event);

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
 * @brief Hump state in a snapshot.
 * @param p_data Snapshot.
 * @return The state; TERRAIN_HUMP_LEVEL unless VALID, or if @p p_data is
 *         NULL.
 */
terrain_hump_state_t
terrain_monitor_data_hump_state (terrain_data_t const * p_data);

/**
 * @brief Highest hump since init in a snapshot.
 * @param p_data Snapshot.
 * @return mm, 0 to 255 (stops at 255); also kept while not VALID. 0 if
 *         @p p_data is NULL or before the first hump.
 */
uint8_t terrain_monitor_data_highest_hump_mm (terrain_data_t const * p_data);

/**
 * @brief Turn rate in a snapshot: how fast the car's body rotates.
 * @param p_data Snapshot.
 * @return Degrees per second, -256 to 255 (clamped), whole degrees;
 *         turning left is positive. Given in every calibration state; 0
 *         while motion's odometry is missing or stale, or if @p p_data is
 *         NULL.
 */
int16_t terrain_monitor_data_turn_rate_dps (terrain_data_t const * p_data);

/**
 * @brief Turning state in a snapshot.
 * @param p_data Snapshot.
 * @return The state, in every calibration state; TERRAIN_TURN_STRAIGHT if
 *         @p p_data is NULL.
 */
terrain_turn_state_t
terrain_monitor_data_turn_state (terrain_data_t const * p_data);

/**
 * @brief Speed state in a snapshot.
 * @param p_data Snapshot.
 * @return The state, in every calibration state;
 *         TERRAIN_SPEED_STATIONARY if @p p_data is NULL.
 */
terrain_speed_state_t
terrain_monitor_data_speed_state (terrain_data_t const * p_data);

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

/**
 * @brief Name of a hump state, for printing.
 * @param state Any terrain_hump_state_t value.
 * @return A constant string such as "ASCENDING"; never NULL.
 */
char const * terrain_monitor_hump_text (terrain_hump_state_t state);

/**
 * @brief Name of a turning state, for printing.
 * @param state Any terrain_turn_state_t value.
 * @return A constant string such as "TURNING"; never NULL.
 */
char const * terrain_monitor_turn_text (terrain_turn_state_t state);

/**
 * @brief Name of a speed state, for printing.
 * @param state Any terrain_speed_state_t value.
 * @return A constant string such as "CRUISING"; never NULL.
 */
char const * terrain_monitor_speed_text (terrain_speed_state_t state);

/**
 * @brief Name of an event kind, for printing.
 * @param type Any terrain_event_type_t value.
 * @return A constant string such as "LEVEL"; never NULL.
 */
char const * terrain_monitor_event_text (terrain_event_type_t type);

#endif /* TERRAIN_MONITOR_H */
