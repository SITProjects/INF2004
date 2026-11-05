/**
 * @file terrain_config.h
 * @brief Wiring and tunables for the terrain (IMU) module.
 *
 * Every value can be overridden before this header is included, for
 * example with a compiler -D flag, so the team can change tunables
 * without editing this file.
 *
 * The I2C wiring is not set here: the kernel's I2C driver owns the bus.
 * The GY-511 is on the Robo Pico's Grove port 3: I2C0 with SDA on GP4 and
 * SCL on GP5, at 400 kHz. Pins and the transfer timeout are in
 * device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h (DEVCNF_I2C0_*), and
 * config/config_device.h switches the driver on (DEVCNF_USE_IIC). Record
 * any change of pins in TEAM.md first.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_CONFIG_H
#define TERRAIN_CONFIG_H

/* Public includes. */
#include "drv_imu.h"

/* Public types, constants, and macros. */

/* 7-bit I2C addresses of the LSM303DLHC's two sensors. */
#ifndef TERRAIN_ACCEL_ADDR
#define TERRAIN_ACCEL_ADDR 0x19u
#endif

#ifndef TERRAIN_MAG_ADDR
#define TERRAIN_MAG_ADDR 0x1Eu
#endif

/*
 * Accelerometer data rate, as an imu_accel_odr_t value from drv_imu.h.
 * Matches the 100 Hz update rate.
 */
#ifndef TERRAIN_ACCEL_ODR
#define TERRAIN_ACCEL_ODR IMU_ACCEL_ODR_100HZ
#endif

/*
 * Accelerometer range, as an imu_accel_range_t value from drv_imu.h.
 * +/-8 g so collisions do not clip; costs resolution (4 mg per count).
 */
#ifndef TERRAIN_ACCEL_RANGE
#define TERRAIN_ACCEL_RANGE IMU_ACCEL_RANGE_8G
#endif

/*
 * 1: collect samples in the chip's 32-sample buffer, so none are lost
 *    when terrain_monitor_update() runs late.
 * 0: read only the newest sample each update (simpler; for debugging).
 */
#ifndef TERRAIN_ACCEL_USE_FIFO
#define TERRAIN_ACCEL_USE_FIFO 1
#endif

/*
 * Axis mapping: how the board is mounted on the car.
 *
 * Car axes: X = right, Y = forward, Z = up.
 * For each car axis, _FROM picks the board axis it comes from
 * (0 = board X, 1 = board Y, 2 = board Z) and _SIGN is 1 or -1.
 * Default: board axes already match the car.
 *
 * Example, board turned 90 degrees so its X points forward and its Y
 * points left: CAR_X_FROM 1 SIGN -1, CAR_Y_FROM 0 SIGN 1, CAR_Z_FROM 2
 * SIGN 1.
 */
#ifndef TERRAIN_CAR_X_FROM
#define TERRAIN_CAR_X_FROM 0
#endif
#ifndef TERRAIN_CAR_X_SIGN
#define TERRAIN_CAR_X_SIGN 1
#endif
#ifndef TERRAIN_CAR_Y_FROM
#define TERRAIN_CAR_Y_FROM 1
#endif
#ifndef TERRAIN_CAR_Y_SIGN
#define TERRAIN_CAR_Y_SIGN 1
#endif
#ifndef TERRAIN_CAR_Z_FROM
#define TERRAIN_CAR_Z_FROM 2
#endif
#ifndef TERRAIN_CAR_Z_SIGN
#define TERRAIN_CAR_Z_SIGN 1
#endif

#if (TERRAIN_CAR_X_FROM == TERRAIN_CAR_Y_FROM) ||                              \
    (TERRAIN_CAR_X_FROM == TERRAIN_CAR_Z_FROM) ||                              \
    (TERRAIN_CAR_Y_FROM == TERRAIN_CAR_Z_FROM)
#error "TERRAIN_CAR_*_FROM must use each board axis exactly once"
#endif
#if (TERRAIN_CAR_X_FROM < 0) || (TERRAIN_CAR_X_FROM > 2) ||                    \
    (TERRAIN_CAR_Y_FROM < 0) || (TERRAIN_CAR_Y_FROM > 2) ||                    \
    (TERRAIN_CAR_Z_FROM < 0) || (TERRAIN_CAR_Z_FROM > 2)
#error "TERRAIN_CAR_*_FROM must be 0, 1 or 2"
#endif
#if ((TERRAIN_CAR_X_SIGN != 1) && (TERRAIN_CAR_X_SIGN != -1)) ||               \
    ((TERRAIN_CAR_Y_SIGN != 1) && (TERRAIN_CAR_Y_SIGN != -1)) ||               \
    ((TERRAIN_CAR_Z_SIGN != 1) && (TERRAIN_CAR_Z_SIGN != -1))
#error "TERRAIN_CAR_*_SIGN must be 1 or -1"
#endif

/*
 * Accelerometer gain per BOARD axis, from the six-side test: lay the
 * board with each axis pointing up, then down, and use
 * gain = 2 x 9.80665 / (reading up - reading down).
 * Gains belong to the chip, not the mounting; redo the test only if the
 * GY-511 board is replaced.
 */
#ifndef TERRAIN_GAIN_BOARD_X
#define TERRAIN_GAIN_BOARD_X 0.981f
#endif
#ifndef TERRAIN_GAIN_BOARD_Y
#define TERRAIN_GAIN_BOARD_Y 0.977f
#endif
#ifndef TERRAIN_GAIN_BOARD_Z
#define TERRAIN_GAIN_BOARD_Z 0.995f
#endif

/*
 * Level reference: the average reading, in car axes and m/s^2 with gains
 * applied, of the car standing still on a floor checked to be flat.
 * Boot calibration compares against it to decide whether the car is
 * level. Record it again whenever the sensor is remounted.
 * Current values: bench reading with the loose sensor taped flat, chip
 * up, on a flat desk (TER-U06, 2026-11-03; 1.4 degrees from the
 * 2026-11-02 reading), before the sensor was mounted on the car.
 */
#ifndef TERRAIN_LEVEL_REF_X
#define TERRAIN_LEVEL_REF_X (-0.366f)
#endif
#ifndef TERRAIN_LEVEL_REF_Y
#define TERRAIN_LEVEL_REF_Y (0.922f)
#endif
#ifndef TERRAIN_LEVEL_REF_Z
#define TERRAIN_LEVEL_REF_Z (8.522f)
#endif

/* Samples in one calibration attempt: 200 is 2 s at 100 Hz. */
#ifndef TERRAIN_CALIB_SAMPLES
#define TERRAIN_CALIB_SAMPLES 200u
#endif

#if TERRAIN_CALIB_SAMPLES < 1
#error "TERRAIN_CALIB_SAMPLES must be at least 1"
#endif

/*
 * Samples skipped at the start of every calibration (power-up and each
 * recalibration), so a hand leaving the car or the car rocking on its
 * wheels is not averaged in: 50 is 0.5 s at 100 Hz. Not repeated
 * between the attempts of one calibration.
 */
#ifndef TERRAIN_CALIB_SETTLE_SAMPLES
#define TERRAIN_CALIB_SETTLE_SAMPLES 50u
#endif

/*
 * Stillness: largest allowed (max - min) on any axis during an attempt,
 * in m/s^2. Readings move in steps of about 0.16 m/s^2, so this allows
 * about four steps of noise.
 */
#ifndef TERRAIN_CALIB_MAX_SPREAD_MPS2
#define TERRAIN_CALIB_MAX_SPREAD_MPS2 0.6f
#endif

/*
 * Stillness: largest allowed difference between the size of the average
 * reading and the size of the level reference, in m/s^2. A bigger
 * difference means the car was being pushed or lifted.
 */
#ifndef TERRAIN_CALIB_MAX_GRAVITY_ERR_MPS2
#define TERRAIN_CALIB_MAX_GRAVITY_ERR_MPS2 0.5f
#endif

/*
 * Largest allowed angle, in degrees, between the measured gravity
 * direction and the level reference. Above it the car is not on level
 * ground and calibration stops with NOT_LEVEL.
 */
#ifndef TERRAIN_LEVEL_MAX_DEG
#define TERRAIN_LEVEL_MAX_DEG 2.0f
#endif

/*
 * Failed terrain_monitor_update() calls in a row before the I2C bus and sensor
 * are set up again. 10 is 100 ms at the 10 ms update rate.
 */
#ifndef TERRAIN_FAULT_RETRY_UPDATES
#define TERRAIN_FAULT_RETRY_UPDATES 10u
#endif

#if TERRAIN_FAULT_RETRY_UPDATES < 1
#error "TERRAIN_FAULT_RETRY_UPDATES must be at least 1"
#endif

/*
 * Time between accelerometer samples, in seconds. Must match
 * TERRAIN_ACCEL_ODR: 0.01 s at 100 Hz. Filters are set in seconds and
 * use this, so they keep their timing if the rate changes.
 */
#ifndef TERRAIN_SAMPLE_PERIOD_S
#define TERRAIN_SAMPLE_PERIOD_S 0.01f
#endif

/*
 * Tilt smoothing time constant, in seconds. After a change of angle,
 * pitch and roll cover about two thirds of it in this time and 95 % in
 * three times it. Larger is smoother but slower.
 */
#ifndef TERRAIN_TILT_TAU_S
#define TERRAIN_TILT_TAU_S 0.15f
#endif

/*
 * Hump state machine (terrain_detect.h), on the lean (size of the
 * smoothed roll) while calibrated. The course's humps are ramps under one
 * drive wheel, so the car leans sideways. A wheel lifted h mm leans the
 * car by asin(h / TERRAIN_TRACK_WIDTH_MM): on a 110.6 mm track, 1 mm is
 * about 0.52 degrees, and an 11 mm ramp about 5.7 degrees.
 *
 * Climb: lean above TERRAIN_HUMP_UP_DEG for TERRAIN_HUMP_ENTER_SAMPLES
 * samples in a row (5 = 50 ms, so a single jolt does not count). 2.1
 * degrees is about 4 mm: above the lean that line-following turns fake
 * (about 1 degree), well below an 11 mm ramp.
 */
#ifndef TERRAIN_HUMP_UP_DEG
#define TERRAIN_HUMP_UP_DEG 2.1f
#endif

/*
 * Descent: lean at least this far below the hump's peak for
 * TERRAIN_HUMP_ENTER_SAMPLES samples, degrees. 1.1 degrees is about
 * 2 mm, so wobbles on the top of a ramp do not count.
 */
#ifndef TERRAIN_HUMP_DROP_DEG
#define TERRAIN_HUMP_DROP_DEG 1.1f
#endif

/* Flat again: lean below this for TERRAIN_HUMP_LEVEL_SAMPLES (about
 * 2 mm). */
#ifndef TERRAIN_HUMP_LEVEL_DEG
#define TERRAIN_HUMP_LEVEL_DEG 1.1f
#endif

/* Samples in a row to start a climb or a descent: 5 = 50 ms. */
#ifndef TERRAIN_HUMP_ENTER_SAMPLES
#define TERRAIN_HUMP_ENTER_SAMPLES 5u
#endif

/* Samples in a row with no new peak to reach the top: 10 = 100 ms. */
#ifndef TERRAIN_HUMP_TOP_SAMPLES
#define TERRAIN_HUMP_TOP_SAMPLES 10u
#endif

/* Flat samples in a row to end a descent: 10 = 100 ms. */
#ifndef TERRAIN_HUMP_LEVEL_SAMPLES
#define TERRAIN_HUMP_LEVEL_SAMPLES 10u
#endif

/* The hump counters are 4 bits wide. */
#if (TERRAIN_HUMP_ENTER_SAMPLES < 1) || (TERRAIN_HUMP_ENTER_SAMPLES > 15)
#error "TERRAIN_HUMP_ENTER_SAMPLES must be 1 to 15"
#endif
#if (TERRAIN_HUMP_LEVEL_SAMPLES < 1) || (TERRAIN_HUMP_LEVEL_SAMPLES > 15)
#error "TERRAIN_HUMP_LEVEL_SAMPLES must be 1 to 15"
#endif
#if (TERRAIN_HUMP_TOP_SAMPLES < 1) || (TERRAIN_HUMP_TOP_SAMPLES > 15)
#error "TERRAIN_HUMP_TOP_SAMPLES must be 1 to 15"
#endif

/*
 * Hump height (terrain_height.h). Track width: centre of one drive wheel
 * to the centre of the other, in mm. The height is track width x
 * sin(lean), so a track width 10 % too wide reads every height 10 % too
 * high. 110.6 mm is motion's MOTION_TRACK_WIDTH_MM (same car), calibrated
 * for turns (MOT-U11/U12); the wheels measure 107.5 mm apart, so heights
 * read about 3 % high.
 */
#ifndef TERRAIN_TRACK_WIDTH_MM
#define TERRAIN_TRACK_WIDTH_MM 110.6f
#endif

/*
 * Roll smoothing for the hump height only, in seconds: lighter than
 * TERRAIN_TILT_TAU_S, whose 0.15 s can cut the short lean of a short ramp
 * and read it low. Lighter lets more motor vibration through, which can
 * read high. TODO(car day): tune over a measured ramp at race speed.
 */
#ifndef TERRAIN_HUMP_HEIGHT_TAU_S
#define TERRAIN_HUMP_HEIGHT_TAU_S 0.05f
#endif

/*
 * Collision detection (terrain_detect.h), from the horizontal push: the
 * forward/back and sideways acceleration, in tenths of a g (10 = 1 g,
 * the pull of gravity). Hard braking is about 0.3 g; leaning on an 11 mm
 * ramp about 0.1 g. A push of at least TERRAIN_COLLISION_LIMIT_DG may be
 * a collision: it counts if the car's speed (motion's odometry) then
 * drops to half or less within TERRAIN_COLLISION_CHECK_SAMPLES, so a
 * wheel jolting on a ramp edge, where the car keeps going, does not. A
 * push of at least TERRAIN_COLLISION_SURE_DG counts at once, for hits
 * where the wheels keep spinning and the speed never drops.
 * TODO(car day): set both from wall hits and ramp runs (TER-U21).
 */
#ifndef TERRAIN_COLLISION_LIMIT_DG
#define TERRAIN_COLLISION_LIMIT_DG 10u
#endif

#ifndef TERRAIN_COLLISION_SURE_DG
#define TERRAIN_COLLISION_SURE_DG 20u
#endif

/*
 * Samples to wait for the speed to halve: 12 = 120 ms, motion's 100 ms
 * speed average (5 x 20 ms updates) plus one update.
 */
#ifndef TERRAIN_COLLISION_CHECK_SAMPLES
#define TERRAIN_COLLISION_CHECK_SAMPLES 12u
#endif

/* Samples after a collision before the next: 50 = 500 ms, one hit gives
 * one event. */
#ifndef TERRAIN_COLLISION_COOLDOWN_SAMPLES
#define TERRAIN_COLLISION_COOLDOWN_SAMPLES 50u
#endif

/*
 * Below this speed, in mm/s, there is nothing to drop from (or the car
 * is standing and was hit): the push alone decides. Line following is
 * about 120 mm/s.
 */
#ifndef TERRAIN_COLLISION_MIN_SPEED_MM_S
#define TERRAIN_COLLISION_MIN_SPEED_MM_S 40u
#endif

/* The push field stops at 12.7 g; the countdowns are 4 and 6 bits. */
#if (TERRAIN_COLLISION_LIMIT_DG < 5) || (TERRAIN_COLLISION_LIMIT_DG > 80)
#error "TERRAIN_COLLISION_LIMIT_DG must be 5 to 80 (0.5 to 8 g)"
#endif
#if (TERRAIN_COLLISION_SURE_DG < TERRAIN_COLLISION_LIMIT_DG) ||                \
    (TERRAIN_COLLISION_SURE_DG > 127)
#error "TERRAIN_COLLISION_SURE_DG must be TERRAIN_COLLISION_LIMIT_DG to 127"
#endif
#if (TERRAIN_COLLISION_CHECK_SAMPLES < 1) ||                                   \
    (TERRAIN_COLLISION_CHECK_SAMPLES > 15)
#error "TERRAIN_COLLISION_CHECK_SAMPLES must be 1 to 15"
#endif
#if (TERRAIN_COLLISION_COOLDOWN_SAMPLES < 1) ||                                \
    (TERRAIN_COLLISION_COOLDOWN_SAMPLES > 63)
#error "TERRAIN_COLLISION_COOLDOWN_SAMPLES must be 1 to 63"
#endif
#if TERRAIN_COLLISION_MIN_SPEED_MM_S > 1000
#error "TERRAIN_COLLISION_MIN_SPEED_MM_S must be 0 to 1000"
#endif

/*
 * Turning state (terrain_detect.h), from the turn rate: how fast the
 * car's body rotates, worked out from motion's two wheel speeds, in
 * degrees per second. Either way round counts the same.
 *
 * TURNING once the turn rate stays above TERRAIN_TURN_ON_DPS for
 * TERRAIN_TURN_ON_SAMPLES samples (10 = 100 ms, so one steering twitch
 * does not count). Turns in place run at 90 deg/s (MOTION_TURN_SPEED_DEG_S)
 * and always count; a gentle line-following curve below 20 deg/s is just
 * following the line, not a turn.
 * STRAIGHT once it stays below TERRAIN_TURN_OFF_DPS for
 * TERRAIN_TURN_OFF_SAMPLES samples (30 = 300 ms). The gap between the two
 * limits stops the state flickering when the rate hovers near one of
 * them. TODO(car day): check the small line-following corrections stay
 * under the on limit (TER-U23).
 */
#ifndef TERRAIN_TURN_ON_DPS
#define TERRAIN_TURN_ON_DPS 20u
#endif

#ifndef TERRAIN_TURN_OFF_DPS
#define TERRAIN_TURN_OFF_DPS 10u
#endif

#ifndef TERRAIN_TURN_ON_SAMPLES
#define TERRAIN_TURN_ON_SAMPLES 10u
#endif

#ifndef TERRAIN_TURN_OFF_SAMPLES
#define TERRAIN_TURN_OFF_SAMPLES 30u
#endif

/* The turn rate field stops at 255 deg/s; the turn counter is 5 bits. */
#if (TERRAIN_TURN_ON_DPS < 1) || (TERRAIN_TURN_ON_DPS > 255)
#error "TERRAIN_TURN_ON_DPS must be 1 to 255"
#endif
#if (TERRAIN_TURN_OFF_DPS < 1) || (TERRAIN_TURN_OFF_DPS >= TERRAIN_TURN_ON_DPS)
#error "TERRAIN_TURN_OFF_DPS must be 1 to below TERRAIN_TURN_ON_DPS"
#endif
#if (TERRAIN_TURN_ON_SAMPLES < 1) || (TERRAIN_TURN_ON_SAMPLES > 31)
#error "TERRAIN_TURN_ON_SAMPLES must be 1 to 31"
#endif
#if (TERRAIN_TURN_OFF_SAMPLES < 1) || (TERRAIN_TURN_OFF_SAMPLES > 31)
#error "TERRAIN_TURN_OFF_SAMPLES must be 1 to 31"
#endif

/*
 * Motion's odometry (TEAM.md IF-05) counts as stale, and its distance as
 * unknown, once its update count has not changed for this long, in ms.
 * tMotion publishes every 20 ms, so 2 s means it has stopped, not that
 * it is late. At most 5000: the hint keeps only the count's low byte,
 * which at 20 ms wraps every 5.12 s.
 */
#ifndef TERRAIN_HINT_STALE_MS
#define TERRAIN_HINT_STALE_MS 2000u
#endif

#if (TERRAIN_HINT_STALE_MS < 100) || (TERRAIN_HINT_STALE_MS > 5000)
#error "TERRAIN_HINT_STALE_MS must be 100 to 5000"
#endif

/*
 * Bench debugging: the terrain task prints pitch and roll every this
 * many milliseconds once calibrated. 0 turns it off; keep 0 in commits
 * so the shared console stays quiet.
 */
#ifndef TERRAIN_DEBUG_PRINT_MS
#define TERRAIN_DEBUG_PRINT_MS 0
#endif

/*
 * Bench tests: 1 adds TER-U06 (record the level reference) and TER-U07
 * (check the axes after mounting) to make TEST=1. They need a person to
 * move the car on cue and take about 40 s. Keep 0 in commits: the whole
 * team's TEST=1 run must not wait for anyone.
 */
#ifndef TERRAIN_TEST_BENCH
#define TERRAIN_TEST_BENCH 0
#endif

/* Public function declarations. */

#endif /* TERRAIN_CONFIG_H */
