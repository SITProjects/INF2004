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
 * Current values: bench reading on a levelled surface (2026-10-16),
 * before the sensor was mounted on the car.
 */
#ifndef TERRAIN_LEVEL_REF_X
#define TERRAIN_LEVEL_REF_X (-0.125f)
#endif
#ifndef TERRAIN_LEVEL_REF_Y
#define TERRAIN_LEVEL_REF_Y (0.917f)
#endif
#ifndef TERRAIN_LEVEL_REF_Z
#define TERRAIN_LEVEL_REF_Z (8.247f)
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
