/**
 * @file terrain_config.h
 * @brief Wiring and tunables for the terrain (IMU) module.
 *
 * Every value can be overridden before this header is included, for
 * example with a compiler -D flag or target_compile_definitions(), so
 * the team can change wiring without editing this file.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef TERRAIN_CONFIG_H
#define TERRAIN_CONFIG_H

/* Public includes. */
#include "imu.h"

/* Public types, constants, and macros. */

/*
 * Robo Pico Grove port the GY-511 is plugged into:
 *   1 = GP0/GP1   (I2C0) - also the uT-Kernel UART console pins
 *   2 = GP2/GP3   (I2C1) - shared with the Maker port; currently used by
 *                          the ultrasonic sensor (app/main.h)
 *   3 = GP4/GP5   (I2C0) - default
 *   4 = GP16/GP17 (I2C0) - GP16 is the uT-Kernel liveness LED by default
 *   0 = custom: define TERRAIN_I2C_UNIT, TERRAIN_I2C_SDA_PIN and
 *       TERRAIN_I2C_SCL_PIN yourself
 */
#ifndef TERRAIN_GROVE_PORT
#define TERRAIN_GROVE_PORT 3
#endif

#if TERRAIN_GROVE_PORT == 1
#define TERRAIN_I2C_UNIT 0u
#define TERRAIN_I2C_SDA_PIN 0u
#define TERRAIN_I2C_SCL_PIN 1u
#elif TERRAIN_GROVE_PORT == 2
#define TERRAIN_I2C_UNIT 1u
#define TERRAIN_I2C_SDA_PIN 2u
#define TERRAIN_I2C_SCL_PIN 3u
#elif TERRAIN_GROVE_PORT == 3
#define TERRAIN_I2C_UNIT 0u
#define TERRAIN_I2C_SDA_PIN 4u
#define TERRAIN_I2C_SCL_PIN 5u
#elif TERRAIN_GROVE_PORT == 4
#define TERRAIN_I2C_UNIT 0u
#define TERRAIN_I2C_SDA_PIN 16u
#define TERRAIN_I2C_SCL_PIN 17u
#elif TERRAIN_GROVE_PORT == 0
#if !defined(TERRAIN_I2C_UNIT) || !defined(TERRAIN_I2C_SDA_PIN) ||             \
    !defined(TERRAIN_I2C_SCL_PIN)
#error "TERRAIN_GROVE_PORT 0 needs TERRAIN_I2C_UNIT, _SDA_PIN and _SCL_PIN"
#endif
#else
#error "TERRAIN_GROVE_PORT must be 0 (custom), 1, 2, 3 or 4"
#endif

/* I2C bus speed in Hz. The LSM303DLHC supports up to 400 kHz. */
#ifndef TERRAIN_I2C_BAUD_HZ
#define TERRAIN_I2C_BAUD_HZ 400000u
#endif

/*
 * Longest time one I2C transfer may take before it is abandoned, in
 * microseconds. A normal sensor read takes about 0.2 ms at 400 kHz.
 */
#ifndef TERRAIN_I2C_TIMEOUT_US
#define TERRAIN_I2C_TIMEOUT_US 2000u
#endif

/* 7-bit I2C addresses of the LSM303DLHC's two sensors. */
#ifndef TERRAIN_ACCEL_ADDR
#define TERRAIN_ACCEL_ADDR 0x19u
#endif

#ifndef TERRAIN_MAG_ADDR
#define TERRAIN_MAG_ADDR 0x1Eu
#endif

/*
 * Accelerometer data rate, as an imu_accel_odr_t value from imu.h.
 * Matches the 100 Hz update rate.
 */
#ifndef TERRAIN_ACCEL_ODR
#define TERRAIN_ACCEL_ODR IMU_ACCEL_ODR_100HZ
#endif

/*
 * Accelerometer range, as an imu_accel_range_t value from imu.h.
 * +/-8 g so collisions do not clip; costs resolution (4 mg per count).
 */
#ifndef TERRAIN_ACCEL_RANGE
#define TERRAIN_ACCEL_RANGE IMU_ACCEL_RANGE_8G
#endif

/*
 * 1: collect samples in the chip's 32-sample buffer, so none are lost
 *    when terrain_update() runs late.
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

/* Public function declarations. */

#endif /* TERRAIN_CONFIG_H */
