/**
 * @file imu.h
 * @brief LSM303DLHC (GY-511) accelerometer and magnetometer driver.
 *
 * Talks to the chip only through imu_bus.h, so it has no Pico SDK or
 * kernel dependencies. Phase 1 covers the accelerometer; the
 * magnetometer follows in the last phase.
 *
 * Register values come from the ST LSM303DLHC datasheet, Doc ID 018771
 * Rev 1 (April 2011); table numbers below refer to that document.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef IMU_H
#define IMU_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/* Samples the accelerometer FIFO can hold. */
#define IMU_ACCEL_FIFO_DEPTH 32u

/** @brief Result of a driver call. */
typedef enum
{
    IMU_OK = 0,
    IMU_ERR_PARAM,     /**< Invalid argument. */
    IMU_ERR_NOT_INIT,  /**< imu_accel_init() has not succeeded yet. */
    IMU_ERR_NOT_FOUND, /**< No device answered at the address. */
    IMU_ERR_BUS,       /**< A transfer failed or timed out. */
    IMU_ERR_VERIFY     /**< A setting did not read back as written. */
} imu_status_t;

/** @brief Accelerometer output data rate (datasheet Table 20). */
typedef enum
{
    IMU_ACCEL_ODR_1HZ = 1,
    IMU_ACCEL_ODR_10HZ = 2,
    IMU_ACCEL_ODR_25HZ = 3,
    IMU_ACCEL_ODR_50HZ = 4,
    IMU_ACCEL_ODR_100HZ = 5,
    IMU_ACCEL_ODR_200HZ = 6,
    IMU_ACCEL_ODR_400HZ = 7
} imu_accel_odr_t;

/** @brief Accelerometer full-scale range (datasheet Table 27). */
typedef enum
{
    IMU_ACCEL_RANGE_2G = 0,
    IMU_ACCEL_RANGE_4G = 1,
    IMU_ACCEL_RANGE_8G = 2,
    IMU_ACCEL_RANGE_16G = 3
} imu_accel_range_t;

/**
 * @brief One accelerometer sample in the board's own axes.
 * Counts are 12-bit signed values; multiply by imu_accel_scale_mps2()
 * to get m/s^2.
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_raw_t;

/* Public function declarations. */

/**
 * @brief Configure the accelerometer: data rate, all three axes,
 *        high-resolution mode, block data update, and the range.
 * @param addr 7-bit I2C address of the accelerometer (0x19 on GY-511).
 * @param odr Output data rate.
 * @param range Full-scale range.
 * @param b_use_fifo True to collect samples in the chip's 32-sample
 *        buffer (stream mode); false to read only the newest sample.
 * @return IMU_OK; IMU_ERR_NOT_FOUND if nothing answers at @p addr;
 *         IMU_ERR_VERIFY if a setting reads back wrong; IMU_ERR_BUS or
 *         IMU_ERR_PARAM otherwise.
 * imu_bus_init() must have succeeded first. The chip has no ID
 * register for the accelerometer, so presence is confirmed by writing
 * the settings and reading them back.
 */
imu_status_t imu_accel_init (uint8_t addr, imu_accel_odr_t odr,
                             imu_accel_range_t range, bool b_use_fifo);

/**
 * @brief Read the newest accelerometer sample.
 * @param p_raw Receives the sample.
 * @return IMU_OK, IMU_ERR_NOT_INIT, IMU_ERR_PARAM or IMU_ERR_BUS.
 * In FIFO mode this removes one sample from the buffer; use
 * imu_accel_read_fifo() instead.
 */
imu_status_t imu_accel_read (imu_accel_raw_t * p_raw);

/**
 * @brief Read every sample waiting in the chip's buffer, oldest first.
 * @param p_buf Receives up to @p max_count samples.
 * @param max_count Capacity of @p p_buf; IMU_ACCEL_FIFO_DEPTH is enough.
 * @param p_count Receives the number of samples stored in @p p_buf.
 * @param p_overrun Receives true if the buffer had filled up, meaning
 *        older samples were lost before this call. May be NULL.
 * @return IMU_OK, IMU_ERR_NOT_INIT, IMU_ERR_PARAM or IMU_ERR_BUS.
 * Only valid when imu_accel_init() was called with b_use_fifo true.
 */
imu_status_t imu_accel_read_fifo (imu_accel_raw_t * p_buf, uint32_t max_count,
                                  uint32_t * p_count, bool * p_overrun);

/**
 * @brief Conversion factor for the configured range.
 * @return m/s^2 per count, or 0 before imu_accel_init() succeeds.
 */
float imu_accel_scale_mps2 (void);

#endif /* IMU_H */
