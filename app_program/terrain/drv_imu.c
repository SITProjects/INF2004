/**
 * @file drv_imu.c
 * @brief LSM303DLHC (GY-511) accelerometer and magnetometer driver.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "drv_imu.h"
#include <stddef.h>
#include "drv_imu_bus.h"

/* Private types, constants, and macros. */

#define REG_CTRL_REG1_A 0x20u
#define REG_CTRL_REG4_A 0x23u
#define REG_CTRL_REG5_A 0x24u
#define REG_OUT_X_L_A 0x28u
#define REG_FIFO_CTRL_REG_A 0x2Eu
#define REG_FIFO_SRC_REG_A 0x2Fu

/* Bit 7 set: auto-increment the register address (section 5.1.1). */
#define AUTO_INCREMENT 0x80u

/* CTRL_REG1_A (Table 19): ODR in bits 7-4, Z/Y/X enable in bits 2-0. */
#define CTRL1_ODR_SHIFT 4u
#define CTRL1_XYZ_ENABLE 0x07u

/* CTRL_REG4_A (Table 27). */
#define CTRL4_BDU 0x80u
#define CTRL4_FS_SHIFT 4u
#define CTRL4_HR 0x08u

/* CTRL_REG5_A (Table 29). */
#define CTRL5_FIFO_EN 0x40u

/* FIFO_CTRL_REG_A (Table 38): mode in bits 7-6. */
#define FIFO_MODE_BYPASS 0x00u
#define FIFO_MODE_STREAM 0x80u

/* FIFO_SRC_REG_A (Table 39). */
#define FIFO_SRC_OVRN 0x40u
#define FIFO_SRC_EMPTY 0x20u
#define FIFO_SRC_FSS_MASK 0x1Fu

/* One sample is X, Y and Z, each 2 bytes, low byte first. */
#define SAMPLE_BYTES 6u
#define X_LOW_BYTE 0u
#define X_HIGH_BYTE 1u
#define Y_LOW_BYTE 2u
#define Y_HIGH_BYTE 3u
#define Z_LOW_BYTE 4u
#define Z_HIGH_BYTE 5u
#define BITS_PER_BYTE 8u

/*
 * Samples are left-justified 12-bit values. Divide rather than >> 4:
 * right-shifting a negative value is implementation-defined.
 */
#define LEFT_JUSTIFY_DIVISOR 16

/* Standard gravity, m/s^2 per g. */
#define STANDARD_GRAVITY_MPS2 9.80665f
#define MG_PER_G 1000.0f

#define RANGE_COUNT 4u
#define ODR_MAX ((uint32_t)IMU_ACCEL_ODR_400HZ)

/* Static data. */

static float const mg_per_count[RANGE_COUNT] = {1.0f, 2.0f, 4.0f, 12.0f};

static uint8_t accel_addr = 0u;
static float accel_scale_mps2 = 0.0f;
static bool b_accel_ready = false;
static bool b_accel_fifo = false;

/* Private function declarations. */
static imu_status_t write_and_verify (uint8_t reg, uint8_t value);
static imu_status_t read_sample (imu_accel_raw_t * p_raw);
static int16_t bytes_to_counts (uint8_t low, uint8_t high);

/* Public function definitions. */

imu_status_t
imu_accel_init (uint8_t addr, imu_accel_odr_t odr, imu_accel_range_t range,
                bool b_use_fifo)
{
    b_accel_ready = false;
    accel_scale_mps2 = 0.0f;

    if (((uint32_t)odr == 0u) || ((uint32_t)odr > ODR_MAX) ||
        ((uint32_t)range >= RANGE_COUNT))
    {
        return IMU_ERR_PARAM;
    }

    if (!imu_bus_probe(addr))
    {
        return IMU_ERR_NOT_FOUND;
    }
    accel_addr = addr;

    uint8_t ctrl1 =
        (uint8_t)(((uint32_t)odr << CTRL1_ODR_SHIFT) | CTRL1_XYZ_ENABLE);
    uint8_t ctrl4 =
        (uint8_t)(CTRL4_BDU | ((uint32_t)range << CTRL4_FS_SHIFT) | CTRL4_HR);
    uint8_t ctrl5 = b_use_fifo ? (uint8_t)CTRL5_FIFO_EN : 0u;

    imu_status_t status = write_and_verify(REG_CTRL_REG1_A, ctrl1);
    if (status == IMU_OK)
    {
        status = write_and_verify(REG_CTRL_REG4_A, ctrl4);
    }
    if (status == IMU_OK)
    {
        status = write_and_verify(REG_CTRL_REG5_A, ctrl5);
    }

    /* Bypass then stream empties the FIFO of samples from before init. */
    if (status == IMU_OK)
    {
        status = write_and_verify(REG_FIFO_CTRL_REG_A, FIFO_MODE_BYPASS);
    }
    if ((status == IMU_OK) && b_use_fifo)
    {
        status = write_and_verify(REG_FIFO_CTRL_REG_A, FIFO_MODE_STREAM);
    }

    if (status != IMU_OK)
    {
        return status;
    }

    accel_scale_mps2 = mg_per_count[range] / MG_PER_G * STANDARD_GRAVITY_MPS2;
    b_accel_fifo = b_use_fifo;
    b_accel_ready = true;
    return IMU_OK;
}

imu_status_t
imu_accel_read (imu_accel_raw_t * p_raw)
{
    if (!b_accel_ready)
    {
        return IMU_ERR_NOT_INIT;
    }
    if (p_raw == NULL)
    {
        return IMU_ERR_PARAM;
    }

    return read_sample(p_raw);
}

imu_status_t
imu_accel_read_fifo (imu_accel_raw_t * p_buf, uint32_t max_count,
                     uint32_t * p_count, bool * p_overrun)
{
    if (!b_accel_ready || !b_accel_fifo)
    {
        return IMU_ERR_NOT_INIT;
    }
    if ((p_buf == NULL) || (p_count == NULL) || (max_count == 0u))
    {
        return IMU_ERR_PARAM;
    }

    *p_count = 0u;
    if (p_overrun != NULL)
    {
        *p_overrun = false;
    }

    uint8_t src = 0u;
    if (!imu_bus_read_regs(accel_addr, REG_FIFO_SRC_REG_A, &src, 1u))
    {
        return IMU_ERR_BUS;
    }

    /*
     * The datasheet does not define FSS exactly; if it is one short, the
     * remaining sample is read on the next call.
     */
    uint32_t waiting = 0u;
    if ((src & FIFO_SRC_EMPTY) != 0u)
    {
        waiting = 0u;
    }
    else if ((src & FIFO_SRC_OVRN) != 0u)
    {
        waiting = IMU_ACCEL_FIFO_DEPTH;
        if (p_overrun != NULL)
        {
            *p_overrun = true;
        }
    }
    else
    {
        waiting = (uint32_t)(src & FIFO_SRC_FSS_MASK);
    }

    if (waiting > max_count)
    {
        waiting = max_count;
    }

    for (uint32_t i = 0u; i < waiting; i++)
    {
        imu_status_t status = read_sample(&p_buf[i]);
        if (status != IMU_OK)
        {
            return status;
        }
        *p_count = i + 1u;
    }

    return IMU_OK;
}

float
imu_accel_scale_mps2 (void)
{
    return accel_scale_mps2;
}

/* Private function definitions. */

/* Read back each setting; the accelerometer has no ID register. */
static imu_status_t
write_and_verify (uint8_t reg, uint8_t value)
{
    if (!imu_bus_write_reg(accel_addr, reg, value))
    {
        return IMU_ERR_BUS;
    }

    uint8_t readback = 0u;
    if (!imu_bus_read_regs(accel_addr, reg, &readback, 1u))
    {
        return IMU_ERR_BUS;
    }

    return (readback == value) ? IMU_OK : IMU_ERR_VERIFY;
}

static imu_status_t
read_sample (imu_accel_raw_t * p_raw)
{
    uint8_t bytes[SAMPLE_BYTES] = {0u};
    if (!imu_bus_read_regs(accel_addr,
                           (uint8_t)(REG_OUT_X_L_A | AUTO_INCREMENT), bytes,
                           SAMPLE_BYTES))
    {
        return IMU_ERR_BUS;
    }

    p_raw->x = bytes_to_counts(bytes[X_LOW_BYTE], bytes[X_HIGH_BYTE]);
    p_raw->y = bytes_to_counts(bytes[Y_LOW_BYTE], bytes[Y_HIGH_BYTE]);
    p_raw->z = bytes_to_counts(bytes[Z_LOW_BYTE], bytes[Z_HIGH_BYTE]);
    return IMU_OK;
}

static int16_t
bytes_to_counts (uint8_t low, uint8_t high)
{
    int16_t left_justified = (int16_t)(((uint16_t)high << BITS_PER_BYTE) | low);
    return (int16_t)(left_justified / LEFT_JUSTIFY_DIVISOR);
}
