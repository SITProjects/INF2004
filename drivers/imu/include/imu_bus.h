/**
 * @file imu_bus.h
 * @brief I2C bus access used by the IMU driver.
 *
 * Uses fixed-width types only; uT-Kernel redefines size_t as a signed
 * type, so size_t must not appear in this interface.
 *
 * Implemented by imu_bus.c for the Pico SDK build. Once the
 * uT-Kernel (mtk3) build is set up, a uT-Kernel implementation will be
 * added and this header shared by both, so they offer the same functions.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef IMU_BUS_H
#define IMU_BUS_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Set up an I2C controller and its two pins.
 * @param unit I2C controller number: 0 or 1.
 * @param sda_pin GPIO number for SDA; must belong to @p unit.
 * @param scl_pin GPIO number for SCL; must belong to @p unit.
 * @param baud_hz Bus speed in Hz, for example 400000.
 * @param timeout_us Longest time any single transfer may take before it
 *        is abandoned and reported as a failure, in microseconds.
 * @return True on success; false for an invalid unit or pin pairing, or
 *         when the platform cannot use the requested pins.
 * Call once before any other imu_bus function.
 */
bool imu_bus_init (uint32_t unit, uint32_t sda_pin, uint32_t scl_pin,
                   uint32_t baud_hz, uint32_t timeout_us);

/**
 * @brief Check whether any device acknowledges a 7-bit address.
 * @param addr 7-bit I2C address.
 * @return True if a device answered.
 */
bool imu_bus_probe (uint8_t addr);

/**
 * @brief Write one byte to a device register.
 * @param addr 7-bit I2C address.
 * @param reg Register address.
 * @param value Byte to write.
 * @return True on success; false on no acknowledge or timeout.
 */
bool imu_bus_write_reg (uint8_t addr, uint8_t reg, uint8_t value);

/**
 * @brief Read consecutive bytes starting at a device register.
 * @param addr 7-bit I2C address.
 * @param reg First register address, sent unchanged. Any device-specific
 *        auto-increment flag must already be set by the caller.
 * @param p_data Destination buffer of at least @p len bytes.
 * @param len Number of bytes to read; must be at least 1.
 * @return True on success; false on no acknowledge, timeout, or bad
 *         arguments. @p p_data is unspecified after a failure.
 */
bool imu_bus_read_regs (uint8_t addr, uint8_t reg, uint8_t * p_data,
                        uint32_t len);

#endif /* IMU_BUS_H */
