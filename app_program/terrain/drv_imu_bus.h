/**
 * @file drv_imu_bus.h
 * @brief I2C bus access used by the IMU driver.
 *
 * Goes through the kernel's I2C device driver ("iica", I2C0). The kernel
 * fixes the bus settings: 400 kHz, SDA on GP4 and SCL on GP5
 * (device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h), and the transfer
 * timeout (DEVCNF_I2C0_TMO, in ms). config/config_device.h switches the
 * driver on. Only one task may use the bus (TEAM.md section 3).
 *
 * Uses fixed-width types only; uT-Kernel redefines size_t as a signed
 * type, so size_t must not appear in this interface.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef DRV_IMU_BUS_H
#define DRV_IMU_BUS_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Open the kernel's I2C device, closing it first if already open.
 * @return True on success; false if the device cannot be opened, for
 *         example when DEVCNF_USE_IIC is 0.
 * Call before any other imu_bus function. Calling it again re-initialises
 * the I2C controller, which can clear a stuck bus.
 */
bool imu_bus_init (void);

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

#endif /* DRV_IMU_BUS_H */
