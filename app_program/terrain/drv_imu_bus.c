/**
 * @file drv_imu_bus.c
 * @brief IMU I2C bus access through the kernel's I2C device driver.
 *
 * A register read is one combined transfer (TDN_I2C_EXEC): the register
 * address is written, then the data read after a repeated start. Every
 * transfer is bounded by the driver's timeout, so a stuck bus returns an
 * error instead of hanging the caller.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "drv_imu_bus.h"
#include <tk/tkernel.h>
#include "../../device/include/dev_i2c.h"

/* Private types, constants, and macros. */
#define IMU_BUS_DEVICE "iica" /* Kernel I2C driver, unit 0 (I2C0). */
#define WRITE_REG_BYTES 2

/* Static data. */
static ID bus_dd;
static bool b_bus_open = false;

/* Private function declarations. */

/* Public function definitions. */

bool
imu_bus_init (void)
{
    if (b_bus_open)
    {
        (void)tk_cls_dev(bus_dd, 0);
        b_bus_open = false;
    }

    ID dd = tk_opn_dev((UB *)IMU_BUS_DEVICE, TD_UPDATE);
    if (dd < E_OK)
    {
        return false;
    }
    bus_dd = dd;
    b_bus_open = true;
    return true;
}

bool
imu_bus_probe (uint8_t addr)
{
    if (!b_bus_open)
    {
        return false;
    }

    UB rx = 0u;
    SZ asize = 0;
    ER er = tk_srea_dev(bus_dd, (W)addr, &rx, 1, &asize);
    return (er >= E_OK) && (asize == 1);
}

bool
imu_bus_write_reg (uint8_t addr, uint8_t reg, uint8_t value)
{
    if (!b_bus_open)
    {
        return false;
    }

    UB tx[WRITE_REG_BYTES] = {reg, value};
    SZ asize = 0;
    ER er = tk_swri_dev(bus_dd, (W)addr, tx, WRITE_REG_BYTES, &asize);
    return (er >= E_OK) && (asize == WRITE_REG_BYTES);
}

bool
imu_bus_read_regs (uint8_t addr, uint8_t reg, uint8_t * p_data, uint32_t len)
{
    if (!b_bus_open || (p_data == NULL) || (len == 0u))
    {
        return false;
    }

    UB reg_addr = reg;
    T_I2C_EXEC exec;
    exec.sadr = addr;
    exec.snd_size = 1;
    exec.snd_data = &reg_addr;
    exec.rcv_size = (SZ)len;
    exec.rcv_data = p_data;

    SZ asize = 0;
    ER er = tk_swri_dev(bus_dd, TDN_I2C_EXEC, &exec, (SZ)sizeof(exec), &asize);
    return (er >= E_OK);
}

/* Private function definitions. */
