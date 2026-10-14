/**
 * @file imu_bus_pico.c
 * @brief IMU I2C bus access through the Pico SDK.
 *
 * Implements imu_bus.h for the Pico SDK (CMake) build. Every transfer
 * uses the SDK's timeout variants, so a missing or stuck device returns
 * an error instead of hanging the caller.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "imu_bus.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

/* Private types, constants, and macros. */
#define IMU_BUS_UNIT_COUNT 2u

/* Static data. */

static i2c_inst_t * p_bus = NULL;
static uint32_t bus_timeout_us = 0u;

/* Private function declarations. */
static bool pin_matches (uint32_t pin, uint32_t unit, bool b_sda);

/* Public function definitions. */

bool
imu_bus_init (uint32_t unit, uint32_t sda_pin, uint32_t scl_pin,
              uint32_t baud_hz, uint32_t timeout_us)
{
    if ((unit >= IMU_BUS_UNIT_COUNT) || (baud_hz == 0u) || (timeout_us == 0u) ||
        !pin_matches(sda_pin, unit, true) || !pin_matches(scl_pin, unit, false))
    {
        return false;
    }

    i2c_inst_t * p_inst = (unit == 0u) ? i2c0 : i2c1;
    (void)i2c_init(p_inst, baud_hz);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    bus_timeout_us = timeout_us;
    p_bus = p_inst;
    return true;
}

bool
imu_bus_probe (uint8_t addr)
{
    if (p_bus == NULL)
    {
        return false;
    }

    uint8_t rx = 0u;
    int result =
        i2c_read_timeout_us(p_bus, addr, &rx, 1u, false, bus_timeout_us);
    return (result == 1);
}

bool
imu_bus_write_reg (uint8_t addr, uint8_t reg, uint8_t value)
{
    if (p_bus == NULL)
    {
        return false;
    }

    uint8_t tx[2] = {reg, value};
    int result = i2c_write_timeout_us(p_bus, addr, tx, sizeof(tx), false,
                                      bus_timeout_us);
    return (result == (int)sizeof(tx));
}

bool
imu_bus_read_regs (uint8_t addr, uint8_t reg, uint8_t * p_data, uint32_t len)
{
    if ((p_bus == NULL) || (p_data == NULL) || (len == 0u))
    {
        return false;
    }

    /* Repeated start (nostop = true) keeps the bus between the two steps. */
    int result =
        i2c_write_timeout_us(p_bus, addr, &reg, 1u, true, bus_timeout_us);
    if (result != 1)
    {
        return false;
    }

    result =
        i2c_read_timeout_us(p_bus, addr, p_data, len, false, bus_timeout_us);
    return (result == (int)len);
}

/* Private function definitions. */

/*
 * RP2040 I2C pin rule: GPIO n belongs to unit (n / 2) % 2, and is SDA
 * when n is even, SCL when n is odd.
 */
static bool
pin_matches (uint32_t pin, uint32_t unit, bool b_sda)
{
    if (pin >= NUM_BANK0_GPIOS)
    {
        return false;
    }

    bool b_is_sda = ((pin % 2u) == 0u);
    return ((((pin / 2u) % 2u) == unit) && (b_is_sda == b_sda));
}
