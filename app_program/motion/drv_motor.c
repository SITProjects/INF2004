/**
 * @file drv_motor.c
 * @brief Two-input H-bridge DC motor driver using hardware PWM.
 *
 * The board helpers in bsp/libbsp.h set the pin function, wrap, level and
 * enable; the clock divider has no helper, so it is written directly.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "drv_motor.h"
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

/* Private types, constants, and macros. */
#define MOTOR_PWM_FREQ_HZ 10000u
#define MOTOR_PWM_MAX_WRAP 65535u
#define MOTOR_PWM_MAX_DIVIDER 255u

/* Bank-0 GPIOs on the RP2040. */
#define MOTOR_GPIO_COUNT 30u

/* clk_sys as set by the kernel's start-up code; SYSCLK is in MHz. */
#define MOTOR_SYS_CLOCK_HZ ((uint32_t)SYSCLK * 1000000u)

/*
 * PWM slice registers: slice n starts at PWM_BASE + 0x14 * n. GPIO n
 * drives slice (n / 2) % 8. DIV is fixed point with 4 fraction bits.
 */
#define PWM_SLICE_STRIDE 0x14u
#define PWM_SLICE_COUNT 8u
#define PWM_DIV_FRAC_BITS 4u

/* RESETS: the PWM block's reset bit, and the atomic bit-clear alias. */
#define RESETS_PWM_BIT (1u << 14)
#define RESETS_DONE_OFFSET 0x8u
#define REG_ALIAS_CLR 0x3000u

typedef struct
{
    uint32_t pin_a;
    uint32_t pin_b;
    bool b_inverted;
    bool b_ready;
} motor_t;

/* Static data. */
static motor_t motors[MOTOR_COUNT];
static uint32_t pwm_divider;
static uint16_t pwm_wrap;
static uint16_t pwm_full; /* Level that holds the output high all period. */

/* Private function declarations. */
static bool compute_pwm_timing (void);
static void configure_pin (uint32_t pin);
static void set_levels (motor_t const * p_motor, uint16_t level_a,
                        uint16_t level_b);
static void release_pwm_reset (void);

/* Public function definitions. */

bool
motor_init (motor_id_t id, uint32_t pin_a, uint32_t pin_b, bool b_inverted)
{
    if ((id >= MOTOR_COUNT) || (pin_a >= MOTOR_GPIO_COUNT) ||
        (pin_b >= MOTOR_GPIO_COUNT) || (pin_a == pin_b) ||
        !compute_pwm_timing())
    {
        return false;
    }

    motor_t * p_motor = &motors[id];
    release_pwm_reset();
    configure_pin(pin_a);
    configure_pin(pin_b);
    p_motor->pin_a = pin_a;
    p_motor->pin_b = pin_b;
    p_motor->b_inverted = b_inverted;
    p_motor->b_ready = true;
    motor_brake(id);
    return true;
}

void
motor_set_duty (motor_id_t id, float duty)
{
    if ((id >= MOTOR_COUNT) || !motors[id].b_ready)
    {
        return;
    }

    motor_t const * p_motor = &motors[id];
    if (p_motor->b_inverted)
    {
        duty = -duty;
    }

    float magnitude = (duty < 0.0f) ? -duty : duty;
    if (magnitude > 1.0f)
    {
        magnitude = 1.0f;
    }

    /* PWM one input and hold the other low: drive with brake between. */
    uint16_t level = (uint16_t)(magnitude * (float)pwm_full);
    if (duty >= 0.0f)
    {
        set_levels(p_motor, level, 0u);
    }
    else
    {
        set_levels(p_motor, 0u, level);
    }
}

void
motor_brake (motor_id_t id)
{
    /* Robo Pico driver: both inputs low brakes, both high coasts. */
    if ((id < MOTOR_COUNT) && motors[id].b_ready)
    {
        set_levels(&motors[id], 0u, 0u);
    }
}

/* Private function definitions. */

/* Every motor slice runs at the same frequency, so shared slices agree. */
static bool
compute_pwm_timing (void)
{
    uint32_t counts_per_period = MOTOR_SYS_CLOCK_HZ / MOTOR_PWM_FREQ_HZ;
    uint32_t divider = (counts_per_period / (MOTOR_PWM_MAX_WRAP + 1u)) + 1u;
    if ((counts_per_period < 2u) || (divider > MOTOR_PWM_MAX_DIVIDER))
    {
        return false;
    }

    uint32_t wrap = (counts_per_period / divider) - 1u;
    pwm_divider = divider;
    pwm_wrap = (uint16_t)wrap;

    /* A level above wrap stays high; at the maximum wrap, use wrap. */
    pwm_full =
        (wrap < MOTOR_PWM_MAX_WRAP) ? (uint16_t)(wrap + 1u) : (uint16_t)wrap;
    return true;
}

static void
configure_pin (uint32_t pin)
{
    UW slice_base =
        PWM_BASE + (PWM_SLICE_STRIDE * ((pin / 2u) % PWM_SLICE_COUNT));

    /* As pwm_init() in the Pico SDK: default mode, counter stopped at 0. */
    out_w(slice_base + PWM_CHx_CSR, 0u);
    out_w(slice_base + PWM_CHx_DIV, pwm_divider << PWM_DIV_FRAC_BITS);
    out_w(slice_base + PWM_CHx_CTR, 0u);
    out_w(slice_base + PWM_CHx_CC, 0u);
    (void)pwm_set_wrap((UINT)pin, pwm_wrap);
    (void)pwm_set_cc((UINT)pin, 0u);
    (void)pwm_set_pin((UINT)pin);
    (void)pwm_set_enabled((UINT)pin, TRUE);
}

static void
set_levels (motor_t const * p_motor, uint16_t level_a, uint16_t level_b)
{
    (void)pwm_set_cc((UINT)p_motor->pin_a, level_a);
    (void)pwm_set_cc((UINT)p_motor->pin_b, level_b);
}

/* Take the PWM block out of reset if nothing has done so yet. */
static void
release_pwm_reset (void)
{
    if ((in_w(RESETS_RESET) & RESETS_PWM_BIT) != 0u)
    {
        out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_PWM_BIT);
        while ((in_w(RESETS_RESET + RESETS_DONE_OFFSET) & RESETS_PWM_BIT) == 0u)
        {
            /* Wait for the block to come out of reset. */
        }
    }
}
