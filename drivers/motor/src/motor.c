/**
 * @file motor.c
 * @brief Two-input H-bridge DC motor driver using hardware PWM.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "motor.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* Private types, constants, and macros. */
#define MOTOR_PWM_FREQ_HZ 10000u
#define MOTOR_PWM_MAX_WRAP 65535u
#define MOTOR_PWM_MAX_DIVIDER 255u

typedef struct
{
    uint32_t slice_a;
    uint32_t channel_a;
    uint32_t slice_b;
    uint32_t channel_b;
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
static void configure_pin (uint32_t pin, uint32_t * p_slice,
                           uint32_t * p_channel);
static void set_levels (motor_t const * p_motor, uint16_t level_a,
                        uint16_t level_b);

/* Public function definitions. */

bool
motor_init (motor_id_t id, uint32_t pin_a, uint32_t pin_b, bool b_inverted)
{
    if ((id >= MOTOR_COUNT) || (pin_a >= NUM_BANK0_GPIOS) ||
        (pin_b >= NUM_BANK0_GPIOS) || (pin_a == pin_b) || !compute_pwm_timing())
    {
        return false;
    }

    motor_t * p_motor = &motors[id];
    configure_pin(pin_a, &p_motor->slice_a, &p_motor->channel_a);
    configure_pin(pin_b, &p_motor->slice_b, &p_motor->channel_b);
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

    /* PWM one input and hold the other low: drive with coast between. */
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
    /* Both inputs high shorts the motor through the H-bridge. */
    if ((id < MOTOR_COUNT) && motors[id].b_ready)
    {
        set_levels(&motors[id], pwm_full, pwm_full);
    }
}

/* Private function definitions. */

/* Every motor slice runs at the same frequency, so shared slices agree. */
static bool
compute_pwm_timing (void)
{
    uint32_t counts_per_period = clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ;
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
configure_pin (uint32_t pin, uint32_t * p_slice, uint32_t * p_channel)
{
    *p_slice = pwm_gpio_to_slice_num(pin);
    *p_channel = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, pwm_divider);
    pwm_config_set_wrap(&config, pwm_wrap);
    pwm_init(*p_slice, &config, false);
    pwm_set_chan_level(*p_slice, *p_channel, 0u);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_enabled(*p_slice, true);
}

static void
set_levels (motor_t const * p_motor, uint16_t level_a, uint16_t level_b)
{
    pwm_set_chan_level(p_motor->slice_a, p_motor->channel_a, level_a);
    pwm_set_chan_level(p_motor->slice_b, p_motor->channel_b, level_b);
}
