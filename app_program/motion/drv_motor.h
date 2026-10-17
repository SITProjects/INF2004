/**
 * @file drv_motor.h
 * @brief Two-input H-bridge DC motor driver using hardware PWM.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */
typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
} motor_id_t;

/* Public function declarations. */

/**
 * @brief Configure both H-bridge inputs of one motor for PWM and brake it.
 * @param id Motor to configure.
 * @param pin_a Input driven with PWM for forward rotation.
 * @param pin_b Input driven with PWM for reverse rotation.
 * @param b_inverted Swap directions for a motor mounted in mirror image.
 * @return True on success; false for an invalid motor, pin, or clock.
 * The PWM slices of both pins belong to this driver afterwards.
 */
bool motor_init (motor_id_t id, uint32_t pin_a, uint32_t pin_b,
                 bool b_inverted);

/**
 * @brief Drive a motor at a signed duty cycle.
 * @param duty Fraction from -1.0 (full reverse) to 1.0 (full forward);
 * values outside the range are clamped, and 0.0 brakes the motor.
 */
void motor_set_duty (motor_id_t id, float duty);

/** @brief Short the motor terminals so the wheel stops quickly. */
void motor_brake (motor_id_t id);

#endif /* DRV_MOTOR_H */
