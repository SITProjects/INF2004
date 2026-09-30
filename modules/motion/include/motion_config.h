/**
 * @file motion_config.h
 * @brief Pin assignments and calibration constants for the motion module.
 *
 * Values marked "calibrate" are starting guesses; measure them on the car.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* Public includes. */
#include <stdbool.h>

/* Public types, constants, and macros. */

/* Robo Pico motor ports: M1 = GP8/GP9, M2 = GP10/GP11. */
#define MOTION_LEFT_MOTOR_PIN_A 8u
#define MOTION_LEFT_MOTOR_PIN_B 9u
#define MOTION_RIGHT_MOTOR_PIN_A 10u
#define MOTION_RIGHT_MOTOR_PIN_B 11u

/* Set true for a wheel that spins backward on a forward command. */
#define MOTION_LEFT_MOTOR_INVERTED false
#define MOTION_RIGHT_MOTOR_INVERTED false

/* Distance between the two wheels' contact points, in mm. Calibrate. */
#define MOTION_TRACK_WIDTH_MM 130.0f

/* Highest wheel speed any command may request, in mm/s. */
#define MOTION_MAX_SPEED_MM_S 500.0f

/*
 * Open-loop duty = MOTION_FF_OFFSET + MOTION_FF_GAIN * speed_mm_s.
 * OFFSET: smallest duty that just starts the wheel turning. Calibrate.
 * GAIN: duty per mm/s above that. Calibrate.
 */
#define MOTION_FF_OFFSET 0.20f
#define MOTION_FF_GAIN 0.0012f

/* Public function declarations. */

#endif /* MOTION_CONFIG_H */
