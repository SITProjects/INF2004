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

/*
 * Quadrature encoder phases A/B, counted by PIO0 (drv_encoder.c): left on
 * Grove P2, right on the GP13/GP14 servo header signal pins. B must be the
 * pin after A. Power the encoders from 3.3 V: the RP2040 inputs are not
 * 5 V tolerant.
 */
#define MOTION_LEFT_ENCODER_PIN_A 2u
#define MOTION_LEFT_ENCODER_PIN_B 3u
#define MOTION_RIGHT_ENCODER_PIN_A 13u
#define MOTION_RIGHT_ENCODER_PIN_B 14u

/* Set true for an encoder that counts down when its wheel drives forward. */
#define MOTION_LEFT_ENCODER_INVERTED false
#define MOTION_RIGHT_ENCODER_INVERTED false

/*
 * Encoder counts per wheel turn, counting all 4 edges per encoder cycle.
 * Placeholder: measure by turning each wheel one revolution by hand
 * (testing/motion/TEST_LOG.md, MOT-U04).
 */
#define MOTION_ENCODER_TICKS_PER_REV 1000u

/* Distance between the two wheels' contact points, in mm. Calibrate. */
#define MOTION_TRACK_WIDTH_MM 115.0f

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
