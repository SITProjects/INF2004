/**
 * @file motion_config.h
 * @brief Pin assignments and calibration constants for the motion module.
 *
 * Values marked "TODO(calibrate)" are starting guesses; measure them on
 * the car with the named test. List them all with:
 *     grep -rn "TODO(calibrate)" app_program/motion
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

/*
 * Set true for a wheel that spins backward on a forward command.
 * TODO(calibrate): confirm with MOT-U06 that FORWARD turns each wheel the
 * way that drives the car forward.
 */
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

/*
 * Set true for an encoder that counts down when its wheel drives forward.
 * TODO(calibrate): set from MOT-U05/MOT-U06 once the encoders are wired.
 */
#define MOTION_LEFT_ENCODER_INVERTED false
#define MOTION_RIGHT_ENCODER_INVERTED false

/*
 * Encoder counts per wheel turn, counting all 4 edges per encoder cycle.
 * TODO(calibrate): placeholder; measure by turning each wheel one
 * revolution by hand (testing/motion/TEST_LOG.md, MOT-U04).
 */
#define MOTION_ENCODER_TICKS_PER_REV 1000u

/*
 * Wheel diameter in mm; odometry uses pi * diameter / ticks per rev.
 * Measured 64 mm. TODO(calibrate): tune with MOT-U08 (move_forward(1000)
 * against a ruler) until the distance is within about one count.
 */
#define MOTION_WHEEL_DIAMETER_MM 64.0f

/*
 * Distance between the two wheels' contact points (tyre centres), in mm.
 * Measured 127 mm outside to outside, less one 19.5 mm tyre width.
 * TODO(calibrate): tune with MOT-U11 and MOT-U12 (360 degrees three times
 * each way) on the demo floor: new = old * 1080 / mean of the two measured
 * total angles. Calibrate the wheel diameter (MOT-U08) first, since turns
 * depend on track width / mm per count.
 */
#define MOTION_TRACK_WIDTH_MM 107.5f

/*
 * Highest wheel speed any command may request, in mm/s.
 * TODO(calibrate): about 10-15 % below the slower wheel's speed at duty
 * 1000 in MOT-U19, so the speed loop always has duty in hand (also
 * confirms the 16-bit counter margin).
 */
#define MOTION_MAX_SPEED_MM_S 500.0f

/*
 * Feed-forward duty = MOTION_FF_OFFSET + MOTION_FF_GAIN * speed_mm_s; the
 * speed PI loop corrects what this misses.
 * OFFSET: smallest duty that just starts the wheel turning.
 * GAIN: duty per mm/s above that.
 * TODO(calibrate): fit a straight line to the MOT-U19 duty/speed lines of
 * both wheels: GAIN = duty change / speed change between two lines above
 * the start-up duty, OFFSET = duty - GAIN * speed at either of them.
 */
#define MOTION_FF_OFFSET 0.20f
#define MOTION_FF_GAIN 0.0012f

/*
 * Speed PI loop of each wheel, on top of the feed-forward duty (report
 * 8.1). KP: duty per mm/s of speed error. KI: duty per mm/s of error per
 * second. TODO(calibrate) with MOT-U20 (lifted): with KI 0, raise KP until
 * the speed overshoots a step, then halve it; then raise KI until the
 * speed settles on the setpoint within about 0.5 s without oscillating.
 * Check on the floor with MOT-U21.
 */
#define MOTION_SPEED_KP 0.001f
#define MOTION_SPEED_KI 0.003f

/*
 * Updates over which each wheel's speed is averaged. At 2 mm per count
 * and 200 mm/s a wheel moves only about 2 counts per update, so one
 * update's speed is too coarse to control on. More updates are smoother
 * but slower to respond.
 * TODO(calibrate): choose once MOT-U04 gives the counts per revolution;
 * fewer if each update sees 10 or more counts at 200 mm/s.
 */
#define MOTION_SPEED_WINDOW_UPDATES 5u

/*
 * Straight-line (sync) loop of distance moves (plan step 6). It holds the
 * heading a move started on by adding a turn rate, in deg/s, to both
 * wheels' setpoints. The error is the heading change since the move
 * started, from the right - left count difference; one count of
 * difference is about 1 deg at 2 mm per count. KP: deg/s per deg of
 * error. KI: deg/s per deg of error per second.
 * TODO(calibrate) with MOT-U25 on the floor: with KI 0, raise KP until the
 * car weaves visibly over 1 m, then halve it; then raise KI only if the
 * final heading is off the same way on every run.
 */
#define MOTION_SYNC_KP 3.0f
#define MOTION_SYNC_KI 0.5f

/*
 * Largest sync correction, in deg/s. Each wheel's setpoint changes by
 * rate * track / 2 (about 19 mm/s at 20 deg/s), which must stay well below
 * MOTION_MOVE_SPEED_MM_S so neither wheel slows to a stop or reverses.
 * TODO(calibrate): large enough to hold MOT-U25 straight, small enough
 * that starts do not swerve.
 */
#define MOTION_SYNC_MAX_DEG_S 20.0f

/*
 * Period between motion_move_update() calls, in ms (CYC_MOTION in the
 * design review). Speeds and the stall timeout are based on it.
 */
#define MOTION_UPDATE_PERIOD_MS 20u

/*
 * Chassis speed of distance moves, in mm/s. The MBF_MOTION_CMD packing
 * has no speed field, so every move uses this speed.
 */
#define MOTION_MOVE_SPEED_MM_S 200

/*
 * Turn rate of turns in place, in degrees per second; each wheel moves at
 * rate * track / 2 (about 84 mm/s at 90 deg/s). The MBF_MOTION_CMD
 * packing has no speed field, so every turn uses this rate.
 * TODO(calibrate): the slowest rate that turns smoothly on the floor
 * without stalling (MOT-U11); slower skids and overshoots less.
 */
#define MOTION_TURN_SPEED_DEG_S 90

/*
 * Degrees to brake before a turn's target, for the coast after braking.
 * TODO(calibrate): leave 0 unless MOT-U13 shows every 90 degree turn off
 * by about the same amount once MOT-U11 is right; then set it to that
 * overshoot (the "coast" in the MOT-U13 lines).
 */
#define MOTION_TURN_STOP_EARLY_DEG 0.0f

/*
 * Largest turn one call accepts, in degrees: enough for calibration turns,
 * small enough to catch a mistaken angle.
 */
#define MOTION_MAX_TURN_DEG 720u

/*
 * A move or turn still short of its target after MOTION_TIMEOUT_FACTOR
 * times its expected time, plus MOTION_TIMEOUT_MARGIN_MS for start-up,
 * brakes and enters MOTION_STATE_TIMED_OUT. The wheels are turning, so it
 * is not a stall: an encoder counting the wrong way (MOT-U17) or wheels
 * slipping on the spot.
 */
#define MOTION_TIMEOUT_FACTOR 3u
#define MOTION_TIMEOUT_MARGIN_MS 500u

/*
 * A wheel with no encoder change for this long during a move or turn
 * means a stall (FR12): both wheels brake and the state becomes STALLED.
 * A multiple of MOTION_UPDATE_PERIOD_MS.
 */
#define MOTION_STALL_TIMEOUT_MS 300u

/*
 * Set 1 to add the manual bench steps (MOT-U04..U06) to make TEST=1. They
 * spin each wheel under power and block the test run for about 33 s, so
 * keep this 0 except on a lifted car during a motion test session.
 */
#define MOTION_MANUAL_TESTS 0

/*
 * Set 1 to add the manual move and stall steps (MOT-U08..U10) to make
 * TEST=1. They drive the car on the floor, then spin it lifted, and block
 * the test run for about a minute; keep this 0 except during a motion test
 * session.
 */
#define MOTION_MOVE_TESTS 0

/*
 * Set 1 to add the manual turn steps (MOT-U11..U16) to make TEST=1. They
 * turn the car on the floor, then spin it lifted, and block the test run
 * for about four minutes; keep this 0 except during a motion test
 * session.
 */
#define MOTION_TURN_TESTS 0

/*
 * Set 1 to add the manual speed-loop steps (MOT-U19..U21) to make TEST=1.
 * They spin the lifted car's wheels, then drive it on the floor, and block
 * the test run for about two minutes; keep this 0 except during a motion
 * test session.
 */
#define MOTION_SPEED_TESTS 0

/*
 * Set 1 to add the manual straight-line steps (MOT-U24..U26) to make
 * TEST=1. They spin the lifted car's wheels, then drive it on the floor,
 * and block the test run for about five minutes; keep this 0 except
 * during a motion test session.
 */
#define MOTION_SYNC_TESTS 0

/* Public function declarations. */

#endif /* MOTION_CONFIG_H */
