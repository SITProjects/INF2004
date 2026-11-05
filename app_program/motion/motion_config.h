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
 * MOT-U06 (2026-11-03): FORWARD turns both wheels forward.
 */
#define MOTION_LEFT_MOTOR_INVERTED false
#define MOTION_RIGHT_MOTOR_INVERTED false

/*
 * Quadrature encoder phases A/B, counted by PIO0 (drv_encoder.c): left on
 * the GP13/GP14 servo header signal pins, right on Grove P2. B must be the
 * pin after A. Power the encoders from 3.3 V: the RP2040 inputs are not
 * 5 V tolerant.
 */
#define MOTION_LEFT_ENCODER_PIN_A 13u
#define MOTION_LEFT_ENCODER_PIN_B 14u
#define MOTION_RIGHT_ENCODER_PIN_A 2u
#define MOTION_RIGHT_ENCODER_PIN_B 3u

/*
 * Set true for an encoder that counts down when its wheel drives forward.
 * MOT-U05/MOT-U06 (2026-11-03): the right encoder counts down.
 */
#define MOTION_LEFT_ENCODER_INVERTED false
#define MOTION_RIGHT_ENCODER_INVERTED true

/*
 * Encoder counts per wheel turn, counting all 4 edges per encoder cycle.
 * MOT-U04 (2026-11-03): about 2500 left and 2530 right, turned by hand.
 * MOT-U04 (2026-11-05): 5 turns each way, about 2539 left and 2525 right.
 */
#define MOTION_ENCODER_TICKS_PER_REV 2532u

/*
 * Wheel diameter in mm; odometry uses pi * diameter / ticks per rev.
 * Measured 64 mm. MOT-U08 (2026-11-03): odometry agreed with the ruler
 * within about 1 mm over 1 m at 2490 ticks per rev.
 * MOT-U08 (2026-11-05): at 2532 ticks per rev, ruler about 1000 mm and
 * odometry 1003-1004 mm over 1 m, so 64 mm kept.
 */
#define MOTION_WHEEL_DIAMETER_MM 64.0f

/*
 * Distance between the two wheels' contact points (tyre centres), in mm.
 * Measured 107.5 mm: 127 mm outside to outside, less one 19.5 mm tyre width.
 * This is the effective value for turns, not the measured one.
 * MOT-U11/U12 (2026-11-05): about 10 degrees short per 360 by eye, so a
 * total of about 1050: 107.5 * 1080 / 1050 = 110.6. MOT-U11..U14 pass at
 * 110.6 (2026-11-05), checked by eye.
 * To retune with MOT-U11 and MOT-U12 (360 degrees three times each way) on
 * the demo floor: new = old * 1080 / mean of the two measured total angles.
 * Calibrate the wheel diameter (MOT-U08) first, since turns depend on track
 * width / mm per count.
 */
#define MOTION_TRACK_WIDTH_MM 110.6f

/*
 * Highest wheel speed any command may request, in mm/s.
 * Set about 10-15 % below the slower wheel's speed at duty 1000 in MOT-U19,
 * so the speed loop always has duty in hand (also confirms the 16-bit
 * counter margin). MOT-U19 (2026-11-05): the slower wheel reached 728 mm/s.
 */
#define MOTION_MAX_SPEED_MM_S 620.0f

/*
 * Feed-forward duty = MOTION_FF_OFFSET + MOTION_FF_GAIN * speed_mm_s; the
 * speed PI loop corrects what this misses.
 * GAIN: duty per mm/s, from a straight line through the lifted MOT-U19
 * duty/speed lines of both wheels (2026-11-05, duty 200-600).
 * OFFSET: deliberately tuned for the floor, not taken from that line, whose
 * lifted fit is 0.009. Do not re-fit it from MOT-U19 alone. The floor needs
 * about duty 300 for 200 mm/s (MOT-U21), and at 0.009 the gentle start let
 * the castor flip swallow a short reverse: 100 mm moved 75-85 mm (MOT-U09).
 * The cost is overshoot when lifted: +17/+21 % on the first MOT-U20 step,
 * still settling within 5 % in about 0.5 s. If this changes, repeat MOT-U09
 * and MOT-U20 and check MOTION_MOVE_STOP_EARLY_MM.
 */
#define MOTION_FF_OFFSET 0.05f
#define MOTION_FF_GAIN 0.00122f

/*
 * Speed PI loop of each wheel, on top of the feed-forward duty (report
 * 8.1). KP: duty per mm/s of speed error. KI: duty per mm/s of error per
 * second. MOT-U20 and MOT-U21 (2026-11-05): KP kept, KI raised from 0.003
 * until each step settled within 5 % in about 0.5 s.
 * To retune with MOT-U20 (lifted): with KI 0, raise KP until the speed
 * overshoots a step, then halve it; then raise KI until the speed settles
 * on the setpoint within about 0.5 s without oscillating. Check on the
 * floor with MOT-U21.
 */
#define MOTION_SPEED_KP 0.001f
#define MOTION_SPEED_KI 0.006f

/*
 * Updates over which each wheel's speed is averaged. More updates are
 * smoother but slower to respond. MOT-U04 (2026-11-05) gives about 0.08 mm
 * per count: about 50 counts per update at 200 mm/s and 25 at 100 mm/s.
 * MOT-U20 and MOT-U21 (2026-11-05) pass with 5. Fewer may respond faster;
 * repeat MOT-U08..U10 and MOT-U19..U21 if this changes.
 */
#define MOTION_SPEED_WINDOW_UPDATES 5u

/*
 * Straight-line (sync) loop of distance moves (plan step 6). It holds the
 * heading a move started on by adding a turn rate, in deg/s, to both
 * wheels' setpoints. The error is the heading change since the move
 * started, from the right - left count difference; one count of
 * difference is about 0.04 deg at 0.08 mm per count. KP: deg/s per deg of
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
 * Chassis speed of line following, in mm/s. The MBF_MOTION_CMD packing
 * has no speed field, so tMotion follows at this speed.
 * TODO(calibrate) with MOT-U31: start slow; raise only once the line gains
 * hold the tightest curve of the course.
 */
#define MOTION_FOLLOW_SPEED_MM_S 120u

/*
 * Line PD loop of line following (plan step 6b). It turns the line error
 * (-MOTION_LINE_ERROR_MAX to MOTION_LINE_ERROR_MAX, + is line to the
 * left) into a turn rate for both wheels' setpoints. KP: deg/s per unit of
 * error, so 0.1 turns at 100 deg/s with the line at one side. KD: deg/s per
 * unit of error per second (in seconds).
 * TODO(calibrate) with MOT-U31 on a straight, then the tightest curve:
 * with KD 0, raise KP until the car weaves along a straight line, then
 * halve it; then raise KD until the weave dies within about one swing.
 */
/* Validated on the robot at 120 mm/s on straights and both curve directions. */
#define MOTION_FOLLOW_KP 0.3f
#define MOTION_FOLLOW_KD 0.006f

/*
 * Largest turn rate the line loop asks for, in deg/s. Each wheel's
 * setpoint changes by rate * track / 2 (about 113 mm/s at 120 deg/s), so
 * at MOTION_FOLLOW_SPEED_MM_S the inner wheel nearly stops. Following
 * also caps the rate at 2 * speed / track (128 deg/s at 120 mm/s), so the
 * inner wheel never reverses at a slower follow speed or a wider track.
 * TODO(calibrate): at least the follow speed / the tightest curve radius
 * (in rad/s), so the car can hold that curve (MOT-U31).
 */
/* Bound the combined proportional and damping correction: the
 * 2 * speed / track limit at 120 mm/s, for the tightest test curves. */
#define MOTION_FOLLOW_MAX_DEG_S 128.0f

/*
 * Following stops (state STOPPED) when the last line error is older than
 * this, so the car never drives on with stale steering. About five of the
 * line task's 20 ms readings. A multiple of MOTION_UPDATE_PERIOD_MS.
 */
#define MOTION_LINE_TIMEOUT_MS 100u

/*
 * Readings in a row that must agree before the line counts as seen, or as
 * gone; one noisy reading cannot end a move at a line.
 * TODO(calibrate) with MOT-U32: raise if moves stop short of the line or
 * on marks that are not a line; each extra reading adds about 20 ms (4 mm
 * at 200 mm/s) of overshoot.
 */
#define MOTION_LINE_SEEN_SAMPLES 2u

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
 * Millimetres to brake before a move's target, for the coast after
 * braking. The coast grows with speed, so this holds only at
 * MOTION_MOVE_SPEED_MM_S; measure again if that changes. The target is
 * checked once per update, so a move also brakes up to one update's
 * travel (4 mm at 200 mm/s) late.
 * MOT-U08/U09 (2026-11-05): with MOTION_FF_OFFSET 0.05, 9 mm ends 1 m moves
 * at 996-998 mm and 10 cm reverses at 99-102 mm by odometry; the ruler
 * agreed by eye. It depends on that offset as well as on the speed.
 */
#define MOTION_MOVE_STOP_EARLY_MM 9.0f

/*
 * Turn rate of turns in place, in degrees per second; each wheel moves at
 * rate * track / 2 (about 87 mm/s at 90 deg/s). The MBF_MOTION_CMD
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
 * Commands MBF_MOTION_CMD holds before a sender waits; tMotion empties it
 * every update. TODO(step 9, tMission): raise it if tMission ever sees
 * E_TMOUT from tk_snd_mbf().
 */
#define MOTION_CMD_QUEUE_DEPTH 4u

/*
 * Set 1 to add the manual bench steps (MOT-U04..U06) to make TEST=1. They
 * spin each wheel under power and block the test run for about 133 s, so
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

/*
 * Set 1 to add the manual line-following steps (MOT-U28..U30) to make
 * TEST=1. They spin the lifted car's wheels, then drive it on the floor
 * with hand-fed line errors, and block the test run for about a minute and
 * a half; keep this 0 except during a motion test session.
 */
#define MOTION_FOLLOW_TESTS 0

/*
 * Set 1 to add the manual tMotion steps (MOT-U36, MOT-U37) to make TEST=1.
 * They send commands to tMotion through MBF_MOTION_CMD and spin the lifted
 * car's wheels, and block the test run for about a minute; keep this 0
 * except during a motion test session.
 */
#define MOTION_TASK_TESTS 0

/* Public function declarations. */

#endif /* MOTION_CONFIG_H */
