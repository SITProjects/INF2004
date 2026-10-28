/**
 * @file obstacle_config.h
 * @brief Wiring, scan settings and calibration for obstacle sensing.
 *
 * Detection thresholds and timing defaults require physical validation.
 * Pin assignments must agree with TEAM.md and the actual wiring.
 */

#ifndef OBSTACLE_CONFIG_H
#define OBSTACLE_CONFIG_H

/* Public includes. */
#include "../robot_types.h"
#include "config.h"

/* Public types, constants, and macros. */

/* Vehicle dimensions. */
#define OBSTACLE_VEHICLE_WIDTH_MM 127u
#define OBSTACLE_VEHICLE_LENGTH_MM 162u

/* Wiring: planned allocation; confirm before changing the hardware. */
#define OBSTACLE_TRIG_PIN 16u
#define OBSTACLE_ECHO_PIN 17u
#define OBSTACLE_SERVO_PIN 12u

/* Task configuration: preserve the current priority and polling period. */
#define OBSTACLE_PRI_SCAN 12
#define OBSTACLE_UPDATE_PERIOD_MS 10u

/* Mechanically permitted sweep. Verify the complete arc on the car. */
#define OBSTACLE_SWEEP_FIRST_DEG ROBOT_OBSTACLE_SWEEP_FIRST_DEG
#define OBSTACLE_SWEEP_LAST_DEG ROBOT_OBSTACLE_SWEEP_LAST_DEG
#define OBSTACLE_SWEEP_STEP_DEG ROBOT_OBSTACLE_SWEEP_STEP_DEG
#define OBSTACLE_CENTRE_DEG 90u

#define OBSTACLE_SWEEP_POINTS ROBOT_OBSTACLE_SWEEP_POINTS

#define OBSTACLE_CENTRE_INDEX                                                  \
    ((OBSTACLE_CENTRE_DEG - OBSTACLE_SWEEP_FIRST_DEG) / OBSTACLE_SWEEP_STEP_DEG)

/* Side positions retain the median of nonzero samples. */
#define OBSTACLE_PINGS_PER_ANGLE 3u

/* Centre confirmation: first near ping plus two usable pings, majority wins. */
#define OBSTACLE_CENTRE_CONFIRM_PINGS 3u
#define OBSTACLE_CENTRE_NEAR_VOTES 2u

/*
 * Two extra attempts per side position or centre group, not per ping.
 * Invalid measurements must not cause an unbounded scan.
 */
#define OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE 2u

/* Centre trigger-to-trigger schedule; late polls do not replay missed pings.
 * The driver independently enforces minimum spacing across all angles.
 */
#define OBSTACLE_FORWARD_PERIOD_MS 60u
#define ULTRASONIC_PING_INTERVAL_US 60000u

/* Trigger waveform: preserve the existing driver values. */
#define ULTRASONIC_TRIG_SETTLE_US 2u
#define ULTRASONIC_TRIG_PULSE_US 10u

/*
 * Stored pulse width. Keep timestamps as uint32_t for elapsed-time maths.
 * Fifteen bits hold 0..32767; zero is reserved by the proposed interface.
 */
#define ULTRASONIC_ECHO_BITS 15u
#define ULTRASONIC_ECHO_MAX_US ((1u << ULTRASONIC_ECHO_BITS) - 1u)

/*
 * Two-part measurement timeout.
 *
 * Start timeout: provisional; measure trigger-to-rise delay on the scope.
 * Pulse timeout: maximum measurable in-range pulse, from the rising edge.
 * Crossing it reports NO_ECHO (beyond range) under the team policy. Some
 * modules may hold ECHO high longer in open space; scope validation is open.
 * A high input at a later trigger attempt remains a separate FAULT.
 */
#define ULTRASONIC_START_TIMEOUT_US 2000u
#define ULTRASONIC_PULSE_TIMEOUT_US 23000u

/* Existing conversion factor, expressed in millimetres per microsecond. */
#define OBSTACLE_MM_PER_ECHO_US 0.1715f

/* Kernel timer period; must match CNF_TIMER_PERIOD in config/config.h. */
#define OBSTACLE_TICK_MS 10u

/*
 * Millimetres to echo microseconds, rounded up (10000 / 1715 = 1 / 0.1715).
 * Rounding up makes "echo < threshold" trip at or just beyond the distance.
 * Exact in 32-bit unsigned arithmetic for distances up to 400000 mm.
 */
#define OBSTACLE_MM_TO_ECHO_US(mm) ((((mm)*10000u) + 1714u) / 1715u)

/*
 * Candidate thresholds, pending stopping-distance tests. Edit only the
 * millimetre values; the echo equivalents are derived.
 *
 * Detection: 300 mm -> 1750 us. A nearer centre reading requests a stop.
 */
#define OBSTACLE_DETECT_DISTANCE_MM 300u
#define OBSTACLE_DETECT_ECHO_US                                                \
    OBSTACLE_MM_TO_ECHO_US(OBSTACLE_DETECT_DISTANCE_MM)

/* Critical: 80 mm -> 467 us. One nearer centre reading sets the flag. */
#define OBSTACLE_CRITICAL_DISTANCE_MM 80u
#define OBSTACLE_CRITICAL_ECHO_US                                              \
    OBSTACLE_MM_TO_ECHO_US(OBSTACLE_CRITICAL_DISTANCE_MM)

/*
 * Critical release: 100 mm -> 584 us, the Week 6 minimum stopping gap.
 * The flag clears only after a complete centre group with no VALID reading
 * nearer than this. 80..100 mm is the hysteresis band.
 */
#define OBSTACLE_CRITICAL_RELEASE_DISTANCE_MM 100u
#define OBSTACLE_CRITICAL_RELEASE_ECHO_US                                      \
    OBSTACLE_MM_TO_ECHO_US(OBSTACLE_CRITICAL_RELEASE_DISTANCE_MM)

/* Candidate edge discontinuity: approximately 80 mm in radial range. */
#define OBSTACLE_EDGE_JUMP_US 466u

/*
 * Servo timing: preserve the existing uncalibrated one-second wait.
 * Replace these after measuring travel and settling under actual load.
 *
 * wait_ms = ceil(delta_deg * travel_ms_per_60_deg / 60) + margin_ms
 */
#define OBSTACLE_SERVO_TRAVEL_MS_PER_60_DEG 0u
#define OBSTACLE_SERVO_SETTLE_MARGIN_MS 1000u

/*
 * Width geometry: provisional zero corrections.
 * Pivot offset is signed, along the sensor's radial measurement axis.
 * A lateral mounting offset needs a separate coordinate transform.
 * Edge correction moves each detected edge toward the obstacle centre.
 */
#define OBSTACLE_SENSOR_RADIAL_OFFSET_MM 0.0f
#define OBSTACLE_EDGE_CORRECTION_DEG 0.0f

/*
 * Freshness. While the stop request is released, going longer than this
 * without a usable centre reading re-asserts it. Three missed forward pings
 * plus two ticks of scheduling jitter: 3 * 60 + 2 * 10 = 200 ms. A healthy
 * sensor peaks at forward period + transaction + tick = 95 ms.
 */
#define OBSTACLE_STALE_MISSED_PINGS 3u
#define OBSTACLE_FORWARD_MAX_AGE_MS                                            \
    ((OBSTACLE_STALE_MISSED_PINGS * OBSTACLE_FORWARD_PERIOD_MS) +              \
     (2u * OBSTACLE_TICK_MS))

/* One ultrasonic transaction (start + pulse timeout), rounded up: 25 ms. */
#define OBSTACLE_TRANSACTION_MS                                                \
    ((ULTRASONIC_START_TIMEOUT_US + ULTRASONIC_PULSE_TIMEOUT_US + 999u) / 1000u)

/* Driver ping spacing, rounded up to milliseconds: 60 ms. */
#define OBSTACLE_PING_INTERVAL_MS ((ULTRASONIC_PING_INTERVAL_US + 999u) / 1000u)

/* Servo wait for a move of deg degrees, as computed by move_sensor(). */
#define OBSTACLE_SETTLE_MS(deg)                                                \
    (((((deg)*OBSTACLE_SERVO_TRAVEL_MS_PER_60_DEG) + 59u) / 60u) +             \
     OBSTACLE_SERVO_SETTLE_MARGIN_MS)

/*
 * Worst case for one side position: every allowed trigger used (pings plus
 * extra attempts), each spaced by the ping interval plus one polling tick,
 * then the last transaction and one more tick: 5 * 70 + 25 + 10 = 385 ms.
 */
#define OBSTACLE_SIDE_POSITION_WORST_MS                                        \
    (((OBSTACLE_PINGS_PER_ANGLE + OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE) *         \
      (OBSTACLE_PING_INTERVAL_MS + OBSTACLE_TICK_MS)) +                        \
     OBSTACLE_TRANSACTION_MS + OBSTACLE_TICK_MS)

/*
 * Worst case for a side scan: centre-to-left move, left position,
 * left-to-right move, right position. The return to centre happens after
 * the result is published. Today: 1000 + 1000 + 2 * 385 = 2770 ms.
 */
#define OBSTACLE_SIDE_SCAN_WORST_MS                                            \
    (OBSTACLE_SETTLE_MS(OBSTACLE_CENTRE_DEG - OBSTACLE_SWEEP_FIRST_DEG) +      \
     OBSTACLE_SETTLE_MS(OBSTACLE_SWEEP_LAST_DEG - OBSTACLE_SWEEP_FIRST_DEG) +  \
     (2u * OBSTACLE_SIDE_POSITION_WORST_MS))

/*
 * Watchdog limits. The stop acknowledgement timeout belongs to the motion
 * handshake (integration). The side-scan timeout is twice the worst case:
 * 5540 ms today, about 2240 ms once the servo is calibrated.
 */
#define OBSTACLE_STOP_ACK_TIMEOUT_MS 500u
#define OBSTACLE_SCAN_TIMEOUT_MS (2u * OBSTACLE_SIDE_SCAN_WORST_MS)

/* Compile-time consistency checks. */

#if (OBSTACLE_CENTRE_CONFIRM_PINGS != 3u) || (OBSTACLE_CENTRE_NEAR_VOTES != 2u)
#error "Centre confirmation requires two of three usable pings"
#endif

#if OBSTACLE_FORWARD_PERIOD_MS < ((ULTRASONIC_PING_INTERVAL_US + 999u) / 1000u)
#error "Forward period cannot be shorter than the driver ping interval"
#endif

#if OBSTACLE_SWEEP_STEP_DEG == 0u
#error "Sweep step must be nonzero"
#else

#if OBSTACLE_SWEEP_LAST_DEG <= OBSTACLE_SWEEP_FIRST_DEG
#error "Sweep limits must be increasing"
#endif

#if ((OBSTACLE_SWEEP_LAST_DEG - OBSTACLE_SWEEP_FIRST_DEG) %                    \
     OBSTACLE_SWEEP_STEP_DEG) != 0u
#error "Sweep interval must divide exactly into the sweep range"
#endif

#if (OBSTACLE_CENTRE_DEG < OBSTACLE_SWEEP_FIRST_DEG) ||                        \
    (OBSTACLE_CENTRE_DEG > OBSTACLE_SWEEP_LAST_DEG)
#error "Centre must lie within the sweep"
#endif

#if ((OBSTACLE_CENTRE_DEG - OBSTACLE_SWEEP_FIRST_DEG) %                        \
     OBSTACLE_SWEEP_STEP_DEG) != 0u
#error "Centre must be one of the sampled angles"
#endif

#if OBSTACLE_SWEEP_POINTS > 15u
#error "Four-bit edge index reserves 15 for unavailable"
#endif

#endif /* OBSTACLE_SWEEP_STEP_DEG */

#if (ULTRASONIC_ECHO_BITS < 1u) || (ULTRASONIC_ECHO_BITS > 15u)
#error "Echo field must fit the agreed 15-bit maximum"
#endif

#if ULTRASONIC_PULSE_TIMEOUT_US > ULTRASONIC_ECHO_MAX_US
#error "Pulse timeout exceeds the stored echo range"
#endif

#if OBSTACLE_CRITICAL_DISTANCE_MM >= OBSTACLE_DETECT_DISTANCE_MM
#error "Critical distance must be below the detection distance"
#endif

#if !((OBSTACLE_CRITICAL_ECHO_US < OBSTACLE_CRITICAL_RELEASE_ECHO_US) &&       \
      (OBSTACLE_CRITICAL_RELEASE_ECHO_US < OBSTACLE_DETECT_ECHO_US))
#error "Thresholds must satisfy critical < critical release < detection"
#endif

#if (OBSTACLE_DETECT_ECHO_US >= ULTRASONIC_PULSE_TIMEOUT_US) ||                \
    (OBSTACLE_DETECT_ECHO_US > ULTRASONIC_ECHO_MAX_US)
#error "Detection threshold must lie inside the measurable echo range"
#endif

#if OBSTACLE_FORWARD_MAX_AGE_MS <=                                             \
    (OBSTACLE_FORWARD_PERIOD_MS + OBSTACLE_TRANSACTION_MS + OBSTACLE_TICK_MS)
#error "Freshness limit must exceed the age of a healthy forward reading"
#endif

#if OBSTACLE_SERVO_SETTLE_MARGIN_MS < OBSTACLE_TRANSACTION_MS
#error "Settle margin must outlast an in-flight measurement (drain rule)"
#endif

#if OBSTACLE_TICK_MS != CNF_TIMER_PERIOD
#error "Tick must match CNF_TIMER_PERIOD in config/config.h"
#endif

/* Public function declarations. */

#endif /* OBSTACLE_CONFIG_H */
