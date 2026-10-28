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

/* Public types, constants, and macros. */

/* Wiring: planned allocation; confirm before changing the hardware. */
#define OBSTACLE_TRIG_PIN 16u
#define OBSTACLE_ECHO_PIN 17u
#define OBSTACLE_SERVO_PIN 12u

/* Task configuration: preserve the current priority and polling period. */
#define OBSTACLE_PRI_SCAN 12
#define OBSTACLE_UPDATE_PERIOD_MS 10u

/* Mechanically permitted sweep. Verify the complete arc on the car. */
#define OBSTACLE_SWEEP_FIRST_DEG 60u
#define OBSTACLE_SWEEP_LAST_DEG 120u
#define OBSTACLE_SWEEP_STEP_DEG 5u
#define OBSTACLE_CENTRE_DEG 90u

#define OBSTACLE_SWEEP_POINTS                                                  \
    (((OBSTACLE_SWEEP_LAST_DEG - OBSTACLE_SWEEP_FIRST_DEG) /                   \
      OBSTACLE_SWEEP_STEP_DEG) +                                               \
     1u)

#define OBSTACLE_CENTRE_INDEX                                                  \
    ((OBSTACLE_CENTRE_DEG - OBSTACLE_SWEEP_FIRST_DEG) / OBSTACLE_SWEEP_STEP_DEG)

/* Three measurements per angle; median/filter behavior is in code. */
#define OBSTACLE_PINGS_PER_ANGLE 3u

/*
 * Proposed retry budget: two extra attempts per angle, not per ping.
 * Invalid measurements must not cause an unbounded scan.
 */
#define OBSTACLE_EXTRA_ATTEMPTS_PER_ANGLE 2u

/* Forward monitoring and minimum spacing between any two triggers. */
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
 * Pulse timeout: retains the existing 23 ms duration limit, but now starts
 * at the rising edge. Total transaction time can therefore be longer.
 */
#define ULTRASONIC_START_TIMEOUT_US 2000u
#define ULTRASONIC_PULSE_TIMEOUT_US 23000u

/* Existing conversion factor, expressed in millimetres per microsecond. */
#define OBSTACLE_MM_PER_ECHO_US 0.1715f

/*
 * Candidate thresholds, pending stopping-distance tests.
 * Echo equivalents are rounded using the conversion above.
 */
#define OBSTACLE_DETECT_DISTANCE_MM 300u
#define OBSTACLE_DETECT_ECHO_US 1749u

#define OBSTACLE_CRITICAL_DISTANCE_MM 80u
#define OBSTACLE_CRITICAL_ECHO_US 466u

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
 * Proposed watchdog limits; validate with the integration owners.
 * The sweep limit accommodates the current slow settling default.
 */
#define OBSTACLE_STOP_ACK_TIMEOUT_MS 500u
#define OBSTACLE_SCAN_TIMEOUT_MS 20000u
#define OBSTACLE_FORWARD_MAX_AGE_MS 200u

/* Compile-time consistency checks. */

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

#define OBSTACLE_VEHICLE_WIDTH_MM 200u
#define OBSTACLE_VEHICLE_LENGTH_MM 300u

/* Public function declarations. */

#endif /* OBSTACLE_CONFIG_H */
