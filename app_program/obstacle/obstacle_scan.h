/**
 * @file obstacle_scan.h
 * @brief Nonblocking obstacle detection and directional scanning.
 */

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

/* Public includes. */

#include <stdint.h>

/* Public types, constants, and macros. */

/** Servo timing measured at the operating voltage and mechanical load. */
typedef struct
{
    /** Travel time in milliseconds per 60 degrees, excluding settling. */
    uint16_t travel_ms_per_60_degrees;
    /** Extra settling time in milliseconds after the estimated travel. */
    uint16_t settle_margin_ms;
} obstacle_servo_timing_t;

typedef enum
{
    OBSTACLE_SCANNING,
    OBSTACLE_CLEAR,
    OBSTACLE_PATH_FOUND,
    OBSTACLE_BLOCKED,
    OBSTACLE_UNKNOWN
} obstacle_status_t;

typedef struct
{
    /** The current status of the obstacle detection. */
    obstacle_status_t status;
    /** Starts at zero; increments for each completed centre or side scan,
     * or a state recovery fault. Wraps modulo 2^32. Unchanged while waiting
     * or SCANNING. Compare with the last consumed sequence for fresh results.
     */
    uint32_t sequence;
    /** Sum of centre-to-side spans in cm. Invalid readings and clear sides
     * (>= 5 cm) contribute zero. Remaining readings contribute only when
     * <= 1.3 times the centre distance. One contributing side is retained
     * without doubling. A zero total is reported as -1 (unavailable), as is
     * the span while scanning. Assumes contributing echoes hit the same
     * object as the centre echo; does not locate the object's edges.
     */
    float sampled_span_cm;
    /** Number of sides contributing to sampled_span_cm: 0, 1, or 2.
     * One contribution is a partial span; even two do not locate edges.
     */
    uint8_t contributing_sides;
    /** Direction in degrees: 90 for CLEAR, 60 or 120 for PATH_FOUND,
     * -1 otherwise. PATH_FOUND selects the farthest valid clear side;
     * equal distances select the left side (60 degrees).
     */
    int path_found_angle;
} obstacle_scan_result_t;

/* Public function declarations. */

/**
 * @brief Perform nonblocking obstacle detection and directional scanning.
 * Complete both side measurements before publishing a side scan result,
 * including when the first side is clear. Call regularly from the initializing
 * task on core 0 after obstacle_scan_init() succeeds. Completed results persist
 * while returning to centre; subsequent scans start with zero accumulated span.
 * Only act on a
 * completed result once per sequence. PATH_FOUND does not mean the vehicle
 * has turned or that a vehicle-width corridor has been measured.
 * @return The current scan result:
 *       OBSTACLE_SCANNING: still scanning, no path found yet
 *       OBSTACLE_CLEAR: no obstacle detected in front
 *       OBSTACLE_PATH_FOUND: obstacle detected, but a path is available
 *       OBSTACLE_BLOCKED: obstacle detected, no path available
 *       OBSTACLE_UNKNOWN: sensor error or invalid reading
 */
obstacle_scan_result_t obstacle_scan_update (void);

/**
 * @brief Initialise the sensor and centre the servo before scanning.
 * @param trig_pin Ultrasonic trigger GPIO number.
 * @param echo_pin Ultrasonic echo GPIO number, distinct from trig_pin.
 * @param servo_pin Servo GPIO number, distinct from both ultrasonic pins.
 * @param timing Non-null timing configuration, copied during this call.
 * At least one timing field must be nonzero. Travel time is rounded up and
 * added to the settling margin. Startup assumes up to 180 degrees of travel
 * because the physical position is unknown. A travel time of zero with a
 * 1000 ms margin preserves the original, uncalibrated one-second wait.
 * @return 0 on success, 1 for invalid pins/timing, an unsupported system clock,
 * or a call not on core 0. Initialise once; call obstacle_scan_update() from
 * the initializing task on core 0.
 */
int obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin,
                        const obstacle_servo_timing_t * timing);

/** Printable status name; never NULL. */
char const * obstacle_scan_status_name (obstacle_status_t status);

#endif /* OBSTACLE_SCAN_H */
