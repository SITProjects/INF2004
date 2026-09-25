/**
 * @file obstacle_scan.h
 * @brief Nonblocking obstacle detection and directional scanning.
 */

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

/* Public types, constants, and macros. */

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
    obstacle_status_t status;
    /** Servo angle in degrees: 90 for CLEAR, selected side for PATH_FOUND,
     * -1 otherwise. */
    int angle;
    /** Sampled width estimate in cm for PATH_FOUND or BLOCKED when both side
     * echoes are valid; -1 when unavailable or while scanning.
     * Assumes the centre and side echoes come from the same obstacle.
     * Fixed scan angles do not locate the obstacle's actual edges. */
    float width_cm;
} obstacle_scan_result_t;

/* Public function declarations. */

/**
 * @brief Advance the scan without waiting for servo movement or echo arrival.
 * Call regularly on the initializing core after obstacle_scan_init() succeeds.
 * CLEAR means the centre is clear; PATH_FOUND means a clear side was found,
 * not that the vehicle has turned.
 * BLOCKED means all tested directions are blocked; UNKNOWN means an echo was
 * unavailable and no clear side was found. A centre obstacle triggers both
 * 60- and 120-degree measurements before publishing a completed result.
 * PATH_FOUND selects the first clear side in that order.
 * Completed results persist during return-to-centre settling.
 * Only CLEAR and PATH_FOUND carry a usable angle. Persistent obstacles cause
 * repeated scans. The caller owns motion decisions for every status.
 * @return A caller-owned snapshot of the current scan result.
 */
obstacle_scan_result_t obstacle_scan_update (void);

/**
 * @brief Initialise the sensor and centre the servo before scanning.
 * @param trig_pin Ultrasonic trigger GPIO number.
 * @param echo_pin Ultrasonic echo GPIO number, distinct from trig_pin.
 * @param servo_pin Servo GPIO number, distinct from both ultrasonic pins.
 * @return 0 on success, 1 for invalid pins or an unsupported system clock, or a call not on core 0.
 * Initialise once and call obstacle_scan_update() from the initializing task on core 0.
 */
int obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin);

/** Printable status name; never NULL. */
char const * obstacle_scan_status_name (obstacle_status_t status);

#endif /* OBSTACLE_SCAN_H */
