/**
 * @file obstacle_scan.h
 * @brief Nonblocking obstacle detection and directional scanning.
 */
#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

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
    /* 90 for CLEAR, selected angle for PATH_FOUND, otherwise -1. */
    int angle;
} obstacle_scan_result_t;

/**
 * @brief Advance the scan without waiting for servo movement or echo arrival.
 * Call regularly from the task that called obstacle_scan_init(). CLEAR means
 * the centre is clear; PATH_FOUND means a clear side was found, not that the
 * vehicle has turned. BLOCKED means all tested directions are blocked;
 * UNKNOWN means an echo was unavailable. Completed results persist during
 * return-to-centre settling. Only CLEAR and PATH_FOUND carry a usable
 * angle. Persistent obstacles cause repeated scans. The caller owns motion
 * decisions for every status.
 */
obstacle_scan_result_t obstacle_scan_update (void);

/**
 * @return 0 on success, 1 for invalid pins, an unsupported system clock, or
 * a call that is not on RP2040 core 0 (see drv_ultrasonic.h).
 * Initialise once before scanning.
 */
int obstacle_scan_init (int trig_pin, int echo_pin, int servo_pin);

/**
 * @brief Printable name of a scan status.
 * @param status Any obstacle_status_t value.
 * @return A constant string; never NULL.
 */
char const * obstacle_scan_status_name (obstacle_status_t status);

#endif /* OBSTACLE_SCAN_H */
