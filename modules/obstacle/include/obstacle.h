/**
 * @file obstacle.h
 * @brief Nonblocking obstacle detection and directional scanning.
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

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
 * Call regularly on the initializing core. CLEAR means the centre is clear;
 * PATH_FOUND means a clear side was found, not that the vehicle has turned.
 * BLOCKED means all tested directions are blocked; UNKNOWN means an echo was
 * unavailable. Completed results persist during return-to-centre settling.
 * Only CLEAR and PATH_FOUND carry a usable angle. Persistent obstacles cause
 * repeated scans. The caller owns motion decisions for every status.
 */
obstacle_scan_result_t obstacle_scan (void);

/**
 * @return 0 on success, 1 for invalid pins or an unsupported system clock.
 * Initialise once before scanning.
 */
int obstacle_init (int trig_pin, int echo_pin, int servo_pin);

#endif /* OBSTACLE_H */
