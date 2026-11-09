/**
 * @file car_status.h
 * @brief One console line of the car's state, for the dashboard.
 *
 * "CAR <key>=<value> ..." with speed, odometry, motion mode, the obstacle
 * and IMU results; tools/dashboard/car_dashboard.py reads it. Printed on the
 * console, so MQTT builds publish it through the console mirror.
 */

#ifndef CAR_STATUS_H
#define CAR_STATUS_H

/* Public includes. */

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Print the status line.
 * Call from one task, about twice a second. Every read is a lock-free copy
 * that never waits. A value that cannot be read prints as 0.
 */
void car_status_print (void);

#endif /* CAR_STATUS_H */
