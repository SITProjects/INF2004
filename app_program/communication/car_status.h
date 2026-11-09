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
#include <stdint.h>

/* Public types, constants, and macros. */

/* Public function declarations. */

/**
 * @brief Read the car's state and pack it for the dashboard.
 * Call from the radio task. Every read is a copy that never waits. A value
 * that cannot be read is sent as 0.
 * @param p_words Destination, ROBOT_STATUS_WORDS long; layout in
 * robot_types.h.
 */
void car_status_encode (uint32_t * p_words);

#endif /* CAR_STATUS_H */
