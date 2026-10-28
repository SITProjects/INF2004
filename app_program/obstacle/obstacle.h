/**
 * @file obstacle.h
 * @brief Obstacle subsystem: an ultrasonic sensor on a servo.
 *        GitHub: Kitsuneez.
 *
 * This header is the obstacle subsystem's PUBLIC interface: only what other
 * subsystems may use. Everything else stays private in the .c files. Other
 * subsystems include it as "../obstacle/obstacle.h".
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

/* Public includes. */
#include "../app.h"
#include "obstacle_config.h"

/* Public types, constants, and macros. */

/** ID of the scan task, which owns the sensor, servo and GPIO interrupt. */
extern ID obstacle_tid_scan;

/* Public function declarations. */

#endif /* OBSTACLE_H */
