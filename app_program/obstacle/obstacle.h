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

/* Public types, constants, and macros. */

/** Scan task priority; the obstacle band is 12-13 (TEAM.md section 4). */
#define OBSTACLE_PRI_SCAN 12

/** GP2: echo output of the ultrasonic sensor. */
#define OBSTACLE_ECHO_PIN 2

/** GP3: trigger input of the ultrasonic sensor. */
#define OBSTACLE_TRIG_PIN 3

/** GP12: servo signal, PWM slice 6 channel A. */
#define OBSTACLE_SERVO_PIN 12

/** ID of the scan task, which owns the sensor, servo and GPIO interrupt. */
extern ID obstacle_tid_scan;

/* Public function declarations. */

#endif /* OBSTACLE_H */
