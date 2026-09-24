/**
 * @file ultrasonic.h
 * @brief Single-shot ultrasonic measurements on the initializing core.
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>

/** @return 0 on success, 1 for invalid pins. Initialise once before use. */
int ultrasonic_init (int trig_pin, int echo_pin);

/**
 * @brief Start a measurement on the configured trigger pin.
 * @return False while a measurement is pending, unread, or within the minimum
 * ping interval. Call all public functions on the initializing core.
 */
bool ultrasonic_trigger (void);

/**
 * @brief Consume a completed measurement, expiring missing echoes after 23 ms.
 * @return True when a result was consumed; distance is -1 for an invalid echo.
 * Pass a non-null distance pointer. False leaves the output unchanged.
 */
bool ultrasonic_read (float * distance_cm);

#endif /* ULTRASONIC_H */
