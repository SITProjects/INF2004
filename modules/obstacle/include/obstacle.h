/**
 * @file obstacle.h
 * @brief Ultrasonic obstacle detection on the initializing core.
 *
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

/* Public includes. */
#include <stdbool.h>
#include "pico/time.h"

/* Public types, constants, and macros. */

/* Public function declarations. */
/**
 * @brief Initialises the GPIO and ISR service for obstacle module
 * @return 0 on success, 1 on failure
*/
int obstacle_init (int trig_pin, int echo_pin);

/** @brief Trigger on the configured pin; ignore requests while awaiting echo. */
void obstacle_trigger (int trig_pin);

/**
 * @brief Consume the latest echo or expire a measurement after 23 ms.
 * @return True only on transition to a detected obstacle.
 * No valid echo clears detection and sets distance to -1 (unavailable).
 * Call all public functions on the core that called obstacle_init().
 */
bool obstacle_update_check (void);

/** @return True if an obstacle is currently detected. */
bool is_obstacle_detected (void);

/** @return Last distance in cm, or -1 before a reading or after no valid echo. */
float obstacle_get_distance_cm (void);
#endif /* OBSTACLE_H */
