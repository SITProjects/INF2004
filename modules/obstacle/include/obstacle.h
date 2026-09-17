/**
 * @file obstacle.h
 * @brief Obstacle module placeholder.
 *
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

/* Public includes. */
#include <stdbool.h>

/* Public types, constants, and macros. */
#include "pico/stdlib.h"
#include "pico/time.h"

/* Public function declarations. */
int obstacle_init (int trig_pin, int echo_pin, repeating_timer_t * timer);

void obstacle_trigger (int trig_pin, int echo_pin);

bool obstacle_update (void);

bool is_obstacle_detected (void);

float obstacle_get_distance_cm (void);
#endif /* OBSTACLE_H */
