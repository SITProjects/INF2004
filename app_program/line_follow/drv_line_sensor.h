/**
 * @file drv_line_sensor.h
 * @brief Digital DO inputs of the two line IR modules (GP26, GP27).
 */

#ifndef DRV_LINE_SENSOR_H
#define DRV_LINE_SENSOR_H

/* Public includes. */
#include <stdbool.h>

/* Public types, constants, and macros. */

/** @brief One sample of both line sensors. */
typedef struct
{
    bool b_left_black;  /**< Left sensor over the tape. */
    bool b_right_black; /**< Right sensor over the tape. */
} line_sensor_reading_t;

/* Public function declarations. */

/**
 * @brief Make both DO pins SIO inputs with the Schmitt trigger and a
 * pull-down. The modules drive DO themselves; an unplugged pin reads 0
 * (white while LINE_DO_BLACK_LEVEL is 1).
 * @note Call once from the line task before the first read.
 */
void drv_line_sensor_init (void);

/**
 * @brief Read both DO pins.
 * @return The sample; black uses LINE_DO_BLACK_LEVEL.
 * @note Line task only. Never blocks.
 */
line_sensor_reading_t drv_line_sensor_read (void);

#endif /* DRV_LINE_SENSOR_H */
