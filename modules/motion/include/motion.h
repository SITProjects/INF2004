/** @file motion.h
 *
 * @brief Public motion-control API.
 *
 * Open-loop drive. Wheel speeds are estimated from duty cycle, not
 * measured, so actual speed varies with battery level and load.
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MOTION_OK = 0,
    MOTION_ERR_PARAM,
    MOTION_ERR_NOT_INIT
} motion_status_t;

/**
 * @brief Configure both motors, then hold both wheels braked.
 * @return MOTION_OK, or MOTION_ERR_PARAM for an invalid pin or clock
 * configuration in motion_config.h.
 */
motion_status_t motion_init (void);

/**
 * @brief Drive continuously until another call or motion_stop().
 * @param linear_mm_s Forward speed of the chassis centre in mm/s;
 * negative drives backward.
 * @param angular_deg_s Turn rate in degrees per second; positive turns
 * left (counter-clockwise), 0 drives straight.
 * Each wheel's speed is clamped to MOTION_MAX_SPEED_MM_S.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_set_velocity (int16_t linear_mm_s,
                                     int16_t angular_deg_s);

/**
 * @brief Brake both wheels immediately.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_stop (void);

#endif /* MOTION_H */
