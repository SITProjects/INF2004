/** @file motion_drive.h
 *
 * @brief Public motion-control API.
 *
 * Open-loop drive. Wheel speeds are estimated from duty cycle, not
 * measured, so actual speed varies with battery level and load. Wheel
 * encoders are counted but not yet used for control.
 */

#ifndef MOTION_DRIVE_H
#define MOTION_DRIVE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MOTION_OK = 0,
    MOTION_ERR_PARAM,
    MOTION_ERR_NOT_INIT,
    MOTION_ERR_STALLED,  /**< Refused while stalled; stop clears it. */
    MOTION_ERR_TIMED_OUT /**< Refused after a timeout; stop clears it. */
} motion_status_t;

/**
 * @brief Both wheels' encoder counts, read together.
 * Each count wraps at 16 bits (Tribunal 1 minimum-bit decision): use
 * only the change between two reads, (int16_t)(now - prev), never the
 * raw total.
 */
typedef struct
{
    int16_t left_count;  /**< Left encoder edges, mod 2^16; + is forward. */
    int16_t right_count; /**< Right encoder edges, mod 2^16; + is forward. */
} motion_counts_t;

/**
 * @brief Configure both motors and encoders, then hold both wheels braked.
 * @return MOTION_OK, or MOTION_ERR_PARAM for an invalid pin or clock
 * configuration in motion_config.h, PIO0 already running another program,
 * or a repeated call. Each motor is braked once configured, so a failure
 * leaves both wheels still.
 * Call once, from the task that will call motion_drive_get_counts().
 */
motion_status_t motion_drive_init (void);

/**
 * @brief Drive continuously until another call or motion_drive_stop().
 * @param linear_mm_s Forward speed of the chassis centre in mm/s;
 * negative drives backward.
 * @param angular_deg_s Turn rate in degrees per second; positive turns
 * left (counter-clockwise), 0 drives straight.
 * Each wheel's speed is clamped to MOTION_MAX_SPEED_MM_S.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_drive_set_velocity (int16_t linear_mm_s,
                                           int16_t angular_deg_s);

/**
 * @brief Brake both wheels immediately.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_drive_stop (void);

/**
 * @brief Read both wheels' encoder counts.
 * @param p_counts Caller-owned snapshot to fill.
 * @return MOTION_OK, MOTION_ERR_PARAM for a NULL pointer, or
 * MOTION_ERR_NOT_INIT.
 * Call from one task only: reading the encoders empties their FIFOs.
 */
motion_status_t motion_drive_get_counts (motion_counts_t * p_counts);

#endif /* MOTION_DRIVE_H */
