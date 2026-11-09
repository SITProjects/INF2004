/** @file motion_drive.h
 *
 * @brief Public motion-control API.
 *
 * Each wheel's duty is a feed-forward estimate from its speed setpoint
 * plus a PI correction from its measured speed (plan step 5), so speed
 * holds as the battery runs down or the load changes. The caller measures
 * the speeds from the encoders and passes them to motion_drive_control()
 * every MOTION_UPDATE_PERIOD_MS; without those calls the drive is open
 * loop.
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

/** @brief What the drive is asking of each wheel, for logs and tuning. */
typedef struct
{
    int16_t left_setpoint_mm_s;  /**< Left speed setpoint; 0 if braked. */
    int16_t right_setpoint_mm_s; /**< Right speed setpoint; 0 if braked. */
    int16_t left_duty_permille;  /**< Left duty, -1000 to 1000. */
    int16_t right_duty_permille; /**< Right duty, -1000 to 1000. */
} motion_command_t;

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
 * Each wheel's speed is clamped to MOTION_MAX_SPEED_MM_S, and its
 * feed-forward duty applied at once. A wheel whose speed rounds to 0 is
 * braked; one that stops or reverses restarts its PI loop.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_drive_set_velocity (int16_t linear_mm_s,
                                           int16_t angular_deg_s);

/**
 * @brief Run one speed-loop step on each driven wheel.
 * @param left_mm_s Measured left wheel speed in mm/s; + is forward.
 * @param right_mm_s Measured right wheel speed in mm/s; + is forward.
 * Sets each driven wheel's duty to its feed-forward duty plus a PI
 * correction, kept between 0 and full duty in the setpoint's direction.
 * For the first MOTION_SPEED_WINDOW_UPDATES - 1 calls after a wheel
 * starts or reverses, the measured speed still includes the time before,
 * so the wheel gets the feed-forward duty alone. Braked wheels are left
 * alone.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Call every MOTION_UPDATE_PERIOD_MS: the integral assumes that period.
 */
motion_status_t motion_drive_control (int16_t left_mm_s, int16_t right_mm_s);

/**
 * @brief Turn the speed PI loop on (the default) or off.
 * @param b_enable False leaves only the feed-forward duty: for bench
 * comparison of open and closed loop (MOT-U21), not for normal use.
 * @return MOTION_OK or MOTION_ERR_NOT_INIT.
 * Either way both wheels' PI state is reset.
 */
motion_status_t motion_drive_set_speed_pid (bool b_enable);

/**
 * @brief Drop what the wheels' speed loops have built up, keeping the
 *        setpoints: each wheel falls back to its feed-forward duty.
 * @return MOTION_OK, or MOTION_ERR_NOT_INIT.
 * Motion task only.
 */
motion_status_t motion_drive_reset_speed_loops (void);

/**
 * @brief Brake both wheels immediately and reset their PI loops.
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

/**
 * @brief Read each wheel's speed setpoint and the duty last applied.
 * @param p_command Caller-owned snapshot to fill.
 * @return MOTION_OK, MOTION_ERR_PARAM for a NULL pointer, or
 * MOTION_ERR_NOT_INIT.
 */
motion_status_t motion_drive_get_command (motion_command_t * p_command);

#endif /* MOTION_DRIVE_H */
