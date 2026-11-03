/**
 * @file motion_pid.h
 * @brief PID controller with output limits and anti-windup (plan step 5).
 *
 * Pure arithmetic with no hardware access: the wheel speed loops use it
 * now, and the straight-line and line-following loops of later plan steps
 * can reuse it. Each controller is a caller-owned motion_pid_t; the
 * functions keep no other state, so any one task may use its own
 * controllers.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#ifndef MOTION_PID_H
#define MOTION_PID_H

/* Public includes. */
#include <stdbool.h>

/* Public types, constants, and macros. */

/**
 * @brief One controller's gains, limits and working values. Set it up with
 * motion_pid_init(); do not change the fields directly.
 */
typedef struct
{
    float kp;         /**< Output per unit of error. */
    float ki;         /**< Output per unit of error per second. */
    float kd;         /**< Output per unit of error change per second. */
    float out_min;    /**< Lowest output. */
    float out_max;    /**< Highest output. */
    float integral;   /**< Integral term, in output units. */
    float prev_error; /**< Error at the last update, for the D term. */
    bool b_has_prev;  /**< False until the first update after a reset. */
} motion_pid_t;

/* Public function declarations. */

/**
 * @brief Set a controller's gains and output limits and reset it.
 * @param p_pid Caller-owned controller; ignored if NULL.
 * @param kp Proportional gain, 0 or more.
 * @param ki Integral gain, 0 or more, per second.
 * @param kd Derivative gain, 0 or more, in seconds.
 * @param out_min Lowest output; must not be above @p out_max.
 * @param out_max Highest output.
 */
void motion_pid_init (motion_pid_t * p_pid, float kp, float ki, float kd,
                      float out_min, float out_max);

/**
 * @brief Change the output limits, keeping the integral within them.
 * @param p_pid Controller set up by motion_pid_init(); ignored if NULL.
 * @param out_min Lowest output; must not be above @p out_max.
 * @param out_max Highest output.
 */
void motion_pid_set_limits (motion_pid_t * p_pid, float out_min, float out_max);

/**
 * @brief Clear the integral and the derivative history.
 * @param p_pid Controller set up by motion_pid_init(); ignored if NULL.
 */
void motion_pid_reset (motion_pid_t * p_pid);

/**
 * @brief Run one control step.
 * @param p_pid Controller set up by motion_pid_init().
 * @param error Setpoint minus measurement, in the caller's units.
 * @param dt_s Time since the last update, in seconds, above 0.
 * @return kp * error + integral + kd * d(error)/dt, clamped to the output
 * limits; 0 for a NULL controller or @p dt_s of 0 or less. The integral
 * stops growing while the output is at a limit and the error pushes it
 * further (anti-windup). The first update after a reset has no D term.
 */
float motion_pid_update (motion_pid_t * p_pid, float error, float dt_s);

#endif /* MOTION_PID_H */
