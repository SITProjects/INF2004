/**
 * @file motion_pid.c
 * @brief PID controller with output limits and anti-windup.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "motion_pid.h"
#include <stddef.h>

/* Private types, constants, and macros. */

/* Static data. */

/* Private function declarations. */
static float clamp_float (float value, float minimum, float maximum);

/* Public function definitions. */

void
motion_pid_init (motion_pid_t * p_pid, float kp, float ki, float kd,
                 float out_min, float out_max)
{
    if (NULL == p_pid)
    {
        return;
    }

    p_pid->kp = kp;
    p_pid->ki = ki;
    p_pid->kd = kd;
    p_pid->out_min = out_min;
    p_pid->out_max = out_max;
    motion_pid_reset(p_pid);
}

void
motion_pid_set_limits (motion_pid_t * p_pid, float out_min, float out_max)
{
    if (NULL == p_pid)
    {
        return;
    }

    p_pid->out_min = out_min;
    p_pid->out_max = out_max;
    p_pid->integral = clamp_float(p_pid->integral, out_min, out_max);
}

void
motion_pid_reset (motion_pid_t * p_pid)
{
    if (NULL == p_pid)
    {
        return;
    }

    p_pid->integral = 0.0f;
    p_pid->prev_error = 0.0f;
    p_pid->b_has_prev = false;
}

float
motion_pid_update (motion_pid_t * p_pid, float error, float dt_s)
{
    if ((NULL == p_pid) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    float derivative = 0.0f;
    if (p_pid->b_has_prev)
    {
        derivative = (p_pid->kd * (error - p_pid->prev_error)) / dt_s;
    }
    p_pid->prev_error = error;
    p_pid->b_has_prev = true;

    float proportional = p_pid->kp * error;
    float integral = clamp_float(p_pid->integral + (p_pid->ki * error * dt_s),
                                 p_pid->out_min, p_pid->out_max);
    float output = proportional + integral + derivative;

    /*
     * Anti-windup: hold the integral while the error drives the output
     * further past a limit.
     */
    if (((output > p_pid->out_max) && (error > 0.0f)) ||
        ((output < p_pid->out_min) && (error < 0.0f)))
    {
        output = proportional + p_pid->integral + derivative;
    }
    else
    {
        p_pid->integral = integral;
    }
    return clamp_float(output, p_pid->out_min, p_pid->out_max);
}

/* Private function definitions. */

static float
clamp_float (float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}
