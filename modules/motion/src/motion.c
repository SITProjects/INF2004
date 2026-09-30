/**
 * @file motion.c
 * @brief Differential-drive motion control (open loop).
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "motion.h"
#include <math.h>
#include "motion_config.h"
#include "motor.h"

/* Private types, constants, and macros. */
#define MOTION_PI 3.14159265f
#define DEG_TO_RAD (MOTION_PI / 180.0f)
#define SPEED_EPSILON_MM_S 1.0f

/* Static data. */
static bool b_initialised = false;

/* Private function declarations. */
static void drive_wheel (motor_id_t id, float speed_mm_s);
static void brake_all (void);
static float clamp_float (float value, float minimum, float maximum);

/* Public function definitions. */

motion_status_t
motion_init (void)
{
    if (!motor_init(MOTOR_LEFT, MOTION_LEFT_MOTOR_PIN_A,
                    MOTION_LEFT_MOTOR_PIN_B, MOTION_LEFT_MOTOR_INVERTED) ||
        !motor_init(MOTOR_RIGHT, MOTION_RIGHT_MOTOR_PIN_A,
                    MOTION_RIGHT_MOTOR_PIN_B, MOTION_RIGHT_MOTOR_INVERTED))
    {
        return MOTION_ERR_PARAM;
    }

    brake_all();
    b_initialised = true;
    return MOTION_OK;
}

motion_status_t
motion_set_velocity (int16_t linear_mm_s, int16_t angular_deg_s)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    /* Differential drive: each wheel is half a track width off centre. */
    float turn_mm_s =
        (float)angular_deg_s * DEG_TO_RAD * (MOTION_TRACK_WIDTH_MM * 0.5f);
    drive_wheel(MOTOR_LEFT, (float)linear_mm_s - turn_mm_s);
    drive_wheel(MOTOR_RIGHT, (float)linear_mm_s + turn_mm_s);
    return MOTION_OK;
}

motion_status_t
motion_stop (void)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    brake_all();
    return MOTION_OK;
}

/* Private function definitions. */

/* Open loop: duty = offset to overcome friction + gain * speed. */
static void
drive_wheel (motor_id_t id, float speed_mm_s)
{
    speed_mm_s =
        clamp_float(speed_mm_s, -MOTION_MAX_SPEED_MM_S, MOTION_MAX_SPEED_MM_S);
    if (fabsf(speed_mm_s) < SPEED_EPSILON_MM_S)
    {
        motor_brake(id);
        return;
    }

    float dir = (speed_mm_s > 0.0f) ? 1.0f : -1.0f;
    float duty =
        dir * (MOTION_FF_OFFSET + (MOTION_FF_GAIN * fabsf(speed_mm_s)));
    motor_set_duty(id, clamp_float(duty, -1.0f, 1.0f));
}

static void
brake_all (void)
{
    motor_brake(MOTOR_LEFT);
    motor_brake(MOTOR_RIGHT);
}

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
