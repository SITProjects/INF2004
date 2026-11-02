/**
 * @file motion_drive.c
 * @brief Differential-drive motion control: feed-forward duty plus a speed
 * PI loop on each wheel.
 *
 * Copyright (c) 2026 10x Slackers
 * SPDX-License-Identifier: MIT
 */

#include "motion_drive.h"
#include <math.h>
#include <stddef.h>
#include "motion_config.h"
#include "motion_pid.h"
#include "drv_encoder.h"
#include "drv_motor.h"

/* Private types, constants, and macros. */
#define SPEED_EPSILON_MM_S 1.0f

/* Multiply degrees by this to get radians. */
#define DEG_TO_RAD (3.14159265f / 180.0f)

/* Time between motion_drive_control() calls, in seconds. */
#define CONTROL_PERIOD_S ((float)MOTION_UPDATE_PERIOD_MS / 1000.0f)

/* Duty fraction to tenths of a percent, for motion_drive_get_command(). */
#define DUTY_TO_PERMILLE 1000.0f

typedef struct
{
    float setpoint_mm_s; /* 0 while braked. */
    float correction;    /* Last PI output, added to the feed-forward. */
    float duty;          /* Duty last applied. */
    uint32_t hold;       /* Updates left before the PI loop may run. */
    motion_pid_t pid;
} wheel_control_t;

/* Static data. */
static bool b_initialised = false;
static bool b_speed_pid = true;
static wheel_control_t wheels[MOTOR_COUNT];

/* Private function declarations. */
static void set_wheel (motor_id_t id, float speed_mm_s);
static void control_wheel (motor_id_t id, float measured_mm_s);
static void limit_correction (motor_id_t id, float ff);
static void reset_wheel (motor_id_t id);
static void apply_duty (motor_id_t id, float duty);
static float feed_forward (float speed_mm_s);
static bool is_driving (motor_id_t id);
static void brake_all (void);
static float clamp_float (float value, float minimum, float maximum);

/* Public function definitions. */

motion_status_t
motion_drive_init (void)
{
    if (b_initialised ||
        !motor_init(MOTOR_LEFT, MOTION_LEFT_MOTOR_PIN_A,
                    MOTION_LEFT_MOTOR_PIN_B, MOTION_LEFT_MOTOR_INVERTED) ||
        !motor_init(MOTOR_RIGHT, MOTION_RIGHT_MOTOR_PIN_A,
                    MOTION_RIGHT_MOTOR_PIN_B, MOTION_RIGHT_MOTOR_INVERTED) ||
        !encoder_init(ENCODER_LEFT, MOTION_LEFT_ENCODER_PIN_A,
                      MOTION_LEFT_ENCODER_PIN_B,
                      MOTION_LEFT_ENCODER_INVERTED) ||
        !encoder_init(ENCODER_RIGHT, MOTION_RIGHT_ENCODER_PIN_A,
                      MOTION_RIGHT_ENCODER_PIN_B,
                      MOTION_RIGHT_ENCODER_INVERTED))
    {
        return MOTION_ERR_PARAM;
    }

    for (uint32_t id = 0u; id < (uint32_t)MOTOR_COUNT; id++)
    {
        motion_pid_init(&wheels[id].pid, MOTION_SPEED_KP, MOTION_SPEED_KI, 0.0f,
                        -1.0f, 1.0f);
    }
    brake_all();
    b_initialised = true;
    return MOTION_OK;
}

motion_status_t
motion_drive_set_velocity (int16_t linear_mm_s, int16_t angular_deg_s)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    /* Differential drive: each wheel is half a track width off centre. */
    float turn_mm_s =
        (float)angular_deg_s * DEG_TO_RAD * (MOTION_TRACK_WIDTH_MM * 0.5f);
    set_wheel(MOTOR_LEFT, (float)linear_mm_s - turn_mm_s);
    set_wheel(MOTOR_RIGHT, (float)linear_mm_s + turn_mm_s);
    return MOTION_OK;
}

motion_status_t
motion_drive_control (int16_t left_mm_s, int16_t right_mm_s)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    control_wheel(MOTOR_LEFT, (float)left_mm_s);
    control_wheel(MOTOR_RIGHT, (float)right_mm_s);
    return MOTION_OK;
}

motion_status_t
motion_drive_set_speed_pid (bool b_enable)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    b_speed_pid = b_enable;
    for (uint32_t id = 0u; id < (uint32_t)MOTOR_COUNT; id++)
    {
        motion_pid_reset(&wheels[id].pid);
        wheels[id].correction = 0.0f;
    }
    return MOTION_OK;
}

motion_status_t
motion_drive_stop (void)
{
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    brake_all();
    return MOTION_OK;
}

motion_status_t
motion_drive_get_counts (motion_counts_t * p_counts)
{
    if (NULL == p_counts)
    {
        return MOTION_ERR_PARAM;
    }
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    p_counts->left_count = encoder_get_count(ENCODER_LEFT);
    p_counts->right_count = encoder_get_count(ENCODER_RIGHT);
    return MOTION_OK;
}

motion_status_t
motion_drive_get_command (motion_command_t * p_command)
{
    if (NULL == p_command)
    {
        return MOTION_ERR_PARAM;
    }
    if (!b_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }

    p_command->left_setpoint_mm_s = (int16_t)wheels[MOTOR_LEFT].setpoint_mm_s;
    p_command->right_setpoint_mm_s = (int16_t)wheels[MOTOR_RIGHT].setpoint_mm_s;
    p_command->left_duty_permille =
        (int16_t)(wheels[MOTOR_LEFT].duty * DUTY_TO_PERMILLE);
    p_command->right_duty_permille =
        (int16_t)(wheels[MOTOR_RIGHT].duty * DUTY_TO_PERMILLE);
    return MOTION_OK;
}

/* Private function definitions. */

/*
 * Set one wheel's speed and apply its feed-forward duty at once, plus the
 * PI correction it had. Starting or reversing restarts the PI loop: its
 * integral belongs to the old direction. The loop then waits until the
 * speed window holds only updates since the start, so it does not wind
 * the integral up on the stopped or reversed speeds still in the window.
 */
static void
set_wheel (motor_id_t id, float speed_mm_s)
{
    wheel_control_t * p_wheel = &wheels[id];

    speed_mm_s =
        clamp_float(speed_mm_s, -MOTION_MAX_SPEED_MM_S, MOTION_MAX_SPEED_MM_S);
    if (fabsf(speed_mm_s) < SPEED_EPSILON_MM_S)
    {
        reset_wheel(id);
        motor_brake(id);
        return;
    }

    if ((speed_mm_s * p_wheel->setpoint_mm_s) <= 0.0f)
    {
        motion_pid_reset(&p_wheel->pid);
        p_wheel->correction = 0.0f;
        p_wheel->hold = MOTION_SPEED_WINDOW_UPDATES - 1u;
    }
    p_wheel->setpoint_mm_s = speed_mm_s;

    float ff = feed_forward(speed_mm_s);
    limit_correction(id, ff);
    apply_duty(id, ff + p_wheel->correction);
}

/*
 * One speed-loop step for a driven wheel; a braked wheel is left alone.
 * The duty stays between 0 and full in the setpoint's direction, so the
 * loop slows a wheel by easing off, never by reversing it.
 */
static void
control_wheel (motor_id_t id, float measured_mm_s)
{
    wheel_control_t * p_wheel = &wheels[id];

    if (!is_driving(id))
    {
        return;
    }

    float ff = feed_forward(p_wheel->setpoint_mm_s);
    if (p_wheel->hold > 0u)
    {
        p_wheel->hold--;
    }
    else if (b_speed_pid)
    {
        limit_correction(id, ff);
        p_wheel->correction = motion_pid_update(
            &p_wheel->pid, p_wheel->setpoint_mm_s - measured_mm_s,
            CONTROL_PERIOD_S);
    }
    apply_duty(id, ff + p_wheel->correction);
}

/*
 * Limit a wheel's PI loop, and the correction it last gave, so that @p ff
 * plus the correction stays between 0 and full duty in the setpoint's
 * direction. A new setpoint's smaller feed-forward could otherwise add to
 * an old negative correction and reverse the wheel for one update.
 */
static void
limit_correction (motor_id_t id, float ff)
{
    wheel_control_t * p_wheel = &wheels[id];
    float minimum = -ff;
    float maximum = 1.0f - ff;

    if (p_wheel->setpoint_mm_s < 0.0f)
    {
        minimum = -1.0f - ff;
        maximum = -ff;
    }
    motion_pid_set_limits(&p_wheel->pid, minimum, maximum);
    p_wheel->correction = clamp_float(p_wheel->correction, minimum, maximum);
}

/* Forget one wheel's setpoint, duty and PI state. */
static void
reset_wheel (motor_id_t id)
{
    wheels[id].setpoint_mm_s = 0.0f;
    wheels[id].correction = 0.0f;
    wheels[id].duty = 0.0f;
    wheels[id].hold = 0u;
    motion_pid_reset(&wheels[id].pid);
}

static void
apply_duty (motor_id_t id, float duty)
{
    wheels[id].duty = clamp_float(duty, -1.0f, 1.0f);
    motor_set_duty(id, wheels[id].duty);
}

/* Open-loop duty: offset to overcome friction + gain * speed. */
static float
feed_forward (float speed_mm_s)
{
    float dir = (speed_mm_s > 0.0f) ? 1.0f : -1.0f;

    return clamp_float(
        dir * (MOTION_FF_OFFSET + (MOTION_FF_GAIN * fabsf(speed_mm_s))), -1.0f,
        1.0f);
}

static bool
is_driving (motor_id_t id)
{
    return fabsf(wheels[id].setpoint_mm_s) >= SPEED_EPSILON_MM_S;
}

static void
brake_all (void)
{
    reset_wheel(MOTOR_LEFT);
    reset_wheel(MOTOR_RIGHT);
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
