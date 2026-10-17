/**
 * @file drv_servo.c
 * @brief Servo position control using a 50 Hz PWM signal.
 *
 * The board helpers in bsp/libbsp.h set the pin function, wrap, level and
 * enable; the clock divider has no helper, so it is written directly.
 */
#include "drv_servo.h"
#include <stdint.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

/* Private types, constants, and macros. */
#define PWM_COUNTER_HZ 1000000u
#define PWM_WRAP_VALUE 19999u
#define MIN_PULSE_US 500u
#define MAX_PULSE_US 2500u
#define MAX_ANGLE 180
#define CENTRE_ANGLE 90

/* Bank-0 GPIOs on the RP2040. */
#define GPIO_COUNT 30

/* clk_sys as set by the kernel's start-up code; SYSCLK is in MHz. */
#define SYS_CLOCK_HZ ((uint32_t)SYSCLK * 1000000u)

/*
 * PWM slice registers: slice n starts at PWM_BASE + 0x14 * n. GPIO n
 * drives slice (n / 2) % 8. DIV is fixed point with 4 fraction bits and an
 * integer part from 1 to 255.
 */
#define PWM_SLICE_STRIDE 0x14u
#define PWM_SLICE_COUNT 8u
#define PWM_DIV_FRAC_BITS 4u
#define PWM_DIV_MIN (1u << PWM_DIV_FRAC_BITS)
#define PWM_DIV_LIMIT (256u << PWM_DIV_FRAC_BITS)

/* RESETS: the PWM block's reset bit, and the atomic bit-clear alias. */
#define RESETS_PWM_BIT (1u << 14)
#define RESETS_DONE_OFFSET 0x8u
#define REG_ALIAS_CLR 0x3000u

/* Static data. */
static UINT servo_gpio;

/* Private function declarations. */
static void release_pwm_reset (void);

/* Public function definitions. */

int
servo_init (int servo_pin)
{
    if (servo_pin < 0 || servo_pin >= GPIO_COUNT)
    {
        return 1;
    }

    /* A 1 MHz counter makes each PWM count approximately one microsecond. */
    uint32_t divider = (SYS_CLOCK_HZ << PWM_DIV_FRAC_BITS) / PWM_COUNTER_HZ;
    if (divider < PWM_DIV_MIN || divider >= PWM_DIV_LIMIT)
    {
        return 1;
    }

    servo_gpio = (UINT)servo_pin;
    UW slice_base =
        PWM_BASE + (PWM_SLICE_STRIDE * ((servo_gpio / 2u) % PWM_SLICE_COUNT));

    release_pwm_reset();
    /* As pwm_init() in the Pico SDK: default mode, counter stopped at 0. */
    out_w(slice_base + PWM_CHx_CSR, 0u);
    out_w(slice_base + PWM_CHx_DIV, divider);
    out_w(slice_base + PWM_CHx_CTR, 0u);
    out_w(slice_base + PWM_CHx_CC, 0u);
    (void)pwm_set_wrap(servo_gpio, PWM_WRAP_VALUE);
    servo_set_angle(CENTRE_ANGLE);
    (void)pwm_set_pin(servo_gpio);
    (void)pwm_set_enabled(servo_gpio, TRUE);
    return 0;
}

void
servo_set_angle (int angle)
{
    if (angle < 0)
    {
        angle = 0;
    }
    else if (angle > MAX_ANGLE)
    {
        angle = MAX_ANGLE;
    }

    uint16_t pulse_count =
        MIN_PULSE_US +
        ((uint32_t)angle * (MAX_PULSE_US - MIN_PULSE_US)) / MAX_ANGLE;
    (void)pwm_set_cc(servo_gpio, pulse_count);
}

/* Private function definitions. */

/* Take the PWM block out of reset if nothing has done so yet. */
static void
release_pwm_reset (void)
{
    if ((in_w(RESETS_RESET) & RESETS_PWM_BIT) != 0u)
    {
        out_w(RESETS_RESET + REG_ALIAS_CLR, RESETS_PWM_BIT);
        while ((in_w(RESETS_RESET + RESETS_DONE_OFFSET) & RESETS_PWM_BIT) == 0u)
        {
            /* Wait for the block to come out of reset. */
        }
    }
}
