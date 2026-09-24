/**
 * @file servo.c
 * @brief Servo position control using a 50 Hz PWM signal.
 */
#include "servo.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PWM_COUNTER_HZ 1000000u
#define PWM_WRAP_VALUE 19999u
#define MIN_PULSE_US 500u
#define MAX_PULSE_US 2500u
#define MAX_ANGLE 180
#define CENTRE_ANGLE 90

static uint servo_slice;
static uint servo_channel;

int
servo_init (int servo_pin)
{
    if (servo_pin < 0 || servo_pin >= (int)NUM_BANK0_GPIOS)
    {
        return 1;
    }

    /* A 1 MHz counter makes each PWM count approximately one microsecond. */
    float divider = (float)clock_get_hz(clk_sys) / PWM_COUNTER_HZ;
    if (divider < 1.0f || divider >= 256.0f)
    {
        return 1;
    }
    servo_slice = pwm_gpio_to_slice_num(servo_pin);
    servo_channel = pwm_gpio_to_channel(servo_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
    pwm_init(servo_slice, &config, false);
    servo_set_angle(CENTRE_ANGLE);
    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
    pwm_set_enabled(servo_slice, true);
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
    pwm_set_chan_level(servo_slice, servo_channel, pulse_count);
}
