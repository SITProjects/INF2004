/**
 * @file obstacle.c
 * @brief Ultrasonic obstacle detection module.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "obstacle.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

/* Private constants. */
#define OBSTACLE_THRESHOLD_CM 5.0f

/* Private state. */
static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_pulse_us = 0;
static volatile bool measurement_ready = false;
static float distance_cm = 0.0f;
static bool obstacle_detected = false;

static int ultrasonic_echo_pin, ultrasonic_trig_pin;

/* Private function declarations. */
static void ultrasonic_isr (void);
static bool obstacle_timer_callback (repeating_timer_t * timer);
/* Public functions. */

int
obstacle_init (int trig_pin, int echo_pin, repeating_timer_t * timer)
{
    ultrasonic_echo_pin = echo_pin;
    ultrasonic_trig_pin = trig_pin;
    /* Trigger pin */
    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    /* Echo pin */
    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);

    /* Capture both edges to measure the echo pulse duration. */
    gpio_add_raw_irq_handler(ultrasonic_echo_pin, ultrasonic_isr);
    gpio_set_irq_enabled(ultrasonic_echo_pin,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* A negative interval schedules callback starts 60 ms apart. */
    if (!add_repeating_timer_ms(-60, obstacle_timer_callback, NULL, timer))
    {
        printf("Failed to start obstacle timer\n");
        return 1;
    }
    return 0;
}

void
obstacle_trigger (int trig_pin, int echo_pin)
{
    /*
     * Send the ultrasonic trigger pulse.
     * Ultrasonic sensor requires a 2us low pulse followed by a 10us high pulse to initiate measurement
     */
    gpio_put(trig_pin, 0);
    busy_wait_us_32(2);

    gpio_put(trig_pin, 1);
    busy_wait_us_32(10);

    gpio_put(trig_pin, 0);
}

bool
obstacle_update (void)
{
    if (!measurement_ready)
    {
        return false;
    }
    bool previous_state = obstacle_detected;

    uint32_t pulse_us = echo_pulse_us;
    measurement_ready = false;

    // Calculate distance in centimeters using the speed of sound (343 m/s)
    distance_cm = ((float)pulse_us * 0.0343f) / 2.0f;
    // Determine if the obstacle is too close based on the threshold
    // TODO: Different threshold level for different motor speed

    // Updates the obstacle detected state based on threshold
    if (distance_cm < OBSTACLE_THRESHOLD_CM)
    {
        obstacle_detected = true;
    }
    else
    {
        obstacle_detected = false;
    }

    // Return true if a new obstacle has just been detected
    return !previous_state && obstacle_detected;
}

bool
is_obstacle_detected (void)
{
    return obstacle_detected;
}

float
obstacle_get_distance_cm (void)
{
    return distance_cm;
}

/* Private functions. */
static void
ultrasonic_isr (void)
{
    // Handle the ultrasonic interrupt service routine
    uint32_t events = gpio_get_irq_event_mask(ultrasonic_echo_pin);

    if (events & GPIO_IRQ_EDGE_RISE)
    {
        echo_start_us = time_us_32();

        gpio_acknowledge_irq(ultrasonic_echo_pin, GPIO_IRQ_EDGE_RISE);
    }

    if (events & GPIO_IRQ_EDGE_FALL)
    {
        uint32_t echo_end_us = time_us_32();

        echo_pulse_us = echo_end_us - echo_start_us;

        measurement_ready = true;

        gpio_acknowledge_irq(ultrasonic_echo_pin, GPIO_IRQ_EDGE_FALL);
    }
}

static bool
obstacle_timer_callback (repeating_timer_t * timer)
{
    (void)timer;
    obstacle_trigger(ultrasonic_trig_pin, ultrasonic_echo_pin);

    return true;
}
