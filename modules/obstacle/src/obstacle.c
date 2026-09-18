/**
 * @file obstacle.c
 * @brief Ultrasonic obstacle detection module.
 *
 * SPDX-License-Identifier: MIT
 */
#include "obstacle.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

/* Private constants. */
#define OBSTACLE_THRESHOLD_CM 5.0f
#define ECHO_TIMEOUT_US 30000u

/* Private state. */
static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_pulse_us = 0;
static volatile bool measurement_ready = false;
static volatile uint32_t trigger_start_us = 0;
static volatile bool measurement_pending = false;
static volatile bool echo_active = false;
static float distance_cm = -1.0f;
static bool obstacle_detected = false;

static int ultrasonic_echo_pin, ultrasonic_trig_pin;

/* Private function declarations. */
static void ultrasonic_isr (void);
static bool obstacle_timer_callback (repeating_timer_t * timer);
static void expire_measurement (uint32_t now_us);
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
obstacle_trigger (int trig_pin)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t now_us = time_us_32();
    expire_measurement(now_us);
    if (measurement_pending)
    {
        restore_interrupts(irq_state);
        return;
    }

    /* Discard stale edges before starting a new measurement. */
    gpio_acknowledge_irq(ultrasonic_echo_pin,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    trigger_start_us = now_us;
    echo_active = false;
    measurement_pending = true;

    /*
     * Send the ultrasonic trigger pulse.
     * The sensor requires a 2us low pulse followed by a 10us high pulse
     * to initiate measurement.
     */
    gpio_put(trig_pin, 0);
    busy_wait_us_32(2);

    gpio_put(trig_pin, 1);
    busy_wait_us_32(10);

    gpio_put(trig_pin, 0);
    restore_interrupts(irq_state);
}

bool
obstacle_update (void)
{
    /* The GPIO ISR, timer, and caller run on the initializing core. */
    uint32_t irq_state = save_and_disable_interrupts();
    expire_measurement(time_us_32());
    if (!measurement_ready)
    {
        restore_interrupts(irq_state);
        return false;
    }
    uint32_t pulse_us = echo_pulse_us;
    measurement_ready = false;
    restore_interrupts(irq_state);

    bool previous_state = obstacle_detected;

    /* A zero pulse denotes no valid echo, not a zero-distance obstacle. */
    distance_cm = pulse_us ? ((float)pulse_us * 0.0343f) / 2.0f : -1.0f;
    // Determine if the obstacle is too close based on the threshold
    // TODO: Different threshold level for different motor speed

    // Updates the obstacle detected state based on threshold
    if ((distance_cm >= 0.0f) && (distance_cm < OBSTACLE_THRESHOLD_CM))
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
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t events = gpio_get_irq_event_mask(ultrasonic_echo_pin);
    uint32_t now_us = time_us_32();
    gpio_acknowledge_irq(ultrasonic_echo_pin, events);
    expire_measurement(now_us);

    if ((events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) ==
        (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL))
    {
        /* Both edges latched: their timing is unknown. Await the timeout. */
        echo_active = false;
    }
    else if (measurement_pending && (events & GPIO_IRQ_EDGE_RISE))
    {
        echo_start_us = now_us;
        echo_active = true;
    }
    else if (measurement_pending && echo_active &&
             (events & GPIO_IRQ_EDGE_FALL))
    {
        echo_pulse_us = now_us - echo_start_us;
        measurement_ready = true;
        measurement_pending = false;
        echo_active = false;
    }
    restore_interrupts(irq_state);
}

/* Called only with interrupts disabled; subtraction handles clock wrap. */
static void
expire_measurement (uint32_t now_us)
{
    if (measurement_pending &&
        ((uint32_t)(now_us - trigger_start_us) >= ECHO_TIMEOUT_US))
    {
        measurement_pending = false;
        echo_active = false;
        echo_pulse_us = 0;
        measurement_ready = true;
    }
}

static bool
obstacle_timer_callback (repeating_timer_t * timer)
{
    (void)timer;
    obstacle_trigger(ultrasonic_trig_pin);

    return true;
}
